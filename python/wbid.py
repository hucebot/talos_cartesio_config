import os

import rospkg
from xbot2_interface import pyxbot2_interface as xbi
from pyopensot.tasks.acceleration import Cartesian, CoM, DynamicFeasibility, Postural
from pyopensot.constraints.acceleration import JointLimits, VelocityLimits
from pyopensot.constraints.force import FrictionCone
import pyopensot as pysot
import numpy as np

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, WrenchStamped
import tf
import subprocess
from ttictoc import tic,toc

try:
    package_path = rospkg.RosPack().get_path('talos_cartesio_config')
except:
    print("To run this example is needed the talos_cartesio_config package that can be download here: hhttps://github.com/hucebot/talos_cartesio_config")

launch_path = package_path + "/launch/talos.launch"
print(launch_path)
roslaunch = subprocess.Popen(['roslaunch', launch_path], stdout=subprocess.PIPE, shell=False)
rviz_file_path = package_path + "/launch/wbid.rviz"
print(rviz_file_path)
rviz = subprocess.Popen(['rviz',  '-d', f'{rviz_file_path}'], stdout=subprocess.PIPE, shell=False)

# Initiliaze node and wait for robot_description parameter
rospy.init_node("LittleDog_id", disable_signals=True)
while not rospy.has_param('/robot_description'):
    pass

# Get robot description parameter and initialize model interface (with Pinocchio)
urdf = rospy.get_param('/robot_description')
model = xbi.ModelInterface2(urdf)
qmin, qmax = model.getJointLimits()
dqmax = model.getVelocityLimits()

print(model.getJointNames())
q = [ 0., 0., 0., # reference base linear
      0., 0., 0., 1., # refernce base quaternion
      0., 0., -0.69, 1.33, -0.64, 0., # leg_left_1_joint, leg_left_2_joint, leg_left_3_joint, leg_left_4_joint, leg_left_5_joint, leg_left_6_joint
      0., 0., -0.69, 1.33, -0.64, 0., # leg_right_1_joint, leg_right_2_joint, leg_right_3_joint, leg_right_4_joint, leg_right_5_joint, leg_right_6_joint
      0., 0.027, # torso_1_joint, torso_2_joint
     -0.034, 0.31, 0.03, -1.03, 0.004, 0.03, 0.025, # arm_left_1_joint, arm_left_2_joint, arm_left_3_joint arm_left_4_joint, arm_left_5_joint, arm_left_6_joint, arm_left_7_joint
     -0.034, -0.31, 0.03, -1.03, 0.004, 0.03, 0.025, # arm_right_1_joint, arm_right_2_joint, arm_right_3_joint arm_right_4_joint, arm_right_5_joint, arm_right_6_joint, arm_right_7_joint
      -0.027, 0.] # head_1_joint, head_2_joint

dq = np.zeros(model.nv)
model.setJointPosition(q)
model.setJointVelocity(dq)
model.update()

dt = 1./500.

# Instantiate Variables: qddot and contact forces (3 per contact)
contact_frames = ["left_foot_upper_right", "left_foot_lower_right", "left_foot_upper_left", "left_foot_lower_left",
                  "right_foot_upper_right", "right_foot_lower_right", "right_foot_upper_left", "right_foot_lower_left"]
variables_vec = dict()
variables_vec["qddot"] = model.nv
for contact_frame in contact_frames:
    variables_vec[contact_frame] = 3
variables = pysot.OptvarHelper(variables_vec)

# Creates tasks cand constraints
com = CoM(model, variables.getVariable("qddot"))
com.setLambda(1.)
com_ref, vel_ref, acc_ref = com.getReference()
com0 = com_ref.copy()

base = Cartesian("base", model, "world", "base_link", variables.getVariable("qddot"))
base.setLambda(1.)

contact_tasks = list()
cartesian_contact_tasks_frames = ["left_sole_link", "right_sole_link"]
for cartesian_contact_task_frame in cartesian_contact_tasks_frames:
    contact_tasks.append(Cartesian(cartesian_contact_task_frame, model, cartesian_contact_task_frame, "world", variables.getVariable("qddot")))

posture = Postural(model, variables.getVariable("qddot"))

stack = (0.1*com + 0.1*(base%[3, 4, 5]))
force_variables = list()
for i in range(len(cartesian_contact_tasks_frames)):
    stack = stack + 10.*(contact_tasks[i])

for i in range(len(contact_frames)):
    force_variables.append(variables.getVariable(contact_frames[i]))

# Creates the stack.
# Notice:  we do not need to keep track of the DynamicFeasibility constraint so it is created when added into the stack.
# The same can be done with other constraints such as Joint Limits and Velocity Limits
stack = (pysot.AutoStack(stack)/posture) << DynamicFeasibility("floating_base_dynamics", model, variables.getVariable("qddot"), force_variables, contact_frames)
stack = stack << JointLimits(model, variables.getVariable("qddot"), qmax, qmin, 10.*dqmax, dt)
stack = stack << VelocityLimits(model, variables.getVariable("qddot"), dqmax, dt)
for i in range(len(contact_frames)):
    T = model.getPose(contact_frames[i])
    mu = (T.linear, 0.8) # rotation is world to contact
    stack = stack << FrictionCone(contact_frames[i], variables.getVariable(contact_frames[i]), model, mu)

# Creates the solver
solver = pysot.iHQP(stack)

# ID loop: we publish also joint position, floating-base pose and contact forces
rate = rospy.Rate(1./dt)
pub = rospy.Publisher('joint_states', JointState, queue_size=1)
msg = JointState()
msg.name = model.getJointNames()[1::]
br = tf.TransformBroadcaster()
w_T_b = TransformStamped()
w_T_b.header.frame_id = "world"
w_T_b.child_frame_id = "base_link"
force_msg = list()
fpubs = list()
for contact_frame in contact_frames:
    force_msg.append(WrenchStamped())
    force_msg[-1].header.frame_id = contact_frame
    force_msg[-1].wrench.torque.x = force_msg[-1].wrench.torque.y = force_msg[-1].wrench.torque.z = 0.
    fpubs.append(rospy.Publisher(contact_frame, WrenchStamped, queue_size=1))

t = 0.
alpha = 0.4
while not rospy.is_shutdown():
    tic()
    # Update actual position in the model
    model.setJointPosition(q)
    model.setJointVelocity(dq)
    model.update()

    # Compute new reference for CoM task
    com_ref[2] = com0[2] + alpha * np.sin(3.1415 * t)
    com_ref[1] = com0[1] + alpha * np.cos(3.1415 * t)
    t = t + dt
    com.setReference(com_ref)

    # Update Stack
    stack.update()

    # Solve
    x = solver.solve()
    ddq = variables.getVariable("qddot").getValue(x) # from variables vector we retrieve the joint accelerations
    q = model.sum(q, dq*dt + 0.5 * ddq * dt * dt) # we use the model sum to account for the floating-base
    dq += ddq*dt

    # Publish joint states
    msg.position = q[7::]
    msg.header.stamp = rospy.get_rostime()

    w_T_b.header.stamp = msg.header.stamp
    w_T_b.transform.translation.x = q[0]
    w_T_b.transform.translation.y = q[1]
    w_T_b.transform.translation.z = q[2]
    w_T_b.transform.rotation.x = q[3]
    w_T_b.transform.rotation.y = q[4]
    w_T_b.transform.rotation.z = q[5]
    w_T_b.transform.rotation.w = q[6]

    br.sendTransformMessage(w_T_b)

    for i in range(len(contact_frames)):
        T = model.getPose(contact_frames[i])
        force_msg[i].header.stamp = msg.header.stamp
        f_local = T.linear.transpose() @ variables.getVariable(contact_frames[i]).getValue(x) # here we compute the value of the contact forces in local frame from world frame
        force_msg[i].wrench.force.x = f_local[0]
        force_msg[i].wrench.force.y = f_local[1]
        force_msg[i].wrench.force.z = f_local[2]
        fpubs[i].publish(force_msg[i])


    pub.publish(msg)
    #print(toc())

    rate.sleep()

roslaunch.kill()
rviz.kill()