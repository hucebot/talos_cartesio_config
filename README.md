# talos_cartesio_config

Package of CartesI/O configurations and launch files to run Inverse Kinematics (IK) and Inverse Dynamics (ID) based on OpenSoT on TALOS robot from PAL Robotics. 

The ```stack``` folder contains the files with the stacks for the IK and ID. These files can be modified to play with different tasks, constraints, solvers, and parameters. 

Whole-Body Inverse Kinematics:
------------------------------
To run the IK just run on the terminal:

```reset && mon launch talos_cartesio_config cartesio.launch```

The interactive marker permits sending direct pose commands or way-points to the controlled end-effectors (to be enabled by right-clicking on the marker). To change the end-effector change the ```Update Topic``` field in the ```InteractiveMarker``` display or add a new interactive marker. The sliders permit to send desired joint positions to the postural task.

Whole-Body Inverse Dynamics:
------------------------------
To run the ID just run on the terminal:

```reset && mon launch talos_cartesio_config cartesio.launch stack:=simple_id```

Interactive markers and sliders work as in the IK case.

Pose script:
------------
Through CarteSI/O Python API is possible to send way-points using a script to the OpenSoT defined tasks.
An example is present in the ```python``` folder. To run it just run:

```python cartesio_simple_poses.py```

within the folder. The left arm will execute 4 waypoints starting from the actual pose. Notice that the same script will work both using the IK and the ID stack.
