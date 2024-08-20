# UR5 Lego Project

This project aims to control a UR5 robotic arm to move megablocks from one position to another. The robot can be simulated or real.
<br>
This project requires `locosim` to initialize the simulation and the real robot, so make sure you have it installed and
that it is set-up correctly, you should be able to run ur5_generic.py (see below) and the robot should move to the homing position.


## Starting the project

First, start the simulation by calling the script ur5_generic.py:
```bash
python locosim/robot_control/base_controllers/ur5_generic.py
```
The robot should move to the homing position.
<br>
Then you can start the robot controller with:
```bash
roslaunch ur5lego ur5lego.launch
```


## Moving the robot

You can easily move the robot in the cartesian space by using the move_wrapper.bash script:
```bash
./scripts/move_wrapper.bash <x> <y> <z> <roll> <pitch> <yaw> <time>
```
If you want to control it from code you wrote yourself, you can call the `move_server` action server with the same arguments as above.
This will come with less overhead and the possibility to wait until the robot has finished moving.


## Spawning blocks

To spawn blocks in the simulation, you can use the spawn_block.bash script:
```bash
./scripts/spawn_block.bash block_name:=<block_name> block_type:=X1-Y1-Z2 spawn_x:=<x> spawn_y:=<y> spawn_z:=<z>
```
The name of the block can be any string as long as it is not the name of another object in the simulation, the block types are listed under blocks_description/


## Automatic Grasping

Grasping in the simulation is functional but may have some bugs due to limitations in Gazebo. To grasp a block, line up the gripper with the block and set the angles of the gripper to approximately -0.08, you can do so by calling the script:
```bash
./scripts/move_wrapper.bash -0.08
```
If you want to control it from code you wrote yourself you can call the `gripper_server` action server with the same arguments as above.


## Manual grasping
If the grasping causes too many issues, you can still grasp the blocks by using a more hard-coded approach by running:
```bash
./scripts/connect_links.bash <block_name> <connect>
```
where `<block_name>` is the name of the block you want to grasp and `<connect>` is either `true` or `false` depending on whether you want to grasp or ungrasp the block.
<br>
This will create or destroy a link between the gripper and the block, meaning that it should be used only when the gripper is already in its grasping position to avoid silly results.  
With this approach there is no need to open and close the gripper and it is not reccomended to do so as the block will probably start glitching as it was doing with the automatic grasping method


## Real robot

To use the real robot, you need to change two flags: one is the `REAL_ROBOT` flag that must be set to `true` in the `movement/position_publisher.cpp` file, and the other one is the `real_robot` flag under `locosim/robot_control/base_controllers/params.py`, inside the configuration of the ur5, which must be set to `True` as well.
Then, follow the startup procedure for the real robot, which is described in the `locosim` readme.
Once the robot has moved to the homing position, you can control it the same way as the simulation
