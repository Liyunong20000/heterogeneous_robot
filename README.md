# heterogeneous_robot

ROS packages for heterogeneous robotic systems.

## ScorpioX

`robots/scorpiox` integrates a CHAMP-compatible Spot base with a Kinova Gen3
7-DOF arm and optional Robotiq 2F-85 gripper. It also contains standalone
Kortex keyboard control and the Gazebo door-opening demo.

Build the package from the Catkin workspace root:

```bash
wstool merge -t src src/heterogeneous_robot/${ROS_DISTRO}.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
catkin build scorpiox
source devel/setup.bash
```

Display the combined model in RViz:

```bash
roslaunch scorpiox spot_kortex_rviz.launch
```

Run the combined Gazebo simulation and Spot teleoperation:

```bash
roslaunch scorpiox spot_kortex_gazebo.launch
roslaunch scorpiox spot_kortex_teleop.launch
```

Use the CHAMP outdoor world with:

```bash
roslaunch scorpiox spot_kortex_gazebo.launch outdoor_environment:=true
```

Run the standalone Kortex door-opening demo with:

```bash
roslaunch scorpiox kortex_open_door_demo.launch
```

Run standalone Kortex keyboard control with:

```bash
rosrun scorpiox kortex_keyboard_control.py
```
