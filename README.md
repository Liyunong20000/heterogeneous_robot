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

The Gazebo launch also starts OptiTrack input for rigid bodies `11` (torso)
and `12` (hand). OptiTrack poses are published in `mocap_world` and transformed
to Gazebo's `world` frame before the mocap controller uses them. The calibration
parameters describe the pose of `mocap_world` in `world` and use radians:

```bash
roslaunch scorpiox spot_kortex_gazebo.launch \
  mocap_world_x:=0.0 mocap_world_y:=0.0 mocap_world_z:=0.0 \
  mocap_world_yaw:=0.0 mocap_world_pitch:=0.0 mocap_world_roll:=0.0
```

For planar calibration, keep mocap tracking disabled and record a torso point
`p1`. Move it along the physical direction that should become Gazebo `+X` and
record `p2`, without changing the mocap setup. With
`d = p2 - p1`, use `mocap_world_yaw:=-atan2(d.y, d.x)`. If a reference mocap
point `p_m` should be Gazebo position `p_g`, use
`t = p_g - R(mocap_world_yaw) p_m` for `mocap_world_x/y`; relative teleoperation
does not require the origins to coincide.

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
