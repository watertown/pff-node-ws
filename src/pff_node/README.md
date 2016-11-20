# 1. Workspace Setup

### 1.1 Install ROS

[Install ROS Kinetic]( http://wiki.ros.org/kinetic/Installation/Ubuntu)

### 1.2 Install Dependencies

```bash
sudo apt-get install ros-kinetic-teleop-twist-keyboard
```

### 1.3 Get Package & Build

```bash
git clone https://github.com/watertown/pff-ros-ws.git
cd pff-ros-ws;
catkin_make;
. devel/setup.bash
```

# 2. Node API

## 2.1 Topics

### 2.1.1 ROS Publishers

| Topic | Msg | Description |
|:---:|:---:|:---:|
| `"/odom"` | `nav_msgs/Odometry` | Publish transform from `/odom` link to `/base_link` |
| `"/joint_states"` | `sensor_msgs/JointState` | Publish joint transform from parent->child link |

### 2.1.2 ROS Subscribers

| Topic | Msg | Description |
|:---:|:---:|:---:|
| `"~select_mode"` | `std_msgs/String` | Select mode {circle/square} |
| `"/cmd_vel"` | `geometry_msgs/Twist` | Commanded velocity to the mobile base |


### 2.1.3 ROS Parameters

| Param | Type | Default | Description |
|:---:|:---:|:---:|:---:|
| `~mode` | string | `circle` | Mode |
| `~circle_diameter` | double | `1.0` | Diameter of Circle Path (m) |
| `~square_side_length` | double | `1.0` | Square Path Side Length (m) |
| `~autonomous` | bool | `true` | Robot will start following path if true |

### 2.1.4 roslaunch

```xml
<?xml version="1.0"?>
<launch>
    <node name="pff_node"
          pkg="pff_node"
          type="pff_node"
          output="screen"
          required="true">
      <rosparam command="load" file="$(find pff_node)/config/config.yaml" />
    </node>
    <node name="teleop_twist_keyboard"
          pkg="teleop_twist_keyboard"
          type="teleop_twist_keyboard.py"
          output="screen"
          required="false">
    </node>
</launch>
```

# 3 Examples

### 3.1.1 Run node with default parameters

```bash
roslaunch pff_node pff_node.launch
```

### 3.1.2 View in rviz

```bash
roslaunch pff_node rviz_sim.launch
```


```bash
roslaunch pff_node keyboard_teleop.launch
```

### 3.1.4 Selecting Mode ("circle" or "square")

##### Square Mode:

```bash
rostopic pub --once /pff_node/select_mode std_msgs/String square
```

##### Circle Mode:

```bash
rostopic pub --once /pff_node/select_mode std_msgs/String circle
```

<!-- 1) Package Functionality Checklist
- [x] Declares dependencies (if you use other ROS packages)
  - [CMakeLists](src/pff_node/CMakeListst.txt)
  - [package.xml](src/pff_node/CMakeListst.txt)
- [x] Contains documentation about the launchers and parameters if any.
  - [Package README](src/pff_node/README.md)
- [x] Can be built with catkin_make
- [x] Contains the URDF description of a simple differential drive robot
  - [URDF File](src/pff_node/config/diff_robot.urdf)
- [x] Contains a launcher to view the robot model in RViz
  - [URDF File](src/pff_node/config/diff_robot.urdf)
- [ ] OPTIONAL: Contains a launcher to simulate the robot in Gazebo and allows for the following functionality via launcher parameters:
- [x] Keyboard teleop mode, where the robot motion can be commanded by the keyboard.
  - using `teleop_twist_keyboard` package.
  - [Package README](src/pff_node/README.md)
- [x] Circle mode, where the robot drives incessantly along a circle of a user-defined diameter (passed as a parameter in m)
  - `roslaunch pff_node pff_node.launch _mode:=circle`

- [x] Square mode, where the robot drives incessantly along a square of user-defined side-length (passed as a parameter in m)
  - `roslaunch pff_node pff_node.launch _mode:=square`
-->
