# uarm_metal
uArm Metal ROS interface package

# Setup
Requires: 

pynput [https://pypi.python.org/pypi/pynput]

python-xlib [https://pypi.python.org/pypi/python-xlib]

pyuarm [https://github.com/uArm-Developer/pyuarm] -  make sure to follow set up instructions here as well.

# Use
Setup a catkin workspace and clone this repo into src as a package. Run catkin make. Source your workspace.

Make sure python files are given required permissions (sudo chmod a+x uarm.py)

To start the interface:
 ```
rosrun uarm_metal uarm.py
```

## ROS Topics
There are 11 ROS topics that can be used to interact with the robot.

Due to the serial link to the robot meaning we cannot read/write rapidly or concurrently, the package is designed to
have only the required topics active at any one time and manages the read/write process to avoid conflicts.
As more topics become active, there will be more read/write requests to the robot, thus reducing the communication
speed. Topic activity versus communication performance is
discussed further later.

Topics:
```
/uarm_metal/attach <- std_msgs/Bool <- True/False attach/detach
/uarm_metal/beep <- uarm_metal/Beep
/uarm_metal/pump <- std_msgs/Bool <- True/False On/Off
/uarm_metal/joint_angles_read <- uarm_metal/JointAngles <- Subscribe to this to read robot joint angles
/uarm_metal/joint_angles_write <- uarm_metal/JointAngles <- Publish to this to control robot (must first attach)
/uarm_metal/position_read <- uarm_metal/Position <- Subscribe to this to read robot end effector coordinates
/uarm_metal/position_write <- uarm_metal/Position <- Publish to this to control robot (must first attach)
/uarm_metal/analog_inputs_read <- std_msgs/String <- Subscribe to this to read robot analog input pins as CSV strings
/uarm_metal/digital_inputs_read <- std_msgs/String <- Subscribe to this to read robot digital input pins as CSV strings
/uarm_metal/string_read <- Unused
/uarm_metal/string_write <- Unused
```

## uarm_metal Custom Data Types

### uarm_metal/Position
```
float32 x
float32 z
float32 y
```
### uarm_metal/JointAngles
```
float32 j0
float32 j1
float32 j2
float32 j3
```
### uarm_metal/Beep
```
float32 frequency
float32 duration
```

## ROS Params
There are 4 rosparams you can set through either the terminal or in code.

```
/uarm_metal/read_analog_inputs
/uarm_metal/read_digital_inputs
/uarm_metal/read_joint_angles
/uarm_metal/read_position
```

read_analog_inputs and read_digital_inputs take lists as input, where the first digit indicates Off/On [0/1] and the
following comma separated digits indicate the relevant pin you wish to measure.

e.g. to measure analog pins 7 & 8

```
rosparam set /uarm_metal/read_analog_inputs "[1, 7, 8]"
```

The analog data for these pins will then be published as a csv std_msgs/String on /uarm_metal/analog_inputs_read.

read_joint_angles and read_position are set using an int to indicate Off/On [0/1]

e.g. to read joint angles from the uArm

```
rosparam set /uarm_metal/read_joint_angles 1
```


# Notes
## Performance
Reading exclusively cartesian end-effector coordinates from the robot will produce a ~70Hz update rate. With each
additional topic that is read, or rapidly written to (e.g. reading position and joint angles at the same time), will
result in a drop in communication rate.

For example, reading joint angles, and two analog inputs will drop the read rate to ~20Hz.
