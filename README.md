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

Due to the serial link to the robot meaning we cannot read/write rapidly or concurrently, the package is designed to have only the required topics active at any one time. As more topics become active, there will be more read/write requests to the robot, thus reducing the communication speed. Topic activity versus communication performance is discussed further later.

Topics:
```
/uarm_metal/attach <- Publish Bool to this to attach/detach
/uarm_metal/beep <- Publish Beep message type to this to beep
/uarm_metal/pump <- Publish Bool to this to turn On/Off
/uarm_metal/joint_angles_read <- Subscribe to this to read robot joint angles as JointAngle type
/uarm_metal/joint_angles_write <- Publish JointAngle type messages to this to control robot (must first attach)
/uarm_metal/position_read <- Subscribe to this to read robot end effector coordinates as Position type
/uarm_metal/position_write <- Publish Position type messages to this to control robot (must first attach)
/uarm_metal/analog_inputs_read <- Subscribe to this to read robot analog input pins as CSV std_msgs/String type
/uarm_metal/digital_inputs_read <- Subscribe to this to read robot digital input pins as CSV std_msgs/String type
/uarm_metal/string_read <- Unused
/uarm_metal/string_write <- Unused
```

## ROS Params
There are 4 rosparams you can set through either the terminal or in code.

```
/uarm_metal/read_analog_inputs
/uarm_metal/read_digital_inputs
/uarm_metal/read_joint_angles
/uarm_metal/read_position
```

read_analog_inputs and read_digital_inputs take lists as input, where the first digit indicates Off/On [0/1] and the following comma separated digits indicate the relevant pin you wish to measure.

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


# Description
