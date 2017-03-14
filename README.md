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
## Settings File
In uarm_metal/src you'll find a file settings.txt where you can set what data is read from the robot (the alternative is programatically setting this using ROS parameters in the console or in code, see below).

For example, here the settings file is set up to read the XYZ coordinates (read_pos, 1), and read digital inputs 1 & 3 (read_digital_inputs,1,1,3). For reading digital inputs or analog inputs, you pass a list of numbers where the first digit indicates On/Off and the following digits indicate the specific pins you want.
```
ros_hz, 1000.0,
read_pos,1,
read_ja,0,
read_analog_inputs,0,7,8,
read_digital_inputs,0,1,3,
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
![alt text](https://cloud.githubusercontent.com/assets/9334388/23900016/748ca194-08af-11e7-8bcc-203d5d1ebde6.png)

## uarm_metal Custom Message Types

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

### Terminal Examples
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
### rospy Examples
e.g. Using rospy to turn off reading joint angles
```
rospy.set_param('/uarm_metal/read_joint_angles', 1)
```

# Notes
## Performance
Performance will vary depending on your system. The figures here are based on testing in a Ubuntu 14.04 virtual machine with ROS Indigo (3Gb RAM), on an i7 Windows 10 laptop.

Reading exclusively cartesian end-effector coordinates from the robot will produce a ~50-60Hz update rate. With each
additional topic that is read, or rapidly written to (e.g. reading position and joint angles at the same time), will
result in a drop in communication rate. This drop in communication rate is due to the communications bottle neck with the uarm control board not allowing simultaneous read/writes. 

Reading XYZ coordinates and Joint Angles simultaneously gives an update rate of ~28-30Hz.

Reading XYZ, Joint Angles, and two analog pins gives an update rate of ~17-18Hz.

Reading Position, and one analog input will give a read rate to ~35Hz.

Reading Joint Angles while also writing Joint Angles@30Hz will give a read rate of ~42Hz.

## Issues
While this uarm_metal package manages read/write requests to avoid communications conflicts, there still appears to be some firmware bugs on the uArm.

These bugs have been identified by other uArm users and appear something like this:
```
pyuarm - ERROR - Communication| ERROR: send #100 P200
```
Unfortunately there is no way to get out of this error once it has started; however this package will catch this error and shut down the node. As a work around, if the node is started from a launch file with the respawn parameter set to true, the node will restart and reconnect to the uArm.

```
<launch>
	<node name="uarm_metal" pkg="uarm_metal" type="uarm.py" respawn="true"/>
</launch>
```

From this url [https://forum.ufactory.cc/t/pyuarm-usb-errors-and-skipping-moves/567/2] it appears to be something that will be fixed in upcoming updates; however uFactory have stated pyuarm will move to Python3 exclusively. This move to Python3 may prove probelematic for this package in future...

# Example: Record and Playback Node
An example of a node offering record and playback functionality is found in ../examples/ROS_record_play.py. To run the example, start up the main uarm_metal node uarm.py and then:
```
rosrun uarm_metal ROS_record_play.py
```
If you cannot find the file from rosrun make sure you have set permissions for it (sudo chmod a+x ROS_record_play.py.

![alt text](https://cloud.githubusercontent.com/assets/9334388/23900017/749e97aa-08af-11e7-9f06-50a6cd7acb4e.png)

# TODO
The main TODO on the list will be to try allow for RViz integration like the original ROS package. Hopefully I can get around to this over the next month or two.
