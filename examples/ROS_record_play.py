#!/usr/bin/env python
import signal
import time
import rospy

# http://stackoverflow.com/questions/36714728/pip-install-getch-clang-error
import getch # pip install https://pypi.python.org/packages/source/g/getch/getch-1.0-python2.tar.gz#md5=586ea0f1f16aa094ff6a30736ba03c50

from std_msgs.msg import String, Bool
from uarm_metal.msg import JointAngles, Beep

def shutdown_signal_handler(signal, frame):
    global exit
    exit = True
    print 'Shutting Down!'

def data_callback(data):
    #print data
    pass

def process(key):
    print key
    if key == '1':
        pass
    elif key == '2':
        pass
    elif key == '3':
        pass
    elif key == '4':
        pass
    elif key == 'q':
        global exit
        exit = True
        print 'Shutting Down!'

if __name__ == '__main__':
    signal.signal(signal.SIGINT, shutdown_signal_handler)
    print 'Ctrl+C or q to exit'
    exit = False
    rospy.set_param('/uarm_metal/read_joint_angles', 1)
    rospy.set_param('/uarm_metal/read_position', 0)
    rospy.set_param('/uarm_metal/read_digital_inputs', [0,0])
    rospy.set_param('/uarm_metal/read_analog_inputs', [0,0])

    ja_pub = rospy.Publisher('/uarm_metal/joint_angles_write', JointAngles, queue_size=10)
    att_put = rospy.Publisher('/uarm_metal/attach', Bool, queue_size=10)
    beep_pub = rospy.Publisher('/uarm_metal/beep', Beep, queue_size=10)

    rospy.Subscriber("/uarm_metal/joint_angles_read", JointAngles, data_callback)
    rospy.init_node('record_play_node', anonymous=True)

    rw_rate = 1000.0
    rec_data = []
    play_data = []
    start = True
    while exit is False:

        key = getch.getch()
        process(key)
        time.sleep(1.0 / rw_rate)

