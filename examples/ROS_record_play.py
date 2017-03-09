#!/usr/bin/env python
import signal
import rospy

# http://stackoverflow.com/questions/36714728/pip-install-getch-clang-error
import getch # pip install https://pypi.python.org/packages/source/g/getch/getch-1.0-python2.tar.gz#md5=586ea0f1f16aa094ff6a30736ba03c50

from std_msgs.msg import String, Bool
from uarm_metal.msg import JointAngles, Beep


global record
global play
global stop
global reset

def shutdown_signal_handler(signal, frame):
    global exit
    exit = True
    print 'Shutting Down!'

def data_callback(data):
    global rec_data
    global record
    if record:
        #print data
        rec_data.append(data)


def process(key):
    global record
    global play
    global stop
    global reset
    global rec_data

    if key == '1':
        print "RECORD"
        record = True
        stop = False
        play = False
        reset = False

    elif key == '2':
        print "STOP"
        stop = True
        record = False
        play = False
        reset = False

    elif key == '3':
        print "PLAY"
        play = True
        stop = False
        record = False
        reset = False

    elif key == '4':
        print "RESET"
        reset = True
        stop = True
        record = False
        play = False
        rec_data = []

    elif key == 'q':
        global exit
        exit = True
        print 'Shutting Down!'

if __name__ == '__main__':
    global record
    global play
    global stop
    global reset
    global rec_data

    record = False
    play = False
    stop = False
    reset = False
    rec_data = []

    signal.signal(signal.SIGINT, shutdown_signal_handler)
    print 'Ctrl+C or q to exit'
    exit = False
    ns = "/uarm_metal/"
    rospy.set_param(ns + 'read_joint_angles', 1)
    rospy.set_param(ns + 'read_position', 0)
    rospy.set_param(ns + 'read_digital_inputs', [0,0])
    rospy.set_param(ns + 'read_analog_inputs', [0,0])

    rospy.init_node('record_play_node', anonymous=False)

    ja_pub = rospy.Publisher(ns + 'joint_angles_write', JointAngles, queue_size=10)
    att_pub = rospy.Publisher(ns + 'attach', Bool, queue_size=10)
    beep_pub = rospy.Publisher(ns + 'beep', Beep, queue_size=10)

    rospy.Subscriber(ns + "joint_angles_read", JointAngles, data_callback)


    rw_rate = 1000.0
    rate = rospy.Rate(100)
    rec_data = []
    play_data = []
    start = True

    bp = Beep()
    bp.frequency = 12000
    bp.duration = 0.1

    while exit is False:

        key = getch.getch()
        process(key)

        if record:
            #att_pub.publish(False)
            if start:
                print "Record"
                att_pub.publish(Bool(False))
                print "Beep"
                beep_pub.publish(bp)
                start = False

        elif play:
            if start:
                print "play"
                play_data = rec_data[:]
                att_pub.publish(Bool(True))
                beep_pub.publish(bp)
                start = False
            try:
                ja_pub.publish(play_data.pop(0))
            except Exception as e:
                print e
                beep_pub.publish(bp)
                play = False

        elif stop:
            print "stop"
            att_pub.publish(Bool(False))
            beep_pub.publish(bp)
            start = True
            stop = False

        rate.sleep()
        #time.sleep(1.0 / rw_rate)

