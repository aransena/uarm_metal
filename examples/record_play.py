#!/usr/bin/env python
import signal
import time
import rospy
from std_msgs.msg import String

global recording
global active
global play
global robot_data
global reset
global stop


def shutdown_signal_handler(signal, frame):
    global exit
    exit = True
    print 'Shutting Down!'

def ctl_callback(data):
    global recording
    global active
    global play
    global stop
    global reset
    msg = data.data
    if msg == "RECORD":
        recording = True
        play = False
    elif msg == "STOP":
        recording = False
        stop = True
        play = False
    elif msg == "PLAY":
        recording = False
        play = True
    elif msg == "RESET":
        recording = False
        play = False
        reset = True

def data_callback(data):
    global robot_data
    robot_data = data.data


if __name__ == '__main__':
    global exit
    global robot_data
    global recording
    global play
    global stop

    stop = False
    reset = False
    robot_data = None
    play = False
    recording = False
    active = False

    signal.signal(signal.SIGINT, shutdown_signal_handler)
    print 'Ctrl+C to exit'
    exit = False

    pub = rospy.Publisher('/uarm_write', String, queue_size=10)
    rospy.Subscriber("/uarm_read", String, data_callback)
    rospy.Subscriber("/recorder", String, ctl_callback)
    rospy.init_node('record_play_node', anonymous=True)

    rw_rate = 10.0
    rec_data = []
    start = True
    while exit is False:
        if recording:
            if start:
                pub.publish("DET")
                pub.publish("BEEP")
                start = False
            rec_data.append(robot_data)

        elif play:
            if start:
                pub.publish("BEEP")
                start = False
            if rec_data:
                data = rec_data.pop()
                print data, len(data)
                if len(data) == 3:
                    msg = "POS" + str(data[0]) + "," + str(data[1]) + "," + str(data[2])
                    pub.publish(msg)
                elif len(data) == 4:
                    msg = "JA" + str(data[0]) + "," + str(data[1]) + "," + str(data[2]) + "," + str(data[3])
                    pub.publish(msg)

            else:
                pub.publish("BEEP")
                pub.publish("BEEP")
                play = False
                start = True

        elif stop:
            pub.publish("BEEP")
            start = True
            stop = False

        elif reset:
            rec_data = []

        time.sleep(1.0 / rw_rate)

