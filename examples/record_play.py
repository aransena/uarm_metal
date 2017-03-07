#!/usr/bin/env python
import signal
import time
import rospy
from UarmMetal import UarmMetal
from std_msgs.msg import String
from uarm_decorators import *

global recording
global active
global play
global robot_data
global reset



def shutdown_signal_handler(signal, frame):
    global exit
    exit = True
    print 'Shutting Down!'

def ctl_callback(data):
    global recording
    global active
    global play
    global reset
    msg = data.data
    if msg == "RECORD":
        recording = True
        play = False
    elif msg == "STOP":
        recording = False
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
    reset = False
    robot_data = None
    play = False
    recording = False
    active = False

    uarm_interface = UarmMetal()
    uarm_interface.connect()

    signal.signal(signal.SIGINT, shutdown_signal_handler)
    print 'Ctrl+C to exit'
    exit = False

    pub = rospy.Publisher('/uarm_write', String, queue_size=10)
    rospy.Subscriber("/uarm_read", String, data_callback)
    rospy.Subscriber("/recorder", String, ctl_callback)

    rw_rate = 10.0
    rec_data = []
    while uarm_interface.alive and exit is False:
        if uarm_interface.ready:
            if recording:
                rec_data.append(robot_data)

            elif play:
                if rec_data:
                    print rec_data.pop()

            elif reset:
                rec_data = []

        time.sleep(1.0 / rw_rate)

