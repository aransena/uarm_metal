#!/usr/bin/env python
import signal
import rospy
import time

from uarm_metal.msg import Beep


def shutdown_signal_handler(signal, frame):
    global exit
    exit = True
    print 'Shutting Down!'

if __name__ == '__main__':
    global exit
    signal.signal(signal.SIGINT, shutdown_signal_handler)
    print 'Ctrl+C to exit'
    exit = False

    ns = "/uarm_metal/"

    rospy.init_node('record_play_node', anonymous=False)
    beep_pub = rospy.Publisher(ns + 'beep', Beep, queue_size=10)

    rate = rospy.Rate(20)

    bp = Beep()
    bp.frequency = 12000
    bp.duration = 0.1

    while exit is False:
        beep_pub.publish(bp)
        rate.sleep()