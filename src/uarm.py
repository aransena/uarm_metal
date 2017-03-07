#!/usr/bin/env python
import signal
import time
import rospy
from UARM_interface import UARM_interface
from std_msgs.msg import String


def shutdown_signal_handler(signal, frame):
    global exit
    exit = True
    print 'Shutting Down!'

if __name__ == '__main__':
    global exit
    uarm_interface = UARM_interface()
    uarm_interface.connect()

    signal.signal(signal.SIGINT, shutdown_signal_handler)
    print 'Ctrl+C to exit'
    exit = False

    pub = rospy.Publisher('/uarm_write', String, queue_size=10)

    i = 0
    while uarm_interface.alive and exit is False:
        if uarm_interface.ready:
            #msg = "JA"+str(i)+","
            #pub.publish(msg)
            i += 1
            if i > 45:
                i = 0

        time.sleep(1)

# program exit