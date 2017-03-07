#!/usr/bin/env python
import signal
import time
import rospy
from UarmMetal import UarmMetal
from std_msgs.msg import String


def shutdown_signal_handler(signal, frame):
    global exit
    exit = True
    print 'Shutting Down!'

if __name__ == '__main__':
    global exit
    uarm_interface = UarmMetal()
    uarm_interface.connect()

    signal.signal(signal.SIGINT, shutdown_signal_handler)
    print 'Ctrl+C to exit'
    exit = False

    pub = rospy.Publisher('/uarm_write', String, queue_size=10)

    i = 0
    write_rate = 30.0
    while uarm_interface.alive and exit is False:
        if uarm_interface.ready:
            if i == 0:
                pub.publish("ATT")
            msg = "JA"+str(i)
            pub.publish(msg)
            i += 4
            if i > 45:
                i = 1

        time.sleep(1.0/write_rate)

# program exit