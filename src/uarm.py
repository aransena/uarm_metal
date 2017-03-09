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
    startup_success = uarm_interface.connect()
    if startup_success:
        signal.signal(signal.SIGINT, shutdown_signal_handler)
        print 'Ctrl+C to exit'
        exit = False

        while uarm_interface.alive and exit is False:
            time.sleep(1.0)
    else:
        print "Startup error! Check robot connection."

# program exit