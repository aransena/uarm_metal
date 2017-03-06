#!/usr/bin/env python
import signal
import time
from UARM_interface import UARM_interface

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

    while uarm_interface.alive and exit is False:
        time.sleep(1)

    if uarm_interface.alive:
        uarm_interface.shutdown()

