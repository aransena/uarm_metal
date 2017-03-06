#!/usr/bin/env python
import signal
import time
from UARM_interface import UARM_interface

def shutdown_signal_handler(signal, frame):
    global exit
    exit = True
    print 'Shutting Down!'

def main():
    uarm_interface = UARM_interface()
    uarm_interface.connect()
    uarm_interface.load_parameters()
    uarm_interface.start_threads()
    return uarm_interface

if __name__ == '__main__':
    global exit
    uarm_interface = main()
    signal.signal(signal.SIGINT, shutdown_signal_handler)
    print 'Ctrl+C to exit'
    exit = False
#    signal.pause()
    while uarm_interface.alive and exit is False:
        time.sleep(1)
    
    uarm_interface.shutdown()

