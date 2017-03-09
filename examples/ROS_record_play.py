#!/usr/bin/env python
import signal
import rospy
from pynput import keyboard
import threading

from uarm_metal.msg import JointAngles, Beep


global rec_data
global data_sub

def shutdown_signal_handler(signal, frame):
    global exit
    exit = True
    print 'Shutting Down!'

def data_callback(data):
    global rec_data
    rec_data.append(data)

def start_record():
    global rec_data
    global data_sub
    rec_data = []
    ns = "/uarm_metal/"

    data_sub = rospy.Subscriber(ns + "joint_angles_read", JointAngles, data_callback)
    return data_sub

def start_playback(data):
    global data_pub

    ns = "/uarm_metal/"
    data_pub = rospy.Publisher(ns + 'joint_angles_write', JointAngles, queue_size=10)

    pub_thread = threading.Thread(target=play, args=(data, data_pub,))
    pub_thread.daemon = True
    pub_thread.start()

    return data_pub

def play(data, pub):
    for point in data:
        pub.publish(data)

    print "Done!"


def stop_ROS_sub(sub):
    try:
        #rospy.signal_shutdown("Stop Rec/Play")
        sub.unregister()
    except Exception as e:
        print "Error in stop_ROS_processes(): ", e
        pass

def on_press(key):
    global rec_data
    global data_sub

    if key == keyboard.Key.esc:
        raise Exception(key)
    elif key.char == '1':
        start_record()
        print "RECORD"
    elif key.char == '2':
        print "STOP"
        stop_ROS_sub(data_sub)
        print len(rec_data)
    elif key.char == '3':
        print "PLAY"
        if len(rec_data) > 0:
            playback(rec_data)
    elif key.char == '4':
        rec_data = []
        print "RESET"

if __name__ == '__main__':
    global rec_data
    rec_data = []
    ns = "/uarm_metal/"
    rospy.set_param(ns + 'read_joint_angles', 1)
    rospy.set_param(ns + 'read_position', 0)

    rospy.init_node('record_node', anonymous=False)

    print "Start."
    with keyboard.Listener(on_press=on_press) as listener:
        try:
            listener.join()
        except Exception as e:
            print "Error", e


    #
    # record = False
    # play = False
    # stop = False
    # reset = False
    # rec_data = []
    #
    # signal.signal(signal.SIGINT, shutdown_signal_handler)
    # print 'Ctrl+C or q to exit'
    # exit = False
    # ns = "/uarm_metal/"
    # rospy.set_param(ns + 'read_joint_angles', 1)
    # rospy.set_param(ns + 'read_position', 0)
    # rospy.set_param(ns + 'read_digital_inputs', [0,0])
    # rospy.set_param(ns + 'read_analog_inputs', [0,0])
    #
    # rospy.init_node('record_play_node', anonymous=False)
    #
    # ja_pub = rospy.Publisher(ns + 'joint_angles_write', JointAngles, queue_size=10)
    # att_pub = rospy.Publisher(ns + 'attach', Bool, queue_size=10)
    # beep_pub = rospy.Publisher(ns + 'beep', Beep, queue_size=10)
    #
    # rospy.Subscriber(ns + "joint_angles_read", JointAngles, data_callback)
    #
    #
    # rw_rate = 1000.0
    # rate = rospy.Rate(100)
    # rec_data = []
    # play_data = []
    # start = True
    #
    # bp = Beep()
    # bp.frequency = 12000
    # bp.duration = 0.1
    #
    # while exit is False:
    #
    #     key = getch.getch()
    #     process(key)
    #
    #     if record:
    #         #att_pub.publish(False)
    #         if start:
    #             print "Record"
    #             att_pub.publish(Bool(False))
    #             print "Beep"
    #             beep_pub.publish(bp)
    #             start = False
    #
    #     elif play:
    #         if start:
    #             print "play"
    #             play_data = rec_data[:]
    #             att_pub.publish(Bool(True))
    #             beep_pub.publish(bp)
    #             start = False
    #         try:
    #             ja_pub.publish(play_data.pop(0))
    #         except Exception as e:
    #             print e
    #             beep_pub.publish(bp)
    #             play = False
    #
    #     elif stop:
    #         print "stop"
    #         att_pub.publish(Bool(False))
    #         beep_pub.publish(bp)
    #         start = True
    #         stop = False
    #
    #     rate.sleep()
        #time.sleep(1.0 / rw_rate)

