#!/usr/bin/env python
import rospy
from pynput import keyboard
import threading

from std_msgs.msg import Bool
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
    global publishers
    global ns
    rec_data = []

    publishers['attach'].publish(Bool(False))

    data_sub = rospy.Subscriber(ns + "joint_angles_read", JointAngles, data_callback)
    return data_sub

def start_playback(data):
    global publishers
    publishers['attach'].publish(Bool(True))
    pub_thread = threading.Thread(target=play, args=(data))
    pub_thread.daemon = True
    pub_thread.start()

    return data_pub

def play(*args):
    global publishers
    global beeps
    rate = rospy.Rate(20)
    for point in args:
        publishers['data'].publish(point)
        rate.sleep()

    publishers['attach'].publish(Bool(False))
    publishers['beep'].publish(beeps['bad'])


def stop_ROS_sub(sub):
    try:
        #rospy.signal_shutdown("Stop Rec/Play")
        sub.unregister()
    except Exception as e:
        print "Error in stop_ROS_processes(): ", e
        pass

def process_key(key):
    global beeps
    global rec_data
    global publishers
    global data_sub

    if key.char == '1':
        data_sub = start_record()
        print "RECORD"
    elif key.char == '2':
        print "STOP"
        publishers['attach'].publish(Bool(False))
        stop_ROS_sub(data_sub)
        print len(rec_data)
    elif key.char == '3':
        print "PLAY"
        publishers['beep'].publish(beeps['good'])
        if len(rec_data) > 0:
            start_playback(rec_data)
    elif key.char == '4':
        rec_data = []
        publishers['beep'].publish(beeps['bad'])
        print "RESET"

def on_press(key):
    if key == keyboard.Key.esc:
        raise Exception(key)
    else:
        try:
            key.char
            process_key(key)
        except:
            #print "Key cannot be processed"
            pass

if __name__ == '__main__':
    global rec_data
    global publishers
    global beeps
    global data_sub
    global ns

    data_sub = None
    beeps = {}
    bp_good = Beep()
    bp_good.frequency = 12000
    bp_good.duration = 0.1
    beeps['good'] = bp_good

    bp_bad = Beep()
    bp_bad.frequency = 6000
    bp_bad.duration = 0.2
    beeps['bad'] = bp_bad

    ns = "/uarm_metal/"
    data_pub = rospy.Publisher(ns + 'joint_angles_write', JointAngles, queue_size=10)
    att_pub = rospy.Publisher(ns + 'attach', Bool, queue_size=10)
    beep_pub = rospy.Publisher(ns + 'beep', Beep, queue_size=10)
    publishers = {}
    publishers['data'] = data_pub
    publishers['attach'] = att_pub
    publishers['beep'] = beep_pub

    rec_data = []
    ns = "/uarm_metal/"
    rospy.set_param(ns + 'read_joint_angles', 1)
    rospy.set_param(ns + 'read_position', 0)

    rospy.init_node('record_node', anonymous=False)

    print "Start. Press esc key to exit."
    with keyboard.Listener(on_press=on_press) as listener:
        try:
            listener.join()
        except Exception as e:
            pass
            #print "Error", e


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

