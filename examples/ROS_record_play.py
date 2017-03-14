#!/usr/bin/env python

### Author: Aran Sena
### Email: aransena_at_gmail_dot_com
### Date: March 2017
### License: MIT
### Version: 1.0
### Description: Example code showing how to use the uarm_metal ROS package to create
###              record and playback functionality. You can set whether playback is done
###              in joint space ('JA') or cartesian space ('POS') using the
###              playback_type global.
import rospy
from pynput import keyboard
import threading
import time
from std_msgs.msg import Bool
from uarm_metal.msg import Beep, Position, JointAngles


global rec_data
global data_sub
global beeps
global playback_type
global publishers


def data_callback(data):
    global rec_data
    # build up recorded data
    # TODO Size/Time limit?
    rec_data.append(data)

def start_record():
    global rec_data
    global data_sub
    global publishers
    global playback_type

    # initialize rec data
    rec_data = []

    # detach for pbd
    publishers['attach'].publish(Bool(False))

    # collect data from requested source
    if playback_type == 'POS':
        data_sub = rospy.Subscriber("/uarm_metal/position_read", Position, data_callback)
    elif playback_type == 'JA':
        data_sub = rospy.Subscriber("/uarm_metal/joint_angles_read", JointAngles, data_callback)

    return data_sub


def start_playback(data):
    global publishers
    # attach robot motors for playback
    publishers['attach'].publish(Bool(True))
    # send playback data in a separate thread so as not to lock up interactions
    # pass rec_data as arg to the thread
    pub_thread = threading.Thread(target=play, args=(data))
    pub_thread.daemon = True
    pub_thread.start()

    return data_pub


def play(*args):
    global publishers
    global beeps

    # publish each point in the rec_data to the pub topic
    rate = rospy.Rate(100)
    for point in args:
        publishers['data'].publish(point)
        rate.sleep()


def stop_ROS_sub(sub):
    try:
        # stop receiving data when idle
        sub.unregister()
    except Exception as exc:
        print "Error in stop_ROS_processes(): ", exc
        pass


def process_key(key):
    global beeps
    global rec_data
    global publishers
    global data_sub

    # process user input
    if key.char == '1':
        data_sub = start_record()
        print "\nRECORD"
    elif key.char == '2':
        print "\nSTOP"
        publishers['attach'].publish(Bool(False))
        stop_ROS_sub(data_sub)
        print "\nRecorded ", len(rec_data), " points."
    elif key.char == '3':
        if len(rec_data) > 0:
            print "\nPLAY"
            publishers['beep'].publish(beeps['good'])
            start_playback(rec_data)
        else:
            print "\nNo data..."
            publishers['beep'].publish(beeps['bad'])
    elif key.char == '4':
        rec_data = []
        publishers['beep'].publish(beeps['good'])
        print "\nRESET"


def on_press(key):
    # catch key input with pynput
    if key == keyboard.Key.esc:
        raise Exception(key)
    else:
        try:
            key.char
            process_key(key)
        except:
            # print "Key cannot be processed"
            pass

if __name__ == '__main__':
    global rec_data
    global publishers
    global beeps
    global data_sub
    global playback_type
    # Options:
    # POS - data playback using robot recorded end-effector positions (tends to be jerky)
    # JA - data playback using servo recorded joint angles (tends to be a bit smoother)
    playback_type = 'JA'
    if playback_type == 'JA':
        print "Mode: Joint Angle record and playback"
    else:
        print "Mode: End Effector XYZ coordinates record and playback"
    rec_data = []
    publishers = {}
    beeps = {}
    data_sub = None

    # Setup some beep sounds
    bp_good = Beep()
    bp_good.frequency = 12000
    bp_good.duration = 0.1
    beeps['good'] = bp_good

    bp_bad = Beep()
    bp_bad.frequency = 6000
    bp_bad.duration = 0.2
    beeps['bad'] = bp_bad

    # Setup playback writer depending on playback type desired
    if playback_type == 'POS':
        data_pub = rospy.Publisher('/uarm_metal/position_write', Position, queue_size=10)
    elif playback_type == 'JA':
        data_pub = rospy.Publisher('/uarm_metal/joint_angles_write', JointAngles, queue_size=10)

    # setup writers for attaching/ beeping
    att_pub = rospy.Publisher('/uarm_metal/attach', Bool, queue_size=10)
    beep_pub = rospy.Publisher('/uarm_metal/beep', Beep, queue_size=10)

    # store publishers in a dict for easy access in program
    publishers['data'] = data_pub
    publishers['attach'] = att_pub
    publishers['beep'] = beep_pub

    # setup uarm metal to broadcast the data required depending on playback type
    if playback_type == 'POS':
        ja_r = 0
        pos_r = 1
    else:
        ja_r = 1
        pos_r = 0
    rospy.set_param('/uarm_metal/read_joint_angles', ja_r)
    rospy.set_param('/uarm_metal/read_position', pos_r)

    ## start ROS node
    rospy.init_node('record_node', anonymous=False)

    # interaction through pynput library, esc to exit
    print "\nSTART\nesc: Exit.\n1: Start Recording (detach) \n2: Stop Record (detach) " \
          "\n3: Playback recorded data (attach) \n4: Delete recorded data"
    with keyboard.Listener(on_press=on_press) as listener:
        try:
            listener.join()
        except Exception as e:
            pass