#!/usr/bin/env python
import time
import threading
import os
import pyuarm
import rospy
import ThreadSafePriorityQueue as tspq
from uarm_decorators import *

from std_msgs.msg import String


class UARM_interface():

    def __init__(self):
        self.uarm = None
        self.connected = False
        self.alive = True

        self.ros_rate = None
        self.ros_hz = 0
        self.read_pos = None
        self.read_ja = None
        self.read_AI = None

        self.settings = []

        self.parameter_monitor_thread = None

        self.uarm_read_thread = None
        self.uarm_read_pub = None
        self.rq = tspq.ThreadSafePriorityQueue("read_queue")

        self.uarm_interface_thread = None
        self.iq = tspq.ThreadSafePriorityQueue("interface_queue")

        self.playback_thread = None
        self.playback_active = False
        self.playback_data = []
        self.loading = False

        self.ja = []

    def get_setting(self, name, string_val=False, list_type=False):
        for line in self.settings:
            if line[0] == name:
                if string_val:
                    return line[1]
                else:
                    if list_type:
                        try:
                            return map(float, line[1:-1])
                        except Exception as e:
                            print e
                    else:
                        return float(line[1])

    def connect(self):
        try:
            self.uarm = pyuarm.get_uarm()
            self.connected = True
            self.load_parameters()
            self.start_threads()
            rospy.loginfo("Connected")
        except Exception as e:
            self.connected = False
            err = str(e.message)
            rospy.logerr("Connection error: " + err)
            rospy.signal_shutdown("Connection error")

    def shutdown(self):
        rospy.loginfo("Shutdown function call")
        rospy.signal_shutdown("Shutdown function call")
        start_shutdown = time.time()
        while self.uarm_interface_thread.is_alive() or self.uarm_read_thread.is_alive():
            self.iq.send_to_queue("SHUTDOWN", priority=1)
            self.rq.send_to_queue("SHUTDOWN", priority=1)
            time.sleep(0.5)
            if time.time()-start_shutdown > 5:
                break
        self.alive = False
        self.uarm.disconnect()

    def load_parameters(self):
        rospy.loginfo("Loading parameters")

        dir_path = os.path.dirname(os.path.realpath(__file__))
        f = open(dir_path + "/settings.txt")
        self.settings = []
        for line in f:
            self.settings.append(line.split(','))
        f.close()

        self.ros_hz = self.get_setting("ros_hz")
        self.read_pos = self.get_setting("read_pos")
        self.read_ja = self.get_setting("read_ja")
        self.read_AI = self.get_setting("read_AI", list_type=True)

        rospy.set_param('uarm_metal/read_position', self.read_pos)
        rospy.set_param('uarm_metal/read_joint_angles', self.read_ja)
        rospy.set_param('uarm_metal/read_AI', self.read_AI)

        self.uarm_read_pub = rospy.Publisher('uarm_read', String, queue_size=10)
        rospy.Subscriber("uarm_write", String, self.uarm_write_callback)
        rospy.init_node('uarm_node', anonymous=True)
        self.ros_rate = rospy.Rate(self.ros_hz)

        rospy.loginfo("Done loading parameters.")

    def parameter_monitor(self):
        rospy.loginfo("Parameter monitor running.")
        while True and (rospy.is_shutdown() is False):
            self.read_pos = rospy.get_param('uarm_metal/read_position')
            self.read_ja = rospy.get_param('uarm_metal/read_joint_angles')
            self.read_AI = rospy.get_param('uarm_metal/read_AI')
            time.sleep(0.5)

        rospy.loginfo("Parameter monitor shutdown")

    def start_threads(self):
        rospy.loginfo("Starting threads")
        if self.connected is True:
            self.uarm_read_thread = threading.Thread(target=self.uarm_read)
            self.uarm_interface_thread = threading.Thread(target=self.uarm_interface)
            self.parameter_monitor_thread = threading.Thread(target=self.parameter_monitor)

            self.uarm_read_thread.daemon = True
            self.uarm_read_thread.start()

            self.uarm_interface_thread.daemon = True
            self.uarm_interface_thread.start()

            self.parameter_monitor_thread.daemon = True
            self.parameter_monitor_thread.start()

            self.request_detach()

        else:
            rospy.logerr("Startup error")
            rospy.signal_shutdown("Startup error")

    def get_read_data(self):
        msg = []
        if self.read_pos > 0:
            msg.append(self.read_position())
        if self.read_ja > 0:
            msg.append(self.read_joint_angles())
        if self.read_AI[0] > 0:
            for i in self.read_AI[1:]:
                msg.append(self.read_analog(i))
        return msg

    def uarm_interface(self):
        rospy.loginfo("uarm_interface running")
        while True and (rospy.is_shutdown() is False):
            request = self.iq.get_from_queue()
            if request == "READ":
                curr_vals = self.get_read_data()
                try:
                    self.rq.send_to_queue(curr_vals)
                except Exception as e:
                    print "Error: ", e
                    rospy.logerr("Error writing to queue. Queue probably shutdown.")
                    break

                # try:
                #     self.ja = curr_vals[1]
                # except Exception as e:
                #     err = str(e.message)
                #     rospy.logerr("uArm not ready! " + err)
                #     rospy.signal_shutdown("Error reading from uArm")
                print "here"
                if self.playback_active is False and self.loading is False:
                    print "there"
                    self.iq.send_to_queue("READ")

            elif request == "SHUTDOWN":
                break
            else:
                print "request: ", request
                self.process_command(request)
                if self.playback_active is False and self.loading is False:
                    time.sleep(1)
                    self.iq.send_to_queue("READ")

        rospy.loginfo("uarm_interface shutdown")
        self.shutdown()

    def request_detach(self):
        rospy.loginfo("uArm detach")
        self.iq.send_to_queue("DET")

    def request_attach(self):
        rospy.loginfo("uArm attach")
        self.iq.send_to_queue("ATT")

    def pump_on(self):
        rospy.loginfo("uArm Pump On")
        self.iq.send_to_queue("PUMP_ON")

    def pump_off(self):
        rospy.loginfo("uArm Pump Off")
        self.iq.send_to_queue("PUMP_OFF")

    @ros_try_catch
    def read_analog(self, pin_num):
        return round(self.uarm.get_analog(pin_num), 3)

    @ros_try_catch
    def get_joint_angles(self):
        return self.uarm.get_servo_angle()

    @ros_try_catch
    def read_position(self):
        return self.uarm.get_position()

    @ros_try_catch
    def read_joint_angles(self):
        return self.get_joint_angles()

    def uarm_read(self):
        rospy.loginfo("uarm_read running")
        while True and (rospy.is_shutdown() is False):
            robot_values = self.rq.get_from_queue(blocking=False)
            print "robot vals: ", robot_values
            time.sleep(1)
            if robot_values == "SHUTDOWN":
                rospy.logwarn("sending shutdown signal")
                rospy.signal_shutdown("Normal Shutdown Procedure")
                break
            elif not robot_values:
                pass
            elif robot_values == "DONE":
                self.uarm_read_pub.publish("DONE")
            else:
                msg = str(robot_values)
                msg = msg.translate(None, '[]')
                if msg[0] is False:
                    rospy.logerr("uArm read error")
                    rospy.is_shutdown("uArm read error")
                    break
                try:
                    self.uarm_read_pub.publish(msg)
                except Exception as e:
                    err_msg = "Failed to publish data: ", e.message
                    rospy.logerr(err_msg)
                    rospy.signal_shutdown("Error Shutdown Procedure")
                    break

        rospy.loginfo("uarm_read shutdown")

    def uarm_write_callback(self, data):
        if data.data == "CLEAR":
                self.iq.get_from__queue(all_msgs=True)
                self.rq.get_from_queue(all_msgs=True)

        elif data.data[0] == "!":
                msg = data.data[1:]
                self.iq.send_to_queue(msg, priority=1)
        else:
                self.iq.send_to_queue(data.data)

    def playback(self):
        if self.playback_data:
            self.loading = False
            self.iq.get_from_queue(all_msgs=True)  # flush queue
            try:
                joint_angles = self.ja
                self.iq.send_to_queue("JA"+str(joint_angles[0]) + "," +
                                      str(joint_angles[1]) + "," +
                                      str(joint_angles[2]) +
                                      str(joint_angles[3]))
            except Exception as e:
                err = str(e.message)
                rospy.logerr("joint angles not ready yet: " + err)
                rospy.signal_shutdown("joint angles not ready yet")

            self.iq.send_to_queue("ATT")

            self.iq.filter_queue("READ")

            for line in self.playback_data:
                if self.playback_active is False:
                    self.iq.get_from_queue(all_msgs=True)
                    break
                msg = str(line)

                self.iq.send_to_queue("READ")
                self.iq.send_to_queue(msg)

        self.iq.send_to_queue("BEEP")
        self.iq.send_to_queue("DET")
        self.playback_data = []

    def load_playback_data(self, data_line):
        self.loading = True
        rospy.loginfo("Loading playback data...")
        self.playback_data.append(data_line)

    @ros_try_catch
    def process_command(self, command):
        if command == "PUMP_ON":
            self.uarm.set_pump(1)

        if command == "PUMP_OFF":
            self.uarm.set_pump(0)

        if command == "ATT":
            self.uarm.set_servo_attach()

        if command == "DET":
            self.uarm.set_servo_detach()

        if command[0:3] == "POS":  # and self.playback_active:
            position = map(float, command[3:].split(','))
            self.uarm.set_position(position[0], position[1], position[2])

        if command[0:3] == "REL":
            adjust = float(command[3:])
            self.uarm.set_position(x=0, y=0, z=adjust, speed=0, relative=True)

        if command[0:2] == "JA":  # and self.playback_active:
            angle = map(float, command[2:].split(','))
            for i in range(0, len(angle)-1):
                self.uarm.set_servo_angle(i, angle[i])

        if command[0:4] == "LOAD":
            self.load_playback_data(command[4:])

        if command == "PLAY":
            if self.playback_data:
                self.playback_active = True
                self.playback_thread = threading.Thread(target=self.playback)
                self.playback_thread.daemon = True

                self.playback_thread.start()
        if command == "BEEP":
            self.uarm.set_buzzer(10000, 0.1)
            self.rq.send_to_queue("DONE", priority=1)
            self.playback_active = False

        if command == "STOP":
            self.playback_active = False
            self.iq.filter_queue(msg_filter="READ")
            time.sleep(0.5)
            self.iq.filter_queue(msg_filter="JA")
            self.playback_data = []
