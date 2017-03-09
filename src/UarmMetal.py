#!/usr/bin/env python
import time
import threading
import os
import pyuarm
import rospy
import ThreadSafePriorityQueue as tspq
from uarm_decorators import *

from std_msgs.msg import String, Bool
from uarm_metal.msg import Position, JointAngles, Beep


class UarmMetal():

    def __init__(self):
        self.uarm = None
        self.connected = False
        self.ready = False
        self.alive = True
        self.moving = False

        self.ros_rate = None
        self.ros_hz = 0
        self.read_pos = None
        self.read_ja = None
        self.read_AI = None
        self.read_DI = None

        self.settings = []

        self.parameter_monitor_thread = None

        self.uarm_read_thread = None
        self.string_read_pub = None
        self.rq = tspq.ThreadSafePriorityQueue("read_queue")

        self.uarm_interface_thread = None
        self.iq = tspq.ThreadSafePriorityQueue("interface_queue")

        self.move_monitor_thread = None
        self.mq = tspq.ThreadSafePriorityQueue("move_queue")

        self.pos_pub = None
        self.ja_pub = None
        self.ai_pub = None
        self.di_pub = None

        self.playback_thread = None
        self.playback_active = False
        self.playback_data = []
        self.loading = False

        self.ja = []

#INIT Initialisation
# Initialisation
#endINIT

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
        except Exception as e:
            print "Connecting to uArm error, check connection: ", e
            return False
        try:
            self.connected = True
            self.load_parameters()
            self.connect_to_ROS()
            self.start_threads()
            rospy.loginfo("Connected")
            self.ready = True
            return True
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
        self.read_AI = self.get_setting("read_analog_inputs", list_type=True)
        self.read_DI = self.get_setting("read_digital_inputs", list_type=True)

        rospy.set_param('uarm_metal/read_position', self.read_pos)
        rospy.set_param('uarm_metal/read_joint_angles', self.read_ja)
        rospy.set_param('uarm_metal/read_analog_inputs', self.read_AI)
        rospy.set_param('uarm_metal/read_digital_inputs', self.read_DI)

    def connect_to_ROS(self):
        ns = 'uarm_metal'
        try:
            print "NAMESP: ", rospy.get_namespace()
        except Exception as e:
            print "Couldn't get ns: ", e

        rospy.init_node(ns, anonymous=False)

        ns += "/"
        self.string_read_pub = rospy.Publisher(ns + 'string_read', String, queue_size=10)
        self.pos_pub = rospy.Publisher(ns + 'position_read', Position, queue_size=10)
        self.ja_pub = rospy.Publisher(ns + 'joint_angles_read', JointAngles, queue_size=10)
        self.ai_pub = rospy.Publisher(ns + 'analog_inputs_read', String, queue_size=10)
        self.di_pub = rospy.Publisher(ns + 'digital_inputs_read', String, queue_size=10)
        #  uarm_metal

        rospy.Subscriber(ns+"string_write", String, self.string_write_callback, queue_size= 1000)
        rospy.Subscriber(ns+"position_write", Position, self.position_write_callback, queue_size=1000)
        rospy.Subscriber(ns+"joint_angles_write", JointAngles, self.ja_write_callback, queue_size=1000)
        rospy.Subscriber(ns+"pump", Bool, self.pump_write_callback, queue_size=1000)
        rospy.Subscriber(ns+"attach", Bool, self.attach_write_callback, queue_size=1000)
        rospy.Subscriber(ns+"beep", Beep, self.beep_write_callback, queue_size=1000)

        self.ros_rate = rospy.Rate(self.ros_hz)

        rospy.loginfo("Done loading parameters.")

    def parameter_monitor(self):
        rospy.loginfo("Parameter monitor running.")
        while True and (rospy.is_shutdown() is False):
            self.read_pos = int(rospy.get_param('uarm_metal/read_position'))
            self.read_ja = int(rospy.get_param('uarm_metal/read_joint_angles'))
            self.read_AI = map(int, rospy.get_param('uarm_metal/read_analog_inputs'))
            self.read_DI = map(int, rospy.get_param('uarm_metal/read_digital_inputs'))
            time.sleep(0.5)

        rospy.loginfo("Parameter monitor shutdown")

    def start_threads(self):
        rospy.loginfo("Starting threads")
        if self.connected is True:




            self.uarm_read_thread = threading.Thread(target=self.uarm_read)
            self.uarm_read_thread.daemon = True
            self.uarm_read_thread.start()

            self.uarm_interface_thread = threading.Thread(target=self.uarm_interface)
            self.uarm_interface_thread.daemon = True
            self.uarm_interface_thread.start()

            self.parameter_monitor_thread = threading.Thread(target=self.parameter_monitor)
            self.parameter_monitor_thread.daemon = True
            self.parameter_monitor_thread.start()

            self.move_monitor_thread = threading.Thread(target=self.monitor_ja_write)
            self.move_monitor_thread.daemon = True
            self.move_monitor_thread.start()

            #self.request_detach()

        else:
            rospy.logerr("Startup error")
            rospy.signal_shutdown("Startup error")

#MAIN Main Threads
# Main threads/functions
#endMAIN

    def get_read_data(self):
        msg = []
        if self.read_pos > 0:
            msg.append(self.read_position())
        if self.read_ja > 0:
            msg.append(self.read_joint_angles())
        if self.read_AI[0] > 0:
            analog_vals = []
            for i in self.read_AI[1:]:
                analog_vals.append(self.read_analog(i))
            msg.append(analog_vals[:])
        if self.read_DI[0] > 0:
            digital_vals = []
            for i in self.read_DI[1:]:
                digital_vals.append(self.read_digital(i))
            msg.append(digital_vals[:])
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

                if self.playback_active is False and self.loading is False:
                    self.iq.send_to_queue("READ")

            elif request == "SHUTDOWN":
                break
            else:
                self.process_command(request)
                #if self.playback_active is False and self.loading is False:
                self.iq.send_to_queue("READ",priority=1)

        rospy.loginfo("uarm_interface shutdown")
        self.shutdown()

    def uarm_read(self):
        rospy.loginfo("uarm_read running")
        while True and (rospy.is_shutdown() is False):
            robot_values = self.rq.get_from_queue(blocking=False)
            if robot_values:
                if robot_values == "SHUTDOWN":
                    rospy.logwarn("sending shutdown signal")
                    rospy.signal_shutdown("Normal Shutdown Procedure")
                    break
                else:
                    msg = str(robot_values)
                    msg = msg.translate(None, '[]')

                    if str(msg)[0:5] == "False":
                        rospy.logerr("uArm read error")
                        rospy.signal_shutdown("uArm read error")
                        self.uarm.disconnect()
                        break
                    try:
                        self.string_read_pub.publish(msg)
                        if self.read_pos > 0:
                            pos_msg = Position()
                            try:
                                pos_data = robot_values[0]
                                pos_msg.x = pos_data[0]
                                pos_msg.y = pos_data[1]
                                pos_msg.z = pos_data[2]
                                self.pos_pub.publish(pos_msg)
                            except Exception as e:
                                #print e
                                pass

                        if self.read_ja > 0:
                            ja_msg = JointAngles()
                            try:
                                ja_data = robot_values[self.read_pos]
                                ja_msg.j0 = ja_data[0]
                                ja_msg.j1 = ja_data[1]
                                ja_msg.j2 = ja_data[2]
                                ja_msg.j3 = ja_data[3]

                                self.ja_pub.publish(ja_msg)
                            except Exception as e:
                                #print e
                                pass

                        if self.read_AI[0] > 0:
                            try:
                                ai_data = robot_values[self.read_pos + self.read_ja]
                                ai_msg = str(ai_data).translate(None,'[]')
                                self.ai_pub.publish(ai_msg)
                            except Exception as e:
                                #print e
                                pass

                        if self.read_DI[0] > 0:
                            try:
                                di_data = robot_values[self.read_pos + self.read_ja + self.read_AI[0]]
                                di_msg = str(di_data).translate(None,'[]')
                                self.di_pub.publish(di_msg)
                            except Exception as e:
                                #print e
                                pass

                    except Exception as e:
                        err_msg = "Failed to publish data: ", e.message
                        rospy.logerr(err_msg)
                        rospy.signal_shutdown("Error Shutdown Procedure")
                        break
        time.sleep(1.0/self.ros_hz)
        rospy.loginfo("uarm_read shutdown")

    def string_write_callback(self, data):
        if data.data == "CLEAR":
                self.iq.get_from_queue(all_msgs=True)
                self.rq.get_from_queue(all_msgs=True)

        elif data.data[0] == "!":
                msg = data.data[1:]
                self.iq.send_to_queue(msg, priority=1)
        else:
                self.iq.send_to_queue(data.data)

    def position_write_callback(self, data):
        self.request_position(data)
        # if data.data[0]:
        #     msg = data.data[1:]
        #     self.iq.send_to_queue(msg)

    def ja_write_callback(self, data):
        #self.request_ja(data)
        self.mq.send_to_queue(data)
       # time.sleep(0.05)

    def pump_write_callback(self, data):
        if data.data:
            self.pump_on()
        else:
            self.pump_off()

    def attach_write_callback(self, data):
        rospy.loginfo("Attach")
        if data.data:
            self.request_attach()
        else:
            self.request_detach()

    def beep_write_callback(self, data):
        print data
        self.request_beep(data)
        # if data.data[0]:
        #     msg = data.data[1:]
        #     self.iq.send_to_queue(msg)

#ACTIONS Action request for uArm
# Actions
#endACTIONS
    def request_move_check(self):
        self.iq.send_to_queue("CHK", priority=1)

    def request_position(self, data):
        msg = "POS" + str(data.x) + "," + str(data.y) + "," + str(data.z)
        self.iq.send_to_queue(msg)

    def monitor_ja_write(self):
        while True and (rospy.is_shutdown() is False):
            print self.mq.queue.qsize(), self.moving
            if self.moving == False and self.mq.queue.qsize()>0:
                # try:
                data = self.mq.get_from_queue()
                msg = "JA" + str(round(data.j0,0)) + "," + str(round(data.j1,0)) \
                      + "," + str(round(data.j2,0)) + "," + str(round(data.j3,0))
                print msg, map(float, msg[2:].split(','))
                self.iq.send_to_queue(msg)
                time.sleep(0.05)
                if self.mq.queue.qsize() > 0:
                    print "CHK"
                    self.request_move_check()

                # except Exception as e:
                #     time.sleep(0.5)
                #     pass
            else:
                #print self.moving
                time.sleep(0.01)

    def request_ja(self, data):
        msg = "JA" + str(round(data.j0,2)) + "," + str(round(data.j1,2)) \
              + "," + str(round(data.j2,2)) + "," + str(round(data.j3,2))
        print msg, map(float, msg[2:].split(','))
        self.iq.send_to_queue(msg)

    def request_beep(self, data):
        rospy.loginfo("Beep")
        msg = "BEEP" + str(data.frequency) + "," + str(data.duration)
        self.iq.send_to_queue(msg)

    def request_detach(self):
        rospy.loginfo("Detach motors")
        self.iq.send_to_queue("DET")

    def request_attach(self):
        rospy.loginfo("Attach motors")
        self.iq.send_to_queue("ATT")

    def pump_on(self):
        rospy.loginfo("Pump On")
        self.iq.send_to_queue("PUMP_ON")

    def pump_off(self):
        rospy.loginfo("Pump Off")
        self.iq.send_to_queue("PUMP_OFF")

# GETS
    @ros_try_catch
    def read_analog(self, pin_num):
        return round(self.uarm.get_analog(pin_num), 3)

    @ros_try_catch
    def read_digital(self, pin_num):
        return self.uarm.get_digital(pin_num)

    @ros_try_catch
    def get_joint_angles(self):
        return self.uarm.get_servo_angle()

    @ros_try_catch
    def read_position(self):
        position = []
        position = self.uarm.get_position()
        return position

    @ros_try_catch
    def read_joint_angles(self):
        angles = []
        angles = self.get_joint_angles()
        return angles

# SETS
    @ros_try_catch
    def process_command(self, command):
        if command == "CHK":
            self.moving = self.uarm.is_moving()
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

        if command[0:2] == "WR":
            angle = float(command[2:])
            self.uarm.set_servo_angle(3,angle)

        if command[0:3] == "REL":
            adjust = float(command[3:])
            self.uarm.set_position(x=0, y=0, z=adjust, speed=0, relative=True)


        if command[0:2] == "JA":  # and self.playback_active:
            angle = map(float, command[2:].split(','))
            for i in range(0, len(angle)):
                self.uarm.set_servo_angle(i, angle[i])

            #time.sleep(0.2)

        if command[0:4] == "BEEP":
            try:
                info = map(float,command[4:].split(','))
                freq = info[0]
                dur = info[1]
            except Exception as e:
                freq = 10000
                dur = 0.1
            self.uarm.set_buzzer(freq, dur)
            self.playback_active = False

        if command == "STOP":
            self.iq.filter_queue(msg_filter="READ")
            time.sleep(0.5)
            self.iq.filter_queue(msg_filter="JA")


            # if command[0:4] == "LOAD":
            #     self.load_playback_data(command[4:])

            # if command == "PLAY":
            #     if self.playback_data:
            #         self.playback_active = True
            #         self.playback_thread = threading.Thread(target=self.playback)
            #         self.playback_thread.daemon = True

            # self.playback_thread.start()
