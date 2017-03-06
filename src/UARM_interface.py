import time
import threading
import Queue
import os
import pyuarm
import rospy

from std_msgs.msg import String


def ros_try_catch(fn):
        def try_catch(*args, **kwargs):
            try:
                return fn(*args, **kwargs)
            except Exception as e:
                err_msg = "Error in function '" + fn.func_name + "': " + str(e.message)
                rospy.signal_shutdown(err_msg)
               # return False
        return try_catch

class UARM_interface():
    def __init__(self):
        self.uarm = None
        self.connected = False
        self.alive = True

        self.ros_rate = None
        self.ros_hz = 0

        self.settings = []

        self.uarm_read_thread = None
        self.uarm_read_queue = None
        self.read_queue_get_lock = threading.Lock()
        self.read_queue_put_lock = threading.Lock()
        self.uarm_read_pub = None

        self.uarm_interface_thread = None
        self.uarm_interface_queue = None
        self.interface_queue_get_lock = threading.Lock()
        self.interface_queue_put_lock = threading.Lock()

        self.playback_thread = None
        self.playback_active = False
        self.playback_data = []
        self.playback_rate = 1.0
        self.loading = False

        self.ja = []

    def get_setting(self, name, string_val=False):
        for line in self.settings:
            if line[0] == name:
                if string_val:
                    return line[1]
                else:
                    return float(line[1])

    def connect(self):
        try:
            self.uarm = pyuarm.get_uarm()
            self.connected = True
            rospy.loginfo("Connected")
        except Exception as e:
            self.connected = False
            err =  str(e.message)
            rospy.logerr("Connection error: " + err)
            rospy.signal_shutdown("Connection error")

    def shutdown(self):
        rospy.loginfo("Shutdown function call")
        rospy.signal_shutdown("Shutdown function call")
        start_shutdown = time.time()
        while self.uarm_interface_thread.is_alive() or self.uarm_read_thread.is_alive():
            #print "Shutdown interface queue"
            self.send_to_interface_queue("SHUTDOWN", priority=True)
            #print "Shutdown read queue"
            self.send_to_read_queue("SHUTDOWN", priority=True)
            #print "Thread status: ", self.uarm_interface_thread.is_alive(), self.uarm_read_thread.is_alive()

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

        self.ros_hz = self.get_setting("ros_rate")
        self.uarm_read_pub = rospy.Publisher('uarm_read', String, queue_size=10)
        rospy.Subscriber("uarm_write", String, self.uarm_write_callback)
        rospy.init_node('uarm_node', anonymous=True)
        self.ros_rate = rospy.Rate(self.ros_hz)

        self.uarm_read_queue = Queue.Queue()
        self.uarm_interface_queue = Queue.Queue()

        rospy.loginfo("Done loading parameters.")

    def start_threads(self):
        rospy.loginfo("Starting threads")
        if self.connected is True:
            self.uarm_read_thread = threading.Thread(target=self.uarm_read)
            self.uarm_interface_thread = threading.Thread(target=self.uarm_interface)

            self.uarm_read_thread.daemon = True
            self.uarm_read_thread.start()

            self.uarm_interface_thread.daemon = True
            self.uarm_interface_thread.start()

            self.request_detach()

        else:
            rospy.logerr("Startup error")
            rospy.signal_shutdown("Startup error")

    def uarm_interface(self):
        rospy.loginfo("uarm_interface running")
        while True and (rospy.is_shutdown() is False):
            request = self.get_from_interface_queue()
            if request == "READ":
                curr_vals = self.read_position()
                self.send_to_read_queue(curr_vals)
                try:
                    self.ja = curr_vals[1]
                except Exception as e:
                    err = str(e.message)
                    rospy.logerr("uArm not ready! " + err)
                    rospy.signal_shutdown("Error reading from uArm")

                if self.playback_active is False and self.loading is False:
                    self.send_to_interface_queue("READ")

            elif request == "SHUTDOWN":
                break
            else:
                self.process_command(request)
                if self.playback_active is False and self.loading is False:
                    self.send_to_interface_queue("READ")

        rospy.loginfo("uarm_interface shutdown")
        self.shutdown()

    @ros_try_catch
    def request_detach(self):
        rospy.loginfo("uArm detach")
        self.send_to_interface_queue("DET")

    @ros_try_catch
    def request_attach(self):
        rospy.loginfo("uArm attach")
        self.send_to_interface_queue("ATT")

    @ros_try_catch
    def get_analog(self, pin_num):
        return round(self.uarm.get_analog(pin_num), 3)

    @ros_try_catch
    def get_joint_angles(self):
        return self.uarm.get_servo_angle()

    @ros_try_catch
    def read_position(self):
        return self.uarm.get_position()

    @ros_try_catch
    def read_joint_angles(self):
        #return [self.get_joint_angles(), time.time()]
        return self.get_joint_angles()

    def uarm_read(self):
        rospy.loginfo("uarm_read running")
        while True and (rospy.is_shutdown() is False):
            robot_values = self.get_from_read_queue(blocking=False)
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
                self.uarm_read_pub.publish(msg)
                self.ros_rate.sleep()

        rospy.loginfo("uarm_read shutdown")

    def send_to_interface_queue(self, msg, msg_list=False, priority=False):
        with self.interface_queue_put_lock:
            if msg_list:
                if priority:
                    msg_list = msg + self.get_from_interface_queue(all_msgs=True)
                for m in msg_list:
                    self.uarm_interface_queue.put(m)
            else:
                if priority:
                    msg_list = [msg] + self.get_from_interface_queue(all_msgs=True)
                    for m in msg_list:
                        self.uarm_interface_queue.put(m)
                else:
                    self.uarm_interface_queue.put(msg)

    def filter_interface_queue(self, msg_filter, invert=False):
        msgs = self.get_from_interface_queue(all_msgs=True, blocking=True)
        if msg_filter is not "":
            #print "FILTER: ", filter
            n = 0
            for i, msg in enumerate(msgs):
                if filter in msg and not invert:
                    msgs.pop(i)
                    n += 1
                elif filter not in msg and invert:
                    msgs.pop(i)
                    n += 1
            if msgs is not []:
                #print n, " messages filtered matching search terms", filter, invert
                #print msgs
                self.send_to_interface_queue(msgs, msg_list=True, priority=True)

    def get_from_interface_queue(self, all_msgs=False, blocking=False):
        with self.interface_queue_get_lock:
            if all_msgs:
                msgs = []
                while self.uarm_interface_queue.qsize() > 0:
                    try:
                        msgs.append(self.uarm_interface_queue.get(blocking))

                    except Queue.Empty:
                        msgs = []

                return msgs
            else:
                try:
                    msg = self.uarm_interface_queue.get(blocking)
                except Queue.Empty:
                    msg = ""
                return msg

    def send_to_read_queue(self, msg, msg_list=False, priority=False):
        with self.read_queue_put_lock:
            if msg_list:
                if priority:
                    msg_list = msg + self.get_from_read_queue(all_msgs=True)
                for m in msg_list:
                    self.uarm_read_queue.put(m)
            else:
                if priority:
                    #print "Sending priority message, read_queue:", msg
                    msg_list = [msg] + self.get_from_read_queue(all_msgs=True)
                    for m in msg_list:
                        self.uarm_read_queue.put(m)
                else:
                    self.uarm_read_queue.put(msg)

    def get_from_read_queue(self, all_msgs=False, blocking=False):
        with self.read_queue_get_lock:
            if all_msgs:
                msgs = []
                while self.uarm_read_queue.qsize() > 0:
                    try:
                        msgs.append(self.uarm_read_queue.get(blocking))
                    except Queue.Empty:
                        msgs = []
                return msgs
            else:
                try:
                    msg = self.uarm_read_queue.get(blocking)
                except Queue.Empty:
                    msg = ""
                return msg

    def uarm_write_callback(self, data):
        #print "DATA: ", data.data
        if data.data == "CLEAR":
                self.get_from_interface_queue(all_msgs=True)
                self.get_from_read_queue(all_msgs=True)

        elif data.data[0] == "!":
                msg = data.data[1:]
                #print msg
                self.send_to_interface_queue(msg, priority=True)
        else:
                self.send_to_interface_queue(data.data, priority=False)

    def playback(self):
        if self.playback_data:
            self.loading = False
            self.get_from_interface_queue(all_msgs=True)  # flush queue
            try:
                joint_angles = self.ja
                self.send_to_interface_queue("JA"+str(joint_angles[0]) + "," +
                                             str(joint_angles[1]) + "," +
                                             str(joint_angles[2]) +
                                             str(joint_angles[3]))
            except Exception as e:
                #print "joint angles not ready yet: ", e
                err = str(e.message)
                rospy.logerr("joint angles not ready yet: " + err)
                rospy.signal_shutdown("joint angles not ready yet")


            self.send_to_interface_queue("ATT")

            self.filter_interface_queue("READ")

            for line in self.playback_data:
                #print self.playback_active, line
                if self.playback_active is False:
                    self.get_from_interface_queue(all_msgs=True)
                    break
                msg = str(line)

                self.send_to_interface_queue("READ")
                self.send_to_interface_queue(msg)

        self.send_to_interface_queue("BEEP")
        self.send_to_interface_queue("DET")
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

        if command[0:3] == "POS" and self.playback_active:
            position = map(float, command[3:].split(','))
            self.uarm.set_position(position[0], position[1], position[2])

        if command[0:3] == "REL":
            adjust = float(command[3:])
            #print adjust
            self.uarm.set_position(x=0, y=0, z=adjust, speed=0, relative=True)

        if command[0:2] == "JA" and self.playback_active:
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
            self.send_to_read_queue("DONE", priority=True)
            self.playback_active = False

        if command == "STOP":
            self.playback_active = False
            self.filter_interface_queue(msg_filter="READ")
            time.sleep(0.5)
            self.filter_interface_queue(msg_filter="JA")
            self.playback_data = []
