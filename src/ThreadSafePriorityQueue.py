#!/usr/bin/env python
import Queue
import threading

class ThreadSafePriorityQueue():

    def __init__(self):
        self.queue = None
        self.get_lock = None
        self.put_lock = None

        self.queue = Queue.PriorityQueue()
        self.get_lock = threading.Lock()
        self.put_lock = threading.Lock()

    def send_to_queue(self, msg, msg_list=False, priority=10):
        with self.put_lock:
            if msg_list:
                for m in msg_list:
                    self.queue.put((priority, m))
            else:
                self.queue.put((priority, msg))

    def get_from_queue(self, all_msgs=False, blocking=False):
        with self.get_lock:
            if all_msgs:
                msgs = []
                while self.queue.qsize() > 0:
                    try:
                        msg = self.queue.get(blocking)
                        try:
                            msg = msg[1]
                        except:
                            pass
                        print msg
                        msgs.append(msg)

                    except Queue.Empty:
                        msgs = []

                return msgs
            else:
                try:
                    msg = self.queue.get(blocking)
                except Queue.Empty:
                    msg = ""
                try:
                    msg = msg[1]
                except:
                    pass
                return msg

    def filter_queue(self, msg_filter, invert=False):
        msgs = self.get_from_queue(all_msgs=True, blocking=True)
        if msg_filter is not "":
            n = 0
            for i, msg in enumerate(msgs):
                if filter in msg and not invert:
                    msgs.pop(i)
                    n += 1
                elif filter not in msg and invert:
                    msgs.pop(i)
                    n += 1
            if msgs is not []:
                self.send_to_queue(msgs, msg_list=True, priority=1)

