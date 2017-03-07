#!/usr/bin/env python
import rospy


def ros_try_catch(fn):
    def try_catch(*args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except Exception as e:
            err_msg = "Error in function '" + fn.func_name + "': " + str(e.message)
            rospy.logerr(err_msg)
            rospy.signal_shutdown(err_msg)

    return try_catch