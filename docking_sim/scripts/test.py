#!/usr/bin/env python

from __future__ import division

import rospy
import tf

def _get_tf(source, target):
    rate = rospy.Rate(hz=10)
    while True:
        try:
            ts, rs = TF_LISTENER.lookupTransform(source, target, rospy.Time(secs=0))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("docking._get_tf error")
        rate.sleep()
    return (ts, rs)

def _get_prepare_rz(rs, location):
    choice = {"0": -math.pi, "1": math.pi/2.0, "2": 0.0, "3": -math.pi/2.0}
    rz = rs[2] + choice[location]
    return rz

def get_delta_rz():
    _, rs_c = _get_tf(source="map", target="s_center_laser")
    _, rs_b = _get_tf(source="map", target="base_link")

if __name__ == "__main__":

    rospy.init_node('test', anonymous=False)

    TF_LISTENER = tf.TransformListener()

    rate = rospy.Rate(hz=1)
    t_start = rospy.get_rostime()

    while True:
        t_now = rospy.get_rostime()
        d_sec = t_now.secs - t_start.secs
        if d_sec > 5:
            break
        get_delta_rz()

        rate.sleep()
