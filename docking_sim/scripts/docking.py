#!/usr/bin/env python

from __future__ import division

import math
import time

import rospy
import tf
import actionlib
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Polygon, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty, EmptyResponse

import _footprint
from docking_sim.srv import Standby, StandbyResponse

IS_MOVING = False

def _start_enter():
    global IS_MOVING
    IS_MOVING = True
    _footprint.set_smaller(pub_local=PUB_FOOT_L, pub_global=PUB_FOOT_G)

def _done_enter(status, result):
    global IS_MOVING
    IS_MOVING = False
    _footprint.set_original(pub_local=PUB_FOOT_L, pub_global=PUB_FOOT_G)

def _start_standby():
    global IS_MOVING
    IS_MOVING = True

def _done_standby(status, result):
    global IS_MOVING
    IS_MOVING = False

def _feedback_leave(feedback):
    xn = float(feedback.base_position.pose.position.x)
    yn = float(feedback.base_position.pose.position.y)
    pass

def _pub_action_goal(ts, rz, start_cb=None, done_cb=None, feedback_cb=None):
    if start_cb:
        start_cb()
    q = tf.transformations.quaternion_from_euler(ai=0.0, aj=0.0, ak=rz)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = ts[0]
    goal.target_pose.pose.position.y = ts[1]
    goal.target_pose.pose.position.z = ts[2]
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]
    ACT_MOVEBASE.send_goal(goal=goal, done_cb=done_cb, feedback_cb=feedback_cb)

def _get_enter_rz(ts_c, ts_b):
    dx = ts_c[0] - ts_b[0]
    dy = ts_c[1] - ts_b[1]
    rz = math.atan2(dy, dx)
    return rz

def _get_standby_ts(ts_c, rs_c, location):
    x, y, _ = ts_c
    rz = rs_c[2]
    length = 1.0
    choice = {"0": (length, 0.0), "1": (0.0, -length), "2": (-length, 0.0), "3": (0.0, length)}
    dx, dy = choice[location]
    xn = x + dx*math.cos(rz) - dy*math.sin(rz)
    yn = y + dx*math.sin(rz) + dy*math.cos(rz)
    return (xn, yn, 0.0)

def _get_standby_rz(rs_c, location):
    choice = {"0": -math.pi, "1": math.pi/2.0, "2": 0.0, "3": -math.pi/2.0}
    rz = rs_c[2] + choice[location]
    return rz

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

def cb_standby(request):
    ts_c, rs_c = _get_tf(source="map", target="s_center_laser")
    ts = _get_standby_ts(ts_c=ts_c, rs_c=rs_c, location=request.location)
    rz = _get_standby_rz(rs_c=rs_c, location=request.location)
    _pub_action_goal(ts=ts, rz=rz, start_cb=_start_standby, done_cb=_done_standby)
    rate = rospy.Rate(hz=1)
    while IS_MOVING:
        rate.sleep()
    return StandbyResponse()

def cb_enter(request):
    ts_c, _ = _get_tf(source="map", target="s_center_laser")
    ts_b, _ = _get_tf(source="map", target="base_link")
    rz = _get_enter_rz(ts_c=ts_c, ts_b=ts_b)

    _pub_action_goal(ts=ts_c, rz=rz, start_cb=_start_enter, done_cb=_done_enter)

    rate = rospy.Rate(hz=1)
    while IS_MOVING:
        rate.sleep()

    return EmptyResponse()

def cb_leave(request):
    _, rs_c = _get_tf(source="map", target="s_center_laser")
    rz_p = _get_standby_rz(rs_c=rs_c, location=request.location)

    ts_b, _ = _get_tf(source="map", target="base_link")

    _pub_action_goal(ts=ts_b, rz=rz_p, start_cb=_start_standby, done_cb=_done_standby)

    rate = rospy.Rate(hz=1)
    while IS_MOVING:
        rate.sleep()

    _move_backward()

    return StandbyResponse()

def _move_backward():
    twist = Twist()
    twist.linear.x = -0.1
    PUB_CMDVEL.publish(twist)
    time.sleep(10)
    twist.linear.x = 0.0
    PUB_CMDVEL.publish(twist)

if __name__ == "__main__":

    rospy.init_node('docking', anonymous=False)

    # -- Get parameter
    ns_move_base = rospy.get_param(param_name="~ns_move_base", default="/move_base")

    # -- Get ActionClient, tf Listener
    ACT_MOVEBASE = actionlib.SimpleActionClient(ns=ns_move_base, ActionSpec=MoveBaseAction)
    TF_LISTENER = tf.TransformListener()

    # -- Node function
    rospy.Service(name="~standby", service_class=Standby, handler=cb_standby)
    rospy.Service(name="~enter", service_class=Empty, handler=cb_enter)
    rospy.Service(name="~leave", service_class=Standby, handler=cb_leave)

    PUB_FOOT_L = rospy.Publisher(name=ns_move_base+"/local_costmap/footprint", data_class=Polygon, queue_size=1)
    PUB_FOOT_G = rospy.Publisher(name=ns_move_base+"/global_costmap/footprint", data_class=Polygon, queue_size=1)
    PUB_CMDVEL = rospy.Publisher(name="/cmd_vel", data_class=Twist, queue_size=1)

    rospy.spin()
