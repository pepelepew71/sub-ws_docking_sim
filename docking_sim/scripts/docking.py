#!/usr/bin/env python

from __future__ import print_function
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
from docking_sim.srv import Prepare, PrepareResponse

IS_MOVING = False

def _start_enter():
    global IS_MOVING
    IS_MOVING = True
    _footprint.set_smaller(pub_local=PUB_FOOT_L, pub_global=PUB_FOOT_G)

def _done_enter(status, result):
    global IS_MOVING
    IS_MOVING = False
    _footprint.set_original(pub_local=PUB_FOOT_L, pub_global=PUB_FOOT_G)

def _start_prepare():
    global IS_MOVING
    IS_MOVING = True

def _done_prepare(status, result):
    global IS_MOVING
    IS_MOVING = False

def _feedback_leave(feedback):
    xn = float(feedback.base_position.pose.position.x)
    yn = float(feedback.base_position.pose.position.y)
    pass

def _pub_action_goal(ts, q, start_cb=None, done_cb=None, feedback_cb=None):
    if start_cb:
        start_cb()
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

def _get_enter_q(ts_c, ts_b):
    dx = ts_c[0] - ts_b[0]
    dy = ts_c[1] - ts_b[1]
    rz = math.atan2(dy, dx)
    q = tf.transformations.quaternion_from_euler(ai=0.0, aj=0.0, ak=rz)
    return q

def _get_prepare_ts(ts, rs, location):
    _, _, rz = tf.transformations.euler_from_quaternion((rs[0], rs[1], rs[2], rs[3]))
    x, y, _ = ts
    choice = {"0": (1.0, 0.0), "1": (0.0, -1.0), "2": (-1.0, 0.0), "3": (0.0, 1.0)}
    dx, dy = choice[location]
    xn = x + dx*math.cos(rz) - dy*math.sin(rz)
    yn = y + dx*math.sin(rz) + dy*math.cos(rz)
    return (xn, yn, 0.0)

def _get_prepare_q(rs, location):
    choice = {"0": -math.pi, "1": math.pi/2.0, "2": 0.0, "3": -math.pi/2.0}
    ak = choice[location]
    p = tf.transformations.quaternion_from_euler(ai=0.0, aj=0.0, ak=ak)
    q = tf.transformations.quaternion_multiply(rs, (p[0], p[1], p[2], p[3]))
    return q

def _get_leave_q(q):
    p = tf.transformations.quaternion_from_euler(ai=0.0, aj=0.0, ak=math.pi)
    q = tf.transformations.quaternion_multiply((q[0], q[1], q[2], q[3]), (p[0], p[1], p[2], p[3]))
    return q

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

def cb_prepare(request):
    ts_c, rs_c = _get_tf(source="map", target="s_center_laser")
    ts = _get_prepare_ts(ts=ts_c, rs=rs_c, location=request.location)
    q = _get_prepare_q(rs=rs_c, location=request.location)
    _pub_action_goal(ts=ts, q=q, start_cb=_start_prepare, done_cb=_done_prepare)
    rate = rospy.Rate(hz=1)
    while IS_MOVING:
        rate.sleep()
    return PrepareResponse()

def cb_enter(request):
    ts_c, _ = _get_tf(source="map", target="s_center_laser")
    ts_b, _ = _get_tf(source="map", target="base_link")
    q = _get_enter_q(ts_c=ts_c, ts_b=ts_b)

    _pub_action_goal(ts=ts_c, q=q, start_cb=_start_enter, done_cb=_done_enter)

    rate = rospy.Rate(hz=1)
    while IS_MOVING:
        rate.sleep()

    return EmptyResponse()

def cb_leave(request):
    ts_c, rs_c = _get_tf(source="map", target="s_center_laser")
    ts_b, _ = _get_tf(source="map", target="base_link")
    ts_p = _get_prepare_ts(ts=ts_c, rs=rs_c, location=request.location)
    q_p = _get_prepare_q(rs=rs_c, location=request.location)
    q_l = _get_leave_q(q=q_p)

    _pub_action_goal(ts=ts_b, q=q_p, start_cb=_start_prepare, done_cb=_done_prepare)

    rate = rospy.Rate(hz=1)
    while IS_MOVING:
        rate.sleep()

    # _pub_action_goal(ts=ts_p, q=q_l, start_cb=_start_prepare, done_cb=_done_prepare)
    _move_backward()
    while IS_MOVING:
        rate.sleep()

    return PrepareResponse()

def _move_backward():
    global IS_MOVING
    IS_MOVING = True
    twist = Twist()
    twist.linear.x = -0.1
    PUB_CMDVEL.publish(twist)
    time.sleep(10)
    twist.linear.x = 0.0
    PUB_CMDVEL.publish(twist)
    IS_MOVING = False

if __name__ == "__main__":

    rospy.init_node('docking', anonymous=False)

    # -- Get parameter
    ns_move_base = rospy.get_param(param_name="~ns_move_base", default="/move_base")

    # -- Get ActionClient, tf Listener
    ACT_MOVEBASE = actionlib.SimpleActionClient(ns=ns_move_base, ActionSpec=MoveBaseAction)
    TF_LISTENER = tf.TransformListener()

    # -- Node function
    rospy.Service(name="~prepare", service_class=Prepare, handler=cb_prepare)
    rospy.Service(name="~enter", service_class=Empty, handler=cb_enter)
    rospy.Service(name="~leave", service_class=Prepare, handler=cb_leave)

    PUB_FOOT_L = rospy.Publisher(name=ns_move_base+"/local_costmap/footprint", data_class=Polygon, queue_size=1)
    PUB_FOOT_G = rospy.Publisher(name=ns_move_base+"/global_costmap/footprint", data_class=Polygon, queue_size=1)
    PUB_CMDVEL = rospy.Publisher(name="/cmd_vel", data_class=Twist, queue_size=1)

    rospy.spin()
