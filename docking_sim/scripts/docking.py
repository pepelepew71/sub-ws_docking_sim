#!/usr/bin/env python

from __future__ import division

import math
import time

import numpy as np

import rospy
import tf
import actionlib
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Polygon, PolygonStamped, Twist
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

def _pub_action_goal(ts, q, start_cb=None, done_cb=None, feedback_cb=None):
    """
    Args:
        ts (list): transform (map to goal)
        q (tf.transformations.Quaterion): quaternion (map to goal)
    """
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
    """
    Args:
        ts_c (list): transform (map to s_center_laser)
        ts_b (list): transform (map to base_link)
    Return:
        (tf.transformations.Quaterion): quaternion
    """
    dx = ts_c[0] - ts_b[0]
    dy = ts_c[1] - ts_b[1]
    ak = math.atan2(dy, dx)
    q = tf.transformations.quaternion_from_euler(ai=0.0, aj=0.0, ak=ak)
    return q

def _get_standby_ts(ts_c, q_c, location):
    """
    Args:
        ts_c: transform (map to s_center_laser)
        q_c (tf.transformations.Quaterion): quaternion (map to s_center_laser)
        location (str): standby location (0, 1, 2, 3)
    Return:
        (list): transform
    """
    x, y, _ = ts_c
    _, _, rz = tf.transformations.euler_from_quaternion((q_c[0], q_c[1], q_c[2], q_c[3]))
    length = 1.0
    choice = {"0": (length, 0.0), "1": (0.0, -length), "2": (-length, 0.0), "3": (0.0, length)}
    dx, dy = choice[location]
    xn = x + dx*math.cos(rz) - dy*math.sin(rz)
    yn = y + dx*math.sin(rz) + dy*math.cos(rz)
    return (xn, yn, 0.0)

def _get_standby_q(q_c, location):
    """
    Args:
        q_c (tf.transformations.Quaterion): quaternion (map to s_center_laser)
        location (str): standby location (0, 1, 2, 3)
    Return:
        (tf.transformations.Quaterion): quaterion
    """
    choice = {"0": -math.pi, "1": math.pi/2.0, "2": 0.0, "3": -math.pi/2.0}
    ak = choice[location]
    p = tf.transformations.quaternion_from_euler(ai=0.0, aj=0.0, ak=ak)
    q = tf.transformations.quaternion_multiply(q_c, (p[0], p[1], p[2], p[3]))
    return q

def _get_tf(source, target):
    """
    Args:
        source (str): tf source
        target (str): tf target
    Return:
        (tuple): transform, quaterion
    """
    rate = rospy.Rate(hz=10)
    while True:
        try:
            ts, q = TF_LISTENER.lookupTransform(source, target, rospy.Time(secs=0))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("docking._get_tf error")
        rate.sleep()
    return (ts, q)

def cb_standby(request):
    """
    Args:
        request (docking_sim.srv.Standby)
    Return:
        (docking_sim.srv.StandbyResponse)
    """
    ts_c, q_c = _get_tf(source="map", target="s_center_laser")
    ts = _get_standby_ts(ts_c=ts_c, q_c=q_c, location=request.location)
    q = _get_standby_q(q_c=q_c, location=request.location)
    _pub_action_goal(ts=ts, q=q, start_cb=_start_standby, done_cb=_done_standby)
    rate = rospy.Rate(hz=1)
    while IS_MOVING:
        rate.sleep()
    return StandbyResponse()

def cb_enter(request):
    """
    Args:
        request (std_srvs.srv.Empty)
    Return:
        (std_srvs.srv.EmptyResponse)
    """
    ts_c, _ = _get_tf(source="map", target="s_center_laser")
    ts_b, _ = _get_tf(source="map", target="base_link")
    q = _get_enter_q(ts_c=ts_c, ts_b=ts_b)

    _pub_action_goal(ts=ts_c, q=q, start_cb=_start_enter, done_cb=_done_enter)

    rate = rospy.Rate(hz=1)
    while IS_MOVING:
        rate.sleep()

    return EmptyResponse()

def cb_leave(request):
    """
    Args:
        request (docking_sim.srv.Standby)
    Return:
        (docking_sim.srv.StandbyResponse)
    """
    _, q_c = _get_tf(source="map", target="s_center_laser")
    q_l = _get_standby_q(q_c=q_c, location=request.location)
    ts_b, _ = _get_tf(source="map", target="base_link")

    # -- step 1: rotate
    _pub_action_goal(ts=ts_b, q=q_l, start_cb=_start_standby, done_cb=_done_standby)

    rate = rospy.Rate(hz=1)
    while IS_MOVING:
        rate.sleep()

    # -- step 2: leave
    _move_backward_pid(location=request.location)

    return StandbyResponse()

def _move_backward_pid(location):
    """
    Args:
        location (str): standby location (0, 1, 2, 3)
    """
    wz_last = 0.0
    p = 5.0
    i = 0.0
    t_start = rospy.get_rostime().to_sec()
    rate = rospy.Rate(hz=5)
    while True:
        twist = Twist()
        t_now = rospy.get_rostime().to_sec()
        if t_now - t_start < 10.0:
            twist.linear.x = -0.1
            error = _get_yaw_error(location=location)
            if abs(error) < 0.025:  # 1.432 degree
                twist.angular.z = 0.0
            else:
                wz_now = p*error + i*wz_last
                twist.angular.z = wz_now
            wz_last = twist.angular.z
            PUB_CMDVEL.publish(twist)
        else:
            PUB_CMDVEL.publish(twist)
            break
        rate.sleep()

def _get_yaw_error(location):
    """
    Args:
        location (str): standby location (0, 1, 2, 3)
    Return:
        (float): yaw error between standby location and base_link
    """
    _, q_c = _get_tf(source="map", target="s_center_laser")
    q_l = _get_standby_q(q_c=q_c, location=location)
    _, q_b = _get_tf(source="map", target="base_link")
    r_b = tf.transformations.euler_from_quaternion(quaternion=q_b)
    r_l = tf.transformations.euler_from_quaternion(quaternion=q_l)
    return r_l[2] - r_b[2]

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

    PUB_FOOT_L = rospy.Publisher(name=ns_move_base+"/local_costmap/footprint", data_class=PolygonStamped, queue_size=1)
    PUB_FOOT_G = rospy.Publisher(name=ns_move_base+"/global_costmap/footprint", data_class=PolygonStamped, queue_size=1)
    PUB_CMDVEL = rospy.Publisher(name="/cmd_vel", data_class=Twist, queue_size=1)

    rospy.spin()
