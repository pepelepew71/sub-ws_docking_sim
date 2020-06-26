#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import math

import rospy
import tf
import actionlib
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Polygon, Point32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty, EmptyResponse

from docking_sim.srv import Goto, GotoResponse

I_ENTER = 0
IS_RUNNING = False

def _done_cb(status, result):
    global I_ENTER, IS_RUNNING
    I_ENTER += 1
    IS_RUNNING = False

def _action_goto(loc, q):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = loc[0]
    goal.target_pose.pose.position.y = loc[1]
    goal.target_pose.pose.position.z = loc[2]
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]
    ACT_MOVEBASE.send_goal(goal=goal)

def _get_q(loc_c, loc_b):
    dx = loc_c[0] - loc_b[0]
    dy = loc_c[1] - loc_b[1]
    rz = math.atan2(dy, dx)
    q = tf.transformations.quaternion_from_euler(ai=0.0, aj=0.0, ak=rz)
    return q

def _get_locs(loc_c, loc_b, num):
    dx = (loc_c[0] - loc_b[0])/num
    dy = (loc_c[1] - loc_b[1])/num
    locs = list()
    for i in range(1, num+1):
        _x = loc_b[0] + i*dx
        _y = loc_b[1] + i*dy
        locs.append((_x, _y, 0.0))
    return locs

def _pub_smaller_footprint():
    ps = ((-0.01, -0.01), (-0.01,  0.01), ( 0.01,  0.01), ( 0.01, -0.01))
    polygon = Polygon()
    for p in ps:
        point = Point32()
        point.x, point.y, point.z = p[0], p[1], 0.0
        polygon.points.append(point)
    PUB_FOOTPRINT_L.publish(polygon)
    PUB_FOOTPRINT_G.publish(polygon)

def _pub_original_footprint():
    ps = ((-0.225, -0.225), (-0.225, 0.225), (0.225, 0.225), (0.225, -0.225))
    polygon = Polygon()
    for p in ps:
        point = Point32()
        point.x, point.y, point.z = p[0], p[1], 0.0
        polygon.points.append(point)
    PUB_FOOTPRINT_L.publish(polygon)
    PUB_FOOTPRINT_G.publish(polygon)

def _pub_bigger_footprint():
    ps = ((-0.325, -0.325), (-0.325, 0.325), (0.325, 0.325), (0.325, -0.325))
    polygon = Polygon()
    for p in ps:
        point = Point32()
        point.x, point.y, point.z = p[0], p[1], 0.0
        polygon.points.append(point)
    PUB_FOOTPRINT_L.publish(polygon)
    PUB_FOOTPRINT_G.publish(polygon)

def _action_enter(locs, q):

    global I_ENTER, IS_RUNNING

    I_ENTER = 0
    _pub_smaller_footprint()

    rate = rospy.Rate(hz=1)

    while True:
        if IS_RUNNING:
            rate.sleep()
        elif I_ENTER < len(locs):
            IS_RUNNING = True
            loc = locs[I_ENTER]
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = loc[0]
            goal.target_pose.pose.position.y = loc[1]
            goal.target_pose.pose.position.z = loc[2]
            goal.target_pose.pose.orientation.x = q[0]
            goal.target_pose.pose.orientation.y = q[1]
            goal.target_pose.pose.orientation.z = q[2]
            goal.target_pose.pose.orientation.w = q[3]
            ACT_MOVEBASE.send_goal(goal=goal, done_cb=_done_cb)
        else:
            break

    _pub_original_footprint()

def _action_exit(locs, q):

    global I_ENTER, IS_RUNNING

    I_ENTER = 0
    _pub_smaller_footprint()

    rate = rospy.Rate(hz=1)

    while True:
        if IS_RUNNING:
            rate.sleep()
        elif I_ENTER < len(locs):
            IS_RUNNING = True
            loc = locs[I_ENTER]
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = loc[0]
            goal.target_pose.pose.position.y = loc[1]
            goal.target_pose.pose.position.z = loc[2]
            goal.target_pose.pose.orientation.x = q[0]
            goal.target_pose.pose.orientation.y = q[1]
            goal.target_pose.pose.orientation.z = q[2]
            goal.target_pose.pose.orientation.w = q[3]
            ACT_MOVEBASE.send_goal(goal=goal, done_cb=_done_cb)
        else:
            break

    _pub_original_footprint()

def _get_goto_loc(trans, rots, target):
    _, _, rz = tf.transformations.euler_from_quaternion((rots[0], rots[1], rots[2], rots[3]))
    x, y, _ = trans
    choice = {"0": (1.0, 0.0), "1": (0.0, -1.0), "2": (-1.0, 0.0), "3": (0.0, 1.0)}
    dx, dy = choice[target]
    xn = x + dx*math.cos(rz) - dy*math.sin(rz)
    yn = y + dx*math.sin(rz) + dy*math.cos(rz)
    return (xn, yn, 0.0)

def _get_goto_q(rots, target):
    choice = {"0": -math.pi, "1": math.pi/2.0, "2": 0.0, "3": -math.pi/2.0}
    ak = choice[target]
    p = tf.transformations.quaternion_from_euler(ai=0.0, aj=0.0, ak=ak)
    q = tf.transformations.quaternion_multiply(rots, (p[0], p[1], p[2], p[3]))
    return q

def _get_s_center_laser_tf():
    rate = rospy.Rate(hz=10)
    while True:
        try:
            trans, rots = TF_LISTENER.lookupTransform('map', 's_center_laser', rospy.Time(secs=0))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("no tf s_center_laser")
        rate.sleep()
    return (trans, rots)

def _get_base_link_tf():
    rate = rospy.Rate(hz=10)
    while True:
        try:
            trans, rots = TF_LISTENER.lookupTransform('map', 'base_link', rospy.Time(secs=0))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("no tf base_link")
        rate.sleep()
    return (trans, rots)

def cb_goto(request):
    trans, rots = _get_s_center_laser_tf()
    loc = _get_goto_loc(trans=trans, rots=rots, target=request.target)
    q = _get_goto_q(rots=rots, target=request.target)
    _action_goto(loc=loc, q=q)
    return GotoResponse()

def cb_enter(request):
    trans_center, _ = _get_s_center_laser_tf()
    trans_base, _ = _get_base_link_tf()
    locs = _get_locs(loc_c=trans_center, loc_b=trans_base, num=1)
    q = _get_q(loc_c=trans_center, loc_b=trans_base)
    _action_enter(locs=locs, q=q)
    return EmptyResponse()

def cb_exit(request):
    trans, rots = _get_s_center_laser_tf()
    loc = _get_goto_loc(trans=trans, rots=rots, target=request.target)
    locs = _get_locs(loc_c=loc, loc_b=trans, num=100)
    q = _get_goto_q(rots=rots, target=request.target)
    _action_exit(locs=locs, q=q)
    return GotoResponse()

if __name__ == "__main__":

    rospy.init_node('docking', anonymous=False)

    # -- Get parameter
    ns_move_base = rospy.get_param(param_name="~ns_move_base", default="/move_base")

    # -- Get ActionClient, tf Listener
    ACT_MOVEBASE = actionlib.SimpleActionClient(ns=ns_move_base, ActionSpec=MoveBaseAction)
    TF_LISTENER = tf.TransformListener()

    # -- Node function
    rospy.Service(name="~goto", service_class=Goto, handler=cb_goto)
    rospy.Service(name="~enter", service_class=Empty, handler=cb_enter)
    rospy.Service(name="~exit", service_class=Goto, handler=cb_exit)

    PUB_FOOTPRINT_L = rospy.Publisher(name=ns_move_base+"/local_costmap/footprint", data_class=Polygon, queue_size=1)
    PUB_FOOTPRINT_G = rospy.Publisher(name=ns_move_base+"/global_costmap/footprint", data_class=Polygon, queue_size=1)

    rospy.spin()
