#!/usr/bin/env python

import time

import rospy
from geometry_msgs.msg import Polygon, Point32

def set_smaller(pub_local, pub_global):
    ps = ((-0.01, -0.01), (-0.01,  0.01), ( 0.01,  0.01), ( 0.01, -0.01))
    polygon = Polygon()
    for p in ps:
        point = Point32()
        point.x, point.y, point.z = p[0], p[1], 0.0
        polygon.points.append(point)
    pub_local.publish(polygon)
    pub_global.publish(polygon)
    time.sleep(0.5)

def set_original(pub_local, pub_global):
    ps = ((-0.225, -0.225), (-0.225, 0.225), (0.225, 0.225), (0.225, -0.225))
    polygon = Polygon()
    for p in ps:
        point = Point32()
        point.x, point.y, point.z = p[0], p[1], 0.0
        polygon.points.append(point)
    pub_local.publish(polygon)
    pub_global.publish(polygon)
    time.sleep(0.5)

def set_bigger(pub_local, pub_global):
    ps = ((-0.325, -0.325), (-0.325, 0.325), (0.325, 0.325), (0.325, -0.325))
    polygon = Polygon()
    for p in ps:
        point = Point32()
        point.x, point.y, point.z = p[0], p[1], 0.0
        polygon.points.append(point)
    pub_local.publish(polygon)
    pub_global.publish(polygon)
    time.sleep(0.5)
