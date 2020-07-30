#! /usr/bin/env python

import threading

import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
import tf
import tf2_ros
import tf_conversions


class TfListener:
    """
    Use threading to get listener tf odom -> big_car, and save them to global vars.
    Attributes:
        _tf_buffer (tf2_ros.Buffer):
    """
    def __init__(self):
        self._tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tf_buffer)  # for buffer

    def _job(self):
        '''
        Update big_car's  pose and orientation in loop.
        '''
        rate = rospy.Rate(hz=10.0)
        while not rospy.is_shutdown():
            try:
                t = self._tf_buffer.lookup_transform(
                    target_frame=frames["odom"],
                    source_frame=frames["big_car"],
                    time=rospy.Time())
            except Exception as err:
                rospy.loginfo(err)
            else:
                global BIG_CAR_P
                BIG_CAR_P[0] = t.transform.translation.x
                BIG_CAR_P[1] = t.transform.translation.y
            rate.sleep()

    def start_thread(self):
        '''
        Start threading
        '''
        thread = threading.Thread(target=self._job, name='job')
        thread.start()


def _cb_car1_imu(msg):
    global CAR1_Q
    CAR1_Q[0] = msg.orientation.x
    CAR1_Q[1] = msg.orientation.y
    CAR1_Q[2] = msg.orientation.z
    CAR1_Q[3] = msg.orientation.w

def _cb_car2_imu(msg):
    global CAR2_Q
    CAR2_Q[0] = msg.orientation.x
    CAR2_Q[1] = msg.orientation.y
    CAR2_Q[2] = msg.orientation.z
    CAR2_Q[3] = msg.orientation.w

def get_car1_tf():
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"
    t.child_frame_id = "car1"
    t.transform.translation.x = 0.465
    t.transform.translation.y = 0.0
    t.transform.translation.z = -0.3
    t.transform.rotation.x = CAR1_Q[0]
    t.transform.rotation.y = CAR1_Q[1]
    t.transform.rotation.z = CAR1_Q[2]
    t.transform.rotation.w = CAR1_Q[3]
    return t

def get_car2_tf():
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"
    t.child_frame_id = "car2"
    t.transform.translation.x = -0.465
    t.transform.translation.y = 0.0
    t.transform.translation.z = -0.3
    t.transform.rotation.x = CAR2_Q[0]
    t.transform.rotation.y = CAR2_Q[1]
    t.transform.rotation.z = CAR2_Q[2]
    t.transform.rotation.w = CAR2_Q[3]
    return t

if __name__ == '__main__':

    # -- global vars
    BIG_CAR_P = [0.0, 0.0]  # big_car pose (from odom)
    CAR1_Q = [0.0, 0.0, 0.0, 1.0]  # car1 imu orientation (initial zero, from odom?)
    CAR2_Q = [0.0, 0.0, 0.0, 1.0]  # car2 imu orientation (initial zero, from odom?)

    # -- ros node function
    ## -- parameters
    rospy.init_node('big_car_tf_br')

    top_car1_imu = rospy.get_param(param_name="~car1_imu", default="car1_imu")
    top_car2_imu = rospy.get_param(param_name="~car2_imu", default="car2_imu")

    ## -- subscriber
    rospy.Subscriber(name=top_car1_imu, data_class=Imu, callback=_cb_car1_imu)
    rospy.Subscriber(name=top_car2_imu, data_class=Imu, callback=_cb_car2_imu)

    # -- tf listenser for big_car
    tf_listener = TfListener()
    tf_listener.start_thread()

    # -- tf broadcaster for car1, car2
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(hz=10)
    try:
        while not rospy.is_shutdown():
            car1_tf = get_car1_tf()
            car2_tf = get_car2_tf()
            br.sendTransform(transform=car1_tf)
            br.sendTransform(transform=car2_tf)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
