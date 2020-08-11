#! /usr/bin/env python

import math
import random
import threading

import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
import tf
import tf2_ros


class TfListener:
    """
    Use threading to get listener tf odom -> base_link, and save them to global vars.
    Attributes:
        _tf_buffer (tf2_ros.Buffer):
    """
    def __init__(self):
        self._tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tf_buffer)  # for buffer

    def _job(self):
        '''
        Get tf odom to frame_in
        '''
        rate = rospy.Rate(hz=10.0)
        while not rospy.is_shutdown():
            try:
                t = self._tf_buffer.lookup_transform(
                    target_frame="odom",
                    source_frame=FRAME_IN,
                    time=rospy.Time())
            except Exception as err:
                rospy.loginfo(err)
            else:
                global BIG_CAR_P, BIG_CAR_Q
                BIG_CAR_P[0] = t.transform.translation.x
                BIG_CAR_P[1] = t.transform.translation.y
                BIG_CAR_Q[0] = t.transform.rotation.x
                BIG_CAR_Q[1] = t.transform.rotation.y
                BIG_CAR_Q[2] = t.transform.rotation.z
                BIG_CAR_Q[3] = t.transform.rotation.w
            rate.sleep()

    def start_thread(self):
        '''
        Start threading
        '''
        thread = threading.Thread(target=self._job, name='job')
        thread.start()


def get_fake_big_car_tf():
    """
    Get tf odom -> frame_out
    Which is combined by frame_in and imu information
    """
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = FRAME_OUT
    t.transform.translation.x = BIG_CAR_P[0] + (random.random()-0.5)*2.0*DELTA_M
    t.transform.translation.y = BIG_CAR_P[1] + (random.random()-0.5)*2.0*DELTA_M
    t.transform.translation.z = 0.0
    _, _, theta = tf.transformations.euler_from_quaternion(BIG_CAR_Q)
    theta += (random.random()-0.5)*2.0*DELTA_DEG/180.0*math.pi
    _q = tf.transformations.quaternion_from_euler(ai=0, aj=0, ak=theta)
    t.transform.rotation.x = _q[0]
    t.transform.rotation.y = _q[1]
    t.transform.rotation.z = _q[2]
    t.transform.rotation.w = _q[3]
    return t

if __name__ == '__main__':

    # -- global vars
    BIG_CAR_P = [0.0, 0.0]  # frame_in pose (tf odom to base_link)
    BIG_CAR_Q = [0.0, 0.0, 0.0, 1.0]  # frame_in orientation (tf odom to base_link)

    # -- ros node function
    ## -- parameters
    rospy.init_node('fake_big_car_br')

    FRAME_IN = rospy.get_param(param_name="~frame_in")
    FRAME_OUT = rospy.get_param(param_name="~frame_out")
    pub_rate = rospy.get_param(param_name="~publish_rate")
    DELTA_M = rospy.get_param(param_name="~delta_m")
    DELTA_DEG = rospy.get_param(param_name="~delta_deg")

    # -- tf listenser from big_car
    tf_listener = TfListener()
    tf_listener.start_thread()

    # -- tf broadcaster for fake big car
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(hz=int(pub_rate))
    try:
        while not rospy.is_shutdown():
            fake_big_car_tf = get_fake_big_car_tf()
            br.sendTransform(transform=fake_big_car_tf)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
