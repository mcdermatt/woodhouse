#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sensor_msgs
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
import tf
import numpy as np

class Keyframe:
    def __init__(self, position, orientation, point_cloud, descriptor=None, id=None, timestamp=None):
        self.position = position  # [x, y, z]
        self.orientation = orientation  # Quaternion [qx, qy, qz, qw]
        self.point_cloud = point_cloud  # Full point cloud
        self.descriptor = descriptor  # Optional pose descriptor
        self.id = id  # Unique identifier
        self.timestamp = timestamp  # Optional metadata

class keyframe_grabber():

    def __init__(self, point_cloud_topic = '/velodyne_points'):

        rospy.init_node('keyframe_grabber_node', anonymous=True)
        self.pcsub = rospy.Subscriber(point_cloud_topic, PointCloud2, self.pc_callback)
        self.tf_listener = tf.TransformListener()

        self.skip_thresh = 10 #at a maximum, look at every <skip_thresh>th scan
        self.move_thresh = 2 #need to be this far from last keyframe

        self.frames_since_last_kf = 0
        self.pose_at_last_kf = np.zeros([3])

    def pc_callback(self, msg):

        # print("got a point cloud")

        #get current estimate of platform location from odometry tf broadcast
        try:
            self.tf_listener.waitForTransform('/map', '/sensor', rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/sensor', rospy.Time(0))
            print("Translation:", trans)
            print("Rotation:", rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"Transform error: {e}")

        #Decide whether or not to add keyframe
        #has enough time elapsed? (i.e. only consider every 10th+ pc) 
        if self.frames_since_last_kf < self.skip_thresh:
            self.frames_since_last_kf += 1
            return

        #has the platform moved far enough since the last keyframe?
        movement_since_last_kf = np.sqrt((self.pose_at_last_kf[0] - trans[0])**2 
                                        + (self.pose_at_last_kf[1] - trans[1])**2
                                        + (self.pose_at_last_kf[2] - trans[2])**2 )
        print(movement_since_last_kf)
        if movement_since_last_kf > self.move_thresh:
            print("saving keyframe")
            self.pose_at_last_kf = trans

if __name__ == '__main__':
    kg = keyframe_grabber()

    while not rospy.is_shutdown():
        rospy.spin()