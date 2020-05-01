#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, PointStamped, Point, Vector3, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from dji_m600_sim.msg import ObstacleDetection, ObstacleDetectionArray
import copy


pub = rospy.Publisher('true_obstacles', MarkerArray, queue_size=10)
sphere_rad = rospy.get_param('kickball_radius')


def trueMarkerCallback(msg):
    markerArray = MarkerArray()

    for detect in msg.detections:
        marker1 = Marker()
        marker1.header.stamp = rospy.Time.now()
        marker1.header.frame_id = "/world"
        marker1.id = detect.id
        marker1.type = marker1.SPHERE
        marker1.action = marker1.ADD
        marker1.pose.position.x = detect.position.x
        marker1.pose.position.y = detect.position.y
        marker1.pose.position.z = detect.position.z
        marker1.pose.orientation.w = 1
        marker1.scale.x = sphere_rad*2
        marker1.scale.y = sphere_rad*2
        marker1.scale.z = sphere_rad*2

        marker1.color.r = 1.0
        marker1.color.g = 0
        marker1.color.b = 0
        marker1.color.a = 0.7
        marker1.lifetime = rospy.Duration.from_sec(0)
        marker1.frame_locked = 0
        markerArray.markers.append(marker1)


    pub.publish(markerArray)


if __name__ == '__main__': 
  try:
    rospy.init_node('marker_displayer')

    rospy.Subscriber(rospy.get_param('true_obstacle_topic'), ObstacleDetectionArray, trueMarkerCallback, queue_size=1)
    # Subscribe to Obstacle Positions
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass