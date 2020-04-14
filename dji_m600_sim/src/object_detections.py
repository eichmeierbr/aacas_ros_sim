#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, PointStamped, Point, Vector3, Quaternion
from scipy.spatial.transform import Rotation as R


class Objects:
    def __init__(self, pos = np.zeros(3), vel=np.zeros(3), dist = np.inf):
        self.pos = pos
        self.vel = vel
        self.dist = dist


if __name__ == '__main__': 
  try:
    rospy.init_node('object_detector')

    ## TODO implement subscription to obstacle locations
    ## FOR NOW: Hard code  fake obstacle detections
    ob_start = np.array([0,10,10])
    start_pos = np.array([0,0,0])
    obstacle = Objects(pos=ob_start, dist = np.linalg.norm(ob_start-start_pos))
    detections = [obstacle]
    detector = vectFieldController(detects=detections)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        field.move()
        rate.sleep()

    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass

