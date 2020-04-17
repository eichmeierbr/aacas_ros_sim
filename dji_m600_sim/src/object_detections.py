#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, PointStamped, Point, Vector3, Quaternion
from dji_m600_sim.msg import ObstacleDetection, ObstacleDetectionArray
from dji_m600_sim.srv import QueryDetections, QueryDetectionsResponse
import copy

class DetectionSimulation:
  def __init__(self):
    self.true_detections_ = []
    self.sim_detections_ = ObstacleDetectionArray()

    # Publisher for the detections
    self.detection_pub_ = rospy.Publisher('Detected_Obstacles', ObstacleDetectionArray, queue_size=1)

    # Service to report detections
    detections_service_name = rospy.get_param('query_detections_service')
    self.task_ctrl_service_ = rospy.Service(detections_service_name, QueryDetections, self.reportDetections)

  def updateDetections(self, pos=np.zeros(3), quat=[0,0,0,1]):
    sim_detections = ObstacleDetectionArray()
    for detection in self.true_detections_:
      if detection.detect_rate > np.random.uniform() and detection.dist < detect.detect_range:
        obj = detection.fake_detection()
        dist = np.linalg.norm(pos - np.array([obj.pos[0], obj.pos[1], obj.pos[2]]))

        out_detection = ObstacleDetection()
        out_detection.type = detection.class_name
        out_detection.position = Point(obj.pos[0], obj.pos[1], obj.pos[2]) 
        out_detection.velocity = Vector3(obj.vel[0], obj.vel[1], obj.vel[2]) 
        out_detection.distance = dist
        sim_detections.detections.append(out_detection)

    self.sim_detections_ = sim_detections


  def publishDetections(self):
    self.updateDetections()
    self.detection_pub_.publish(self.sim_detections_)


  def reportDetections(self, req):
    veh_pos = [req.vehicle_position.x, req.vehicle_position.y, req.vehicle_position.z]
    self.updateDetections(pos=veh_pos, quat=req.attitude)
    return self.sim_detections_




class Objects:
  def __init__(self, class_name='ball', pos = np.zeros(3), vel=np.zeros(3), dist = np.inf):
      self.class_name = class_name
      self.pos = pos
      self.vel = vel
      self.dist = dist
      self.pos_noise = rospy.get_param('object_position_noise')
      self.vel_noise = rospy.get_param('object_velocity_noise')
      self.detect_rate = rospy.get_param('object_detection_rate')
      self.detect_range = rospy.get_param('object_detection_dist')

  def get_position(self):
      return self.pos + np.random.randn(3) * self.pos_noise

  def get_velocity(self):
      return self.vel + np.random.randn(3) * self.vel_noise

  def fake_detection(self):
      outObj = copy.copy(self)
      outObj.pos = self.get_position()
      outObj.vel = self.get_velocity()
      return outObj



if __name__ == '__main__': 
  try:
    rospy.init_node('simulated_detector')

    detector = DetectionSimulation()

    obs_x = rospy.get_param('ob_start_x')
    obs_y = rospy.get_param('ob_start_y')
    obs_z = rospy.get_param('ob_start_z')
    ob_start = np.array([obs_x, obs_y, obs_z])
    obstacle = Objects(pos=ob_start)

    detector.true_detections_ = [obstacle]

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # detector.publishDetections()
        rate.sleep()

    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass

