#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, PointStamped, Point, Vector3, Quaternion
from sensor_msgs.msg import Imu, Joy
import numpy as np
# from PACKAGE_NAME.srv import SERVICE1, SERVICE2, etc


class DJI_simulator:
  def __init__(self, pos_init=np.zeros(3), yaw_init=0):

    ## State Parameters
    self.pos_ = pos_init
    self.vel_ = np.zeros(3)
    self.acc_ = np.zeros(3)
    self.yaw_ = yaw_init
    self.yaw_rate_ = 0

    # Initialize desired state values
    self.pos_des_ = self.pos_
    self.vel_des_ = self.vel_
    self.yaw_des_ = self.yaw_
    self.yaw_rate_des_ = self.yaw_rate_
    self.is_vel_ctrl_ = True

    ## Example Publisher Information
    self.attitude_pub_name = rospy.get_param('attitude_pub_name')
    self.imu_pub_name = rospy.get_param('imu_pub_name')
    self.velocity_pub_name = rospy.get_param('velocity_pub_name')
    self.position_pub_name = rospy.get_param('position_pub_name')
    self.height_pub_name = rospy.get_param('height_pub_name')

    self.attitude_pub_ = rospy.Publisher(self.attitude_pub_name, QuaternionStamped, queue_size=10)
    self.imu_pub_ = rospy.Publisher(self.imu_pub_name, Imu, queue_size=10)
    self.velocity_pub_ = rospy.Publisher(self.velocity_pub_name, Vector3Stamped, queue_size=10)
    self.position_pub_ = rospy.Publisher(self.position_pub_name, PointStamped, queue_size=10)
    self.height_pub_ = rospy.Publisher(self.height_pub_name, Float32, queue_size=10)


    ## Subscriber Information
    self.pos_ctrl_sub_name = rospy.get_param('pos_ctrl_sub_name')
    self.vel_ctrl_sub_name = rospy.get_param('vel_ctrl_sub_name')

    # rospy.Subscriber(self.pos_ctrl_sub_name, Joy, self.pos_ctrl_callback, queue_size=10)
    rospy.Subscriber(self.vel_ctrl_sub_name, Joy, self.vel_ctrl_callback, queue_size=10)

    ## Position Control Parameters
    A_x = rospy.get_param('pos_A_x')
    A_y = rospy.get_param('pos_A_y')
    A_z = rospy.get_param('pos_A_z')
    B_u = rospy.get_param('pos_B_u')
    B_v = rospy.get_param('pos_B_v')
    B_w = rospy.get_param('pos_B_w')
    K_pos = rospy.get_param('pos_K_pos_coeff')
    K_vel = rospy.get_param('pos_K_vel_coeff')


    self.A_pos_ = np.zeros([6,6])
    self.A_pos_[:3,-3:] = np.diag([A_x, A_y, A_z])
    self.B_pos_ = np.zeros([6,3])
    self.B_pos_[-3:] = np.diag([B_u, B_v, B_w])

    self.K_pos_ = np.hstack((K_pos*np.identity(3), K_vel*np.identity(3)))
    self.K_yaw_ = rospy.get_param('K_yaw')


    ## Velocity Control Parameters
    B_u = rospy.get_param('vel_B_u')
    B_v = rospy.get_param('vel_B_v')
    B_w = rospy.get_param('vel_B_w')

    self.A_vel_ = np.zeros([3,3])
    self.B_vel_ = np.diag([1, 1, 1])
    self.K_vel_ = np.diag([1,1,1])
    self.K_yaw_rate_ = rospy.get_param('K_yaw_rate')


    ## Last Update Time
    self.last_update_time_ = rospy.Time.now()


      ## Service Server Information
      # self.serv_ = rospy.Service(SERVICE_NAME, SERVICE_TYPE, SERVICE_CALLBACK)



  ## Service Client Information
  # def server_client
  #     rospy.wait_for_service(SERVICE_NAME)
  #     try:

  #         service_input = []

  #         service_var = rospy.ServiceProxy(SERVICE_NAME, SERVICE_TYPE)
  #         service_response = service_var(service_input)
  #         return service_response.data
  #     except rospy.ServiceException, e:
  #         print "Service call failed: %s"%e


# TODO Implement Position Control
  def pos_ctrl_callback(self, received_data):
    # Parse Input
    # self.last_update_time_ = received_data.Header.stamp
    self.pos_des_ = received_data.axes[:3]
    self.yaw_des_ = received_data.axes[3]
    self.is_vel_ctrl_ = False
    self.performMotion() ###### CONSIDER REMOVING THIS LINE #######




  def vel_ctrl_callback(self, received_data):
    # Parse Input
    # self.last_update_time_ = received_data.Header.stamp
    self.vel_des_ = received_data.axes[:3]
    self.yaw_rate_des_ = received_data.axes[3]
    self.is_vel_ctrl_ = True
    self.performMotion() ###### CONSIDER REMOVING THIS LINE #######
  

  def publishData(self):    
    head = Header()
    head.stamp = rospy.Time.now()

    # Publish Attitude
    myQuat = self.getQuat
    outQuat = QuaternionStamped()
    outQuat.header = head
    outQuat.quaternion = myQuat
    self.attitude_pub_.publish(outQuat)

    # Publish IMU
    imuOut = Imu()
    imuOut.header = head
    imuOut.orientation = outQuat
    imuOut.angular_velocity = Vector3(np.array([0, 0, self.yaw_rate_]))
    imuOut.linear_acceleration = Vector3(self.acc_)
    self.imu_pub_.publish(imuOut)

    # Publish Velocity
    velOut = Vector3Stamped()
    velOut.header = head
    velOut.vector = Vector3(self.vel_)
    self.velocity_pub_.publish(velOut)

    # Publish Position
    posStamp = PointStamped()
    posStamp.header = head
    posStamp.point = Point(self.pos_)
    self.position_pub_.publish(posStamp)

    # Publish Height Above Ground
    height = Float32()
    height.data = self.pos_(-1)
    self.height_pub_.publish(height)


  # TODO: Add in smart way to play out the quaternion
  # FOR NOW: Always return orientation straight up
  def getQuat(self):
    q = np.array([0, 0, 0, 1])
    quat = Quaternion(q)
    return quat


  def performMotion(self):
    oldTime = self.last_update_time_
    self.last_update_time_ = rospy.Time.now()
    dt = oldTime - self.last_update_time_

    if self.is_vel_ctrl_:
      # calculate errors
      vel_error = self.vel_ - self.vel_des_
      yaw_rate_error = self.yaw_rate_ - self.yaw_rate_des_

      # Calculate New Input
      velDot = (self.A_vel_ - self.B_vel_ @ self.K_vel_) @ vel_error
      yaw_rate_dot = self.K_yaw_rate_ * yaw_rate_error

      # TODO: Update State using Integration (ODE45)
      # FOR NOW: Euler Integration
      self.vel_ += velDot * dt
      self.pos_ += self.vel_ * dt
      self.yaw_rate_ += yaw_rate_dot * dt
      self.yaw_ += self.yaw_rate_ * dt
      self.acc_ = velDot

    else: # We are in position control
      # calculate errors
      pos_error = self.pos_ - self.pos_des_
      yaw_error = self.yaw_ - self.yaw_des_

      # Calculate New Input
      posDot = (self.A_pos_ - self.B_pos_ @ self.K_pos_) @ pos_error
      yawDot = self.K_yaw_ * yaw_error

      # TODO: Update State using Integration (ODE45)
      # FOR NOW: Euler Integration
      self.vel_ += posDot[-3:] * dt
      self.pos_ += self.vel_ * dt
      self.yaw_rate_ += yawDot * dt
      self.yaw_ += self.yaw_rate_ * dt
      self.acc_ = posDot[-3:]
      

    self.publishData()



    


if __name__ == '__main__': 
  try:
    rospy.init_node('dji_sdk_sim')

    
    sim = DJI_simulator()

    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass