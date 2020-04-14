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

        self.pos_noise = rospy.get_param('object_position_noise')
        self.vel_noise = rospy.get_param('object_velocity_noise')
        self.detect_rate = rospy.get_param('object_detection_rate')
        self.detect_range = rospy.get_param('object_detection_dist')

    def get_position(self):
        return self.pos + np.random.randn(3) * self.pos_noise

    def get_velocity(self):
        return self.vel + np.random.randn(3) * self.vel_noise

    def fake_detection(self):
        outObj = Objects()
        outObj.pos = self.get_position()
        outObj.vel = self.get_velocity()
        outObj.dist = self.detect_rate
        outObj.pos_noise = self.pos_noise
        outObj.vel_noise = self.vel_noise
        outObj.detect_rate = self.detect_rate
        outObj.detect_range = self.detect_range
        return outObj
    


class vectFieldController:

    def __init__(self, waypoints = [[0,0,0]]):
        self.v_max =  rospy.get_param('maximum_velocity')
        self.detections = []

        # Waypoint params
        self.waypoints = waypoints
        self.goalPt = 0
        self.goal = self.waypoints[self.goalPt]
        self.switch_dist =  rospy.get_param('switch_waypoint_distance')

        # Orbit params
        self.freq = -1 # Orbit direction (+: CW, -: ccw)
        self.safe_dist = rospy.get_param('safe_distance')
        self.rad = self.safe_dist # Radius of orbit
        self.k_conv =  rospy.get_param('orbit_k_conv') # Gain to converge to orbit
        self.K_theta =  rospy.get_param('heading_k_theta')

        # Go to Goal Parameters
        self.g2g_sig =  rospy.get_param('g2g_sigma')
        self.g2g_sig_sq = self.g2g_sig**2

        # state Information
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.yaw = 0
        self.yaw_rate = 0

        # Publisher Information
        vel_ctrl_pub_name = rospy.get_param('vel_ctrl_sub_name')
        self.vel_ctrl_pub_ = rospy.Publisher(vel_ctrl_pub_name, Joy, queue_size=10)

        # Subscriber Information
        position_sub_name = rospy.get_param('position_pub_name')
        velocity_sub_name = rospy.get_param('velocity_pub_name')
        attitude_sub_name = rospy.get_param('attitude_pub_name')

        rospy.Subscriber(position_sub_name, PointStamped,      self.position_callback, queue_size=1)
        rospy.Subscriber(velocity_sub_name, Vector3Stamped,    self.velocity_callback, queue_size=1)
        rospy.Subscriber(attitude_sub_name, QuaternionStamped, self.attitude_callback, queue_size=1)


    def position_callback(self, msg):
        pt = msg.point
        self.pos = np.array([pt.x, pt.y, pt.z])

    def velocity_callback(self, msg):
        pt = msg.vector
        self.vel = np.array([pt.x, pt.y, pt.z])

    def attitude_callback(self, msg):
        q = msg.quaternion
        r = R.from_quat([q.x, q.y, q.z, q.w])
        [roll, pitch, yaw] = r.as_euler('xyz')
        self.yaw = yaw

    ## TODO: Implement in 3D
    ## For Now: 2D Implementation
    def getXdes(self):
        velDes = np.zeros(4)

        if len(self.waypoints) == 0: return velDes
        
        # Check if we are close to an object
        [closeObject, move] = self.getCloseObject()
        
        # If close to object, orbit
        if all([move, closeObject.dist < self.safe_dist]):
            self.decideOrbitDirection(closeObject)
            velDes[:3] = self.getOrbit(closeObject.pos)    

        else: # Go to goal 
            velDes[:3] = self.goToGoalField()

        # Normalize velocity
        if np.linalg.norm(velDes[:3]) > self.v_max:
            velDes[:3] = velDes[:3]/np.linalg.norm(velDes[:3])*self.v_max
        
        # Heading Control
        w_d = self.headingControl(velDes)
        velDes[3] = w_d

        return velDes

    def move(self):
        # Check if we have reached the next waypoint. If so, update
        self.changeGoalPt()
        
        # Get velocity vector
        velDes = self.getXdes() 
        
        # Publish Vector
        joy_out = Joy()
        joy_out.header.stamp = rospy.Time.now()
        joy_out.axes = [velDes[0], velDes[1], velDes[2],velDes[3],]
        self.vel_ctrl_pub_.publish(joy_out)


    ## TODO: Implement in 3D
    ## For Now: 2D Implementation
    def getCloseObject(self):
        closeObject = Objects()
        move = False

        # Perform transformed coordinates
        T_vo = self.transformToGoalCoords()    
        
        for i in range(len(self.detections)):
            obst = self.detections[i].fake_detection() ########### Rework fake detection with detection node
            obs_pos = obst.pos
            obst.dist = np.linalg.norm(self.pos- obs_pos)
            
            if obst.dist < obst.detect_range and obst.detect_rate > np.random.uniform():
                pos = np.array([obs_pos[0], obs_pos[1], 1])
                obst_trans = T_vo @ pos
                if all([obst.dist < closeObject.dist, obst_trans[1] > 0]):
                    closeObject = obst
                    move = True
        return [closeObject, move]


    def changeGoalPt(self):
        dist_to_goal = np.linalg.norm(self.pos-self.goal)

        if(dist_to_goal < self.switch_dist):
            self.goalPt += 1
            if(self.goalPt > len(self.waypoints)-1):
                self.goalPt = 0
            self.goal =self.waypoints[self.goalPt]


    def headingControl(self, velDes):
        vel_angle = np.arctan2(velDes[1], velDes[0])
        angleDiff = vel_angle - self.yaw
        angleDiff = (angleDiff + np.pi) % (2 * np.pi) - np.pi
        w_d = self.K_theta * angleDiff

        # w_d = vel_angle
        return w_d


    def decideOrbitDirection(self, closeObst):
        # Note: All directions are assuming the vehicle is looking
        # straight at the goal
        
        obst_vel = closeObst.vel
        obst_pos = closeObst.pos
        
        
        # Perform transformed coordinates
        T_vo = self.transformToGoalCoords()
        
        trans_vel = T_vo @ [obst_vel[0], obst_vel[1], 0]
        trans_pos = T_vo @ [obst_pos[0], obst_pos[1], 1]

        # Check if object is stationary
        if np.linalg.norm(trans_vel) > 50:
            if(trans_vel[0] >= 0):          # If obstacle is moving right
                self.freq = 1               # Orbit CW
            else:                           # If obstacle is moving left
                self.freq = -1              # Orbit CCW

        # else object is stationary
        else:
            if(trans_pos[0] >= 0):  # If object is to the right
                self.freq = 1       # Orbit CW
            else:                   # If object is to the left
                self.freq = -1      # Orbit CCW


    ## TODO: Implement in 3D
    ## For Now: 2D Implementation
    def getOrbit(self, center):
        xhat = self.pos[:2] - center[:2] # Change to orbit coords
        gam = self.k_conv*(self.rad**2 - xhat@xhat) # Convergence to orbit

        A = np.array([[gam, self.freq], [-self.freq, gam]]) # Modified harmonic oscillator
        g = A @ xhat[:2]   #  Calculate nominal velocity
        
        # Scale the vector field
        v_g = np.linalg.norm(g)
        g = self.v_max/v_g * g 
        
        # Pad output with z-vel
        velDes = np.array([g[0], g[1], 0])

        return velDes



    def goToGoalField(self):
        g = self.goal - self.pos
        
        # Scale the magnitude of the resulting vector
        dist2goal = np.linalg.norm(g)
        v_g = self.v_max * (1- np.exp(-dist2goal**2/self.g2g_sig_sq))
        
        if dist2goal > 0: # Avoid dividing by zero
            velDes = v_g/dist2goal * g # Dividing by dist is dividing by the norm
        else:
            velDes = np.array([0, 0, 0])
        
        if v_g > self.v_max:
            g = self.v_max/v_g * g

        return velDes


    ## TODO: Implement in 3D
    ## For Now: 2D Implementation
    def transformToGoalCoords(self):

        dp = self.goal - self.pos

        th = np.arctan2(dp[1],dp[0]) - np.pi/2
        th = np.arctan2(np.sin(th), np.cos(th))
        R_ov = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        t_ov = self.pos[:2]

        tempVect = (-1 * R_ov.T) @ t_ov
        T_vo = np.array([[R_ov[0,0], R_ov[1,0], tempVect[0]], [R_ov[0,1], R_ov[1,1], tempVect[1]], [0, 0, 1]])

        return T_vo


if __name__ == '__main__': 
  try:
    rospy.init_node('vectFieldController')

    ## TODO implement subscription to obstacle locations
    ## FOR NOW: Hard code  fake obstacle detections
    obs_x = rospy.get_param('ob_start_x')
    obs_y = rospy.get_param('ob_start_y')
    obs_z = rospy.get_param('ob_start_z')
    ob_start = np.array([obs_x, obs_y, obs_z])
    start_pos = np.array([0,0,0])
    obstacle = Objects(pos=ob_start, dist = np.linalg.norm(ob_start-start_pos))
    detections = [obstacle]

    # Launch Node
    field = vectFieldController()
    field.detections = detections
    field.waypoints  = np.array([[0, 0, 10], 
                                 [0, 20, 10]])
    field.goal = field.waypoints[0]

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        field.move()
        rate.sleep()

    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass

