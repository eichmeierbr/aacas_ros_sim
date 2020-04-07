import numpy as np


class Objects:
    def __init__(self, pos = np.zeros(3), vel=np.zeros(3), dist = np.inf):
        self.pos = pos
        self.vel = vel
        self.dist = dist


class vectFieldController:

    def __init__(self, safe = 5.5):
        self.v_max = 5
        self.detections = []

        # Waypoint params
        self.waypoints = np.array([[0, 20, 0], [0, 0, 0]])
        self.goalPt = 0
        self.goal = self.waypoints[self.goalPt]
        self.switch_dist = 1

        # Orbit params
        self.freq = -1 # Orbit direction (+: CW, -: ccw)
        self.safe_dist = safe
        self.rad = self.safe_dist - 1 # Radius of orbit
        self.k_conv = .01 # Gain to converge to orbit
        self.K_theta = 2.0

        # Go to Goal Parameters
        self.g2g_sig = 1.5
        self.g2g_sig_sq = self.g2g_sig**2

        # Control Information
        self.A = np.zeros([4,4])
        self.B = np.identity(4)
        self.K = np.diag([1,1,1,1])
        self.t = 0
        self.dt = 0.005
        self.veh_state = np.zeros(12)

    def getXdes(self):
        velDes = np.zeros(4)      
        
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

    def move(self,inState):
        self.veh_state = inState
            
        # Check if we have reached the next waypoint. If so, update
        self.changeGoalPt()
        
        x_out = np.zeros(12)
        velDes = self.getXdes() # Get velocity vector
        
        velDot = (self.A - self.B @ self.K) @ (self.veh_state[3:7] - velDes) # Control ################ NEEDS DEBUGGING
        
        # Prepare Output Vector
        x_out[3:6] = self.veh_state[3:6] + velDot[:3] * self.dt # Update Velocity
        x_out[:3]  = self.veh_state[:3]  + x_out[3:6] * self.dt # Update Position
        x_out[9]   = self.veh_state[9]   + velDot[3]  * self.dt # Update yaw velocity
        x_out[6]   = self.veh_state[6]   + x_out[9]   * self.dt # Update Yaw position

        self.veh_state = x_out.copy()

        return x_out


############### NEED TO IMpLEMENT OBSTACLE FOR THIS TO WORK  #############################
    def getCloseObject(self):
        closeObject = Objects()
        move = False

        # Perform transformed coordinates
        T_vo = self.transformToGoalCoords()    
        
        for i in range(len(self.detections)):
            obst = self.detections[i]
            pos = np.array([obst.pos[0], obst.pos[1], 1])
            obst_trans = T_vo @ pos
            if all([obst.dist < closeObject.dist, obst_trans[1] > 0]):
                closeObject = obst
                move = True
        return [closeObject, move]


    def changeGoalPt(self):
        pos = self.veh_state[:3]
        dist_to_goal = np.linalg.norm(pos-self.goal)

        if(dist_to_goal < self.switch_dist):
            self.goalPt += 1
            if(self.goalPt > len(self.waypoints)-1):
                self.goalPt = 0


    def headingControl(self, velDes):
        # vel_angle = wrapToPi(atan2(velDes(2), velDes(1)));
        # angleDiff = vel_angle - veh_state(7);
        # w_d = self.K_theta * angleDiff;

        w_d = 0
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
        if np.linalg.norm(trans_vel) > 0.1:
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



    def getOrbit(self, center):
        xhat = self.veh_state[:2] - center[:2] # Change to orbit coords
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
        g = self.goal - self.veh_state[:3]
        
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


    def transformToGoalCoords(self):

        dp = self.goal - self.veh_state[:3]

        th = np.arctan2(dp[1],dp[0]) - np.pi/2
        th = np.arctan2(np.sin(th), np.cos(th))
        R_ov = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        t_ov = self.veh_state[:2]

        tempVect = (-1 * R_ov.T) @ t_ov
        T_vo = np.array([[R_ov[0,0], R_ov[1,0], tempVect[0]], [R_ov[0,1], R_ov[1,1], tempVect[1]], [0, 0, 1]])

        return T_vo


field = vectFieldController()

ob_start = np.array([0,5,0])
obstacle = Objects(pos=ob_start, dist = np.linalg.norm(ob_start-field.veh_state[:3]))
field.detections = [obstacle]
print(field.getXdes())
print(field.move(field.veh_state))