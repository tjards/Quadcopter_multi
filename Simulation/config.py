# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""


import numpy as np



class config():

    
    def __init__(self):

        # Select Orientation of Quadcopter and Reference Frame
        # ---------------------------
        # "NED" for front-right-down (frd) and North-East-Down
        # "ENU" for front-left-up (flu) and East-North-Up
        self.orient = "NED"
        
        # Select whether to use gyroscopic precession of the rotors in the quadcopter dynamics
        # ---------------------------
        # Set to False if rotor inertia isn't known (gyro precession has negigeable effect on drone dynamics)
        self.usePrecession = bool(False)
        
        # Simulation Setup
        # --------------------------- 
        self.Ti = 0
        self.Ts = 0.005 #default 0.005 (larger numbers could result in instability)
        self.Tf = 26 #1500 # 26
        self.ifsave = 1
        self.ifsavedata = 0     # subset of ifsave
        self.trialLen = 3
        self.wpType = 0   # [0 = fixed, 1 = random, 2 = TBD , 3 = learning (linked to doLearn below)] 
        self.nVeh = 2     # only tested for 1 or 2 right now
        
        # Choose trajectory settings
        # --------------------------- 
        ctrlOptions = ["xyz_pos", "xy_vel_z_pos", "xyz_vel"]
        self.trajSelect = np.zeros(3)
        
        # Select Control Type             (0: xyz_pos,                  1: xy_vel_z_pos,            2: xyz_vel)
        self.ctrlType = ctrlOptions[0]   
        # Select Position Trajectory Type (0: hover,                    1: pos_waypoint_timed,      2: pos_waypoint_interp,    
        #                                  3: minimum velocity          4: minimum accel,           5: minimum jerk,           6: minimum snap
        #                                  7: minimum accel_stop        8: minimum jerk_stop        9: minimum snap_stop
        #                                 10: minimum jerk_full_stop   11: minimum snap_full_stop
        #                                 12: pos_waypoint_arrived
        self.trajSelect[0] = 1         
        # Select Yaw Trajectory Type      (0: none                      1: yaw_waypoint_timed,      2: yaw_waypoint_interp     3: follow          4: zero)
        self.trajSelect[1] = 4           
        # Select if waypoint time is used, or if average speed is used to calculate waypoint time   (0: waypoint time,   1: average speed)
        self.trajSelect[2] = 0 
        
        # Choose Obstacle Avoidance settings
        # ----------------------------------
        self.PIC = 0    # do we want to using planar inequality constraint shifting?
        self.PF = 1     # do we want to use potential fields?
        
        # Create obstacle(s)
        # -----------------------------------------
        self.o1 = np.array([-2.1, 0, -3.5],)           # obstacle 1 (x,y,z)
        self.o2 = np.array([2, -1.2, 0.9])           # obstacle 2 (x,y,z)
        self.o3 = np.array([0, 2.5, -2.5])           # obstacle 3 (x,y,z)
        
        # Learning stuff
        # ---------------
        self.doLearn = 0
        #self.nParams = 14       # moved lower based on the control architecture 
        self.nOptions = 7
        self.optionsInterval = [0.1,5] 
        self.wpRange = 1
        self.learnRate = 0.1
        self.a = 1               # weight of positive reinforcement (default one)
        self.b = 0               # weight of negative reinforcement (default zero)
        self.learnWhat = [0, 0, 0, 1]
        #                [1 = pos (2), 1 = vel (6), 1 = att (2), 1 = rate (4)]   
        self.nParams = 2*self.learnWhat[0] + 6*self.learnWhat[1] + 2*self.learnWhat[2] + 4*self.learnWhat[3] 
        if self.doLearn == 1:
            self.wpType = 3     # wp type must be 3 for learning (see waypoints.py)
    
    
        
    
    #return orient, usePrecession, Ti, Ts, Tf, ifsave, ctrlOptions, trajSelect, ctrlType, trajSelect
    
