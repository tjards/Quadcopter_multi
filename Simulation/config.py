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
        self.Tf = 20
        self.ifsave = 1
         
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
    
    #return orient, usePrecession, Ti, Ts, Tf, ifsave, ctrlOptions, trajSelect, ctrlType, trajSelect
    
