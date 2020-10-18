# -*- coding: utf-8 -*-
"""
The purpose of this project is to implement Reinforcement Learning
(specifically, Finite Action-set Learning Automata) 
to tune the PID controller gains of a simulated Quadcopter
+ multi-vehicle 
+ obstacle avoidance

Track the full project status here:
    https://github.com/users/tjards/projects/3


    
editing author: P. Travis Jardine, PhD
email: travis.jardine@gmail.com 

The vehicle dynamics, controllers, and other files have been modified from the
original implementation of:

author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!

 

"""

import numpy as np
import matplotlib.pyplot as plt
import time
#import cProfile

from trajectory import Trajectory
from ctrl import Control
from quadFiles.quad import Quadcopter
from utils.windModel import Wind
import utils
import config

# my libraries
from fala import falaObj 
from potentialField import potentialField as pf


def quad_sim(t, Ts, quad, ctrl, wind, traj, fala, obsPF):
    
    # Dynamics (using last timestep's commands)
    # ---------------------------
    quad.update(t, Ts, ctrl.w_cmd, wind)
    t += Ts

    # Learn from the trial
    # --------------------------
    fala.learn(quad,traj,ctrl,Ts,t)
    
    # Trajectory for Desired States (for next iteration)
    # ---------------------------
    sDes = traj.desiredState(t, Ts, quad)

    # Update the trajectory for obstacles with potential fields 
    # ---------------------------    
    
    # ~~~~ update obstacle positions (if required) ~~~ #
    #o1 = np.array([1,1,1])                      # obstacle 1 (x,y,z)
    #o2 = np.array([-2,-1,-3])                   # obstacle 2 (x,y,z)
    #obsPF.Po = np.vstack((o1,o2)).transpose()   # stack obstacles
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
    
    obsPF.updateTraj(quad.state[0:3],traj.sDes[0:3],traj)

    # Generate Commands (for next iteration)
    # ---------------------------
    ctrl.controller(traj, quad, sDes, Ts)

    return t
    

def main():
    start_time = time.time()

    # Simulation Setup
    # --------------------------- 
    Ti = 0
    Ts = 0.005 #default 0.005 (larger numbers could result in instability)
    Tf = 20
    ifsave = 1

    # Choose trajectory settings
    # --------------------------- 
    ctrlOptions = ["xyz_pos", "xy_vel_z_pos", "xyz_vel"]
    trajSelect = np.zeros(3)

    # Select Control Type             (0: xyz_pos,                  1: xy_vel_z_pos,            2: xyz_vel)
    ctrlType = ctrlOptions[0]   
    # Select Position Trajectory Type (0: hover,                    1: pos_waypoint_timed,      2: pos_waypoint_interp,    
    #                                  3: minimum velocity          4: minimum accel,           5: minimum jerk,           6: minimum snap
    #                                  7: minimum accel_stop        8: minimum jerk_stop        9: minimum snap_stop
    #                                 10: minimum jerk_full_stop   11: minimum snap_full_stop
    #                                 12: pos_waypoint_arrived
    trajSelect[0] = 1         
    # Select Yaw Trajectory Type      (0: none                      1: yaw_waypoint_timed,      2: yaw_waypoint_interp     3: follow          4: zero)
    trajSelect[1] = 4           
    # Select if waypoint time is used, or if average speed is used to calculate waypoint time   (0: waypoint time,   1: average speed)
    trajSelect[2] = 0           
    print("Control type: {}".format(ctrlType))

    # Initialize Quadcopter, Controller, Wind, Result Matrixes
    # ---------------------------
    quad = Quadcopter(Ti)
    traj = Trajectory(quad, ctrlType, trajSelect)
    ctrl = Control(quad, traj.yawType)
    wind = Wind('None', 2.0, 90, -15)
    
    # Create learning object
    # ---------------------------
    # for the controller
    fala = falaObj(nParams=14, nOptions=10, optionsInterval=[0.1,2], learnRate=0.15, trialLen=3)
    
    # Trajectory for First Desired States
    # ---------------------------
    sDes = traj.desiredState(0, Ts, quad)    

    # Create a Potential Field object
    # note: there is something wrong with the obstacle positions
    # -------------------------------
    o1 = np.array([-2, -1, -3])                  # obstacle 1 (x,y,z)
    o2 = np.array([3, -2, 1])               # obstacle 2 (x,y,z)
    Po = np.vstack((o1,o2)).transpose()     # stack obstacles
    obsPF = pf(traj, Po, gamma=1, eta=0.5, obsRad=1)
        
    # Generate First Commands
    # ---------------------------
    ctrl.controller(traj, quad, sDes, Ts)
    
    # Initialize Result Matrixes
    # ---------------------------
    numTimeStep = int(Tf/Ts+1)
    
    # TRAVIS will run collect_init() here

    t_all          = np.zeros(numTimeStep)
    s_all          = np.zeros([numTimeStep, len(quad.state)])
    pos_all        = np.zeros([numTimeStep, len(quad.pos)])
    vel_all        = np.zeros([numTimeStep, len(quad.vel)])
    quat_all       = np.zeros([numTimeStep, len(quad.quat)])
    omega_all      = np.zeros([numTimeStep, len(quad.omega)])
    euler_all      = np.zeros([numTimeStep, len(quad.euler)])
    sDes_traj_all  = np.zeros([numTimeStep, len(traj.sDes)])
    sDes_calc_all  = np.zeros([numTimeStep, len(ctrl.sDesCalc)])
    w_cmd_all      = np.zeros([numTimeStep, len(ctrl.w_cmd)])
    wMotor_all     = np.zeros([numTimeStep, len(quad.wMotor)])
    thr_all        = np.zeros([numTimeStep, len(quad.thr)])
    tor_all        = np.zeros([numTimeStep, len(quad.tor)])
    # learning items
    #falaError_all  = np.zeros([numTimeStep, len(fala.error_accumulated)])
    falaError_all  = np.zeros([numTimeStep, 1])
    

    t_all[0]            = Ti
    s_all[0,:]          = quad.state
    pos_all[0,:]        = quad.pos
    vel_all[0,:]        = quad.vel
    quat_all[0,:]       = quad.quat
    omega_all[0,:]      = quad.omega
    euler_all[0,:]      = quad.euler
    sDes_traj_all[0,:]  = traj.sDes
    sDes_calc_all[0,:]  = ctrl.sDesCalc
    w_cmd_all[0,:]      = ctrl.w_cmd
    wMotor_all[0,:]     = quad.wMotor
    thr_all[0,:]        = quad.thr
    tor_all[0,:]        = quad.tor
    #learning items
    falaError_all[0,:]  = fala.error_accumulated

    # Run Simulation
    # ---------------------------
    t = Ti
    i = 1
    while round(t,3) < Tf:
        
        t = quad_sim(t, Ts, quad, ctrl, wind, traj, fala, obsPF)
        
        
        # TRAVIS WILL RUN collect() here
        
        # print("{:.3f}".format(t))
        t_all[i]             = t
        s_all[i,:]           = quad.state
        pos_all[i,:]         = quad.pos
        vel_all[i,:]         = quad.vel
        quat_all[i,:]        = quad.quat
        omega_all[i,:]       = quad.omega
        euler_all[i,:]       = quad.euler
        sDes_traj_all[i,:]   = traj.sDes
        sDes_calc_all[i,:]   = ctrl.sDesCalc
        w_cmd_all[i,:]       = ctrl.w_cmd
        wMotor_all[i,:]      = quad.wMotor
        thr_all[i,:]         = quad.thr
        tor_all[i,:]         = quad.tor
        # learning items
        falaError_all[i,:]  = fala.error_accumulated
        
        i += 1
    
    end_time = time.time()
    print("Simulated {:.2f}s in {:.6f}s.".format(t, end_time - start_time))
    #print(fala.Qtable)
    

    # View Results
    # ---------------------------

    #utils.fullprint(sDes_traj_all[:,3:6])
        
    # save data
    if ifsave:
        np.savetxt("Data/Qtable.csv", fala.Qtable, delimiter=",",header=" ")
        np.savetxt("Data/errors.csv", falaError_all, delimiter=",",header=" ")
        # plt.plot(np.array(falaError_all))
        # plt.show()
        # data_all=np.hstack((np.array(t_all,ndmin=2).transpose(),pos_all,vel_all, quat_all, omega_all, euler_all, w_cmd_all, wMotor_all, thr_all, tor_all, sDes_traj_all[:,0:3], falaError_all))
        # data_all_labels='t_all,\
        #                  pos_all_x, pos_all_y,pos_all_z,\
        #                      vel_all_x,vel_all_y,vel_all_z, \
        #                          quat_all,quat_all,quat_all,quat_all,\
        #                              omega_all_p,omega_all_q,omega_all_r,\
        #                                  euler_all_phi,euler_all_theta,euler_all_psi,\
        #                                      w_cmd_all,w_cmd_all,w_cmd_all,w_cmd_all,\
        #                                          wMotor_all,wMotor_all,wMotor_all,wMotor_all,\
        #                                              thr_all,thr_all,thr_all,thr_all,\
        #                                                  tor_all,tor_all,tor_all,tor_all,\
        #                                                      sDes_traj_all_x,sDes_traj_all_y,sDes_traj_all_z,\
        #                                                          falaError'
        # np.savetxt("Data/data_all.csv", data_all, delimiter=",", header=data_all_labels)
    
    utils.makeFigures(quad.params, t_all, pos_all, vel_all, quat_all, omega_all, euler_all, w_cmd_all, wMotor_all, thr_all, tor_all, sDes_traj_all, sDes_calc_all)
    ani = utils.sameAxisAnimation(t_all, traj.wps, pos_all, quat_all, sDes_traj_all, Ts, quad.params, traj.xyzType, traj.yawType, ifsave)
    plt.show()
    #ani2 = utils.sameAxisAnimation(t_all, traj.wps, pos_all, quat_all, sDes_traj_all, Ts, quad.params, traj.xyzType, traj.yawType, ifsave)
    #plt.show()
    
    

if __name__ == "__main__":
    if (config.orient == "NED" or config.orient == "ENU"):
        main()
        # cProfile.run('main()')
    else:
        raise Exception("{} is not a valid orientation. Verify config.py file.".format(config.orient))
        
        
        
        
        
        
        
        
        
        