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
#import config as simConfig

# my libraries
from fala import falaObj 
from potentialField import potentialField as pf
import utils.collectData as collect
import config as simConfig
import utils.QP as QP
from utils.animation2 import sameAxisAnimation2 as sameAxisAnimation2



# this is called by each vehicle, so no need to send in  two vehicle data
def quad_sim(t, Ts, quad, ctrl, wind, traj, fala, obsPF, config):
    
    # Dynamics (using last timestep's commands)
    # ---------------------------
    quad.update(t, Ts, ctrl.w_cmd, wind, config)
    t += Ts

    # Learn from the trial
    # --------------------------
    fala.learn(quad,traj,ctrl,Ts,t)
    
    # Trajectory for Desired States (for next iteration)
    # ---------------------------
    sDes = traj.desiredState(t, Ts, quad)

    # Use PIC shifting to move target
    # note: just try with first obstacle for now
    # to slow down the shifting, do it at 1/10th speed
    # -----------------------
    if config.PIC:
        xv = np.reshape(quad.state[0:3], (1,3))
        xt = np.reshape(traj.sDes[0:3], (1,3))
        cx = QP.moveTarget(quad.state, obsPF.Po, xv, xt, 0.1, 0.1, 0.3)
        traj.sDes[0:3] = np.array(cx['x'][:]) 

    # Update the trajectory for obstacles with potential fields 
    # ---------------------------       
    obsPF.updateTraj(quad.state[0:3],traj.sDes[0:3],traj)

    # Generate Commands (for next iteration)
    # ---------------------------
    ctrl.controller(traj, quad, sDes, config)

    return t
    

def main():
    start_time = time.time()

    # Simulation Setup
    # --------------------------- 
    config=simConfig.config()

    # Initialize Quadcopter, Controller, Wind, Result Matrixes
    # ---------------------------
    quad = Quadcopter(config)
    traj = Trajectory(quad, config.ctrlType, config.trajSelect)
    ctrl = Control(quad, traj.yawType)
    wind = Wind('None', 2.0, 90, -15)
    
    # second vehicle
    quad2 = Quadcopter(config)
    traj2 = Trajectory(quad2, config.ctrlType, config.trajSelect)
    ctrl2 = Control(quad2, traj2.yawType)

    # Create learning object
    # ---------------------------
    # for the controller
    fala = falaObj(nParams=14, nOptions=10, optionsInterval=[0.1,2], learnRate=0.15, trialLen=3)
    
    # Trajectory for First Desired States
    # ---------------------------
    sDes = traj.desiredState(0, config.Ts, quad) 
    sDes2 = traj2.desiredState(0, config.Ts, quad2) 

    # Create a Potential Field object
    # -------------------------------
    o1 = config.o1  # np.array([-2.1, 0, -3],)           # obstacle 1 (x,y,z)
    o2 = config.o2  # np.array([2, -1.2, 0.9])           # obstacle 2 (x,y,z)
    o3 = config.o3  # np.array([0, 2.5, -2.5])           # obstacle 2 (x,y,z)
    #Po = np.vstack((o1,o2)).transpose()     # stack obstacles
    Po = np.vstack((o1,o2,o3,quad2.state[0:3])).transpose()     # stack obstacles (include other vehicle)
    Po2 = np.vstack((o1,o2,o3,quad.state[0:3])).transpose() 
    
    obsPF = pf(traj, Po, gamma=1, eta=0.5, obsRad=1)
    obsPF2 = pf(traj2, Po2, gamma=1, eta=0.5, obsRad=1)
    
    # Generate First Commands
    # ---------------------------
    ctrl.controller(traj, quad, sDes, config)
    ctrl2.controller(traj2, quad2, sDes2, config)
    
    # Initialize Result Matrixes
    # ---------------------------
    numTimeStep = int(config.Tf/config.Ts+1)
    myData = collect.quadata(quad, traj, ctrl, fala, config.Ti, numTimeStep)
    myData2 = collect.quadata(quad2, traj2, ctrl2, fala, config.Ti, numTimeStep)

    # Run Simulation
    # ---------------------------
    t = config.Ti
    i = 1
    while round(t,3) < config.Tf:
        
        # Update the obstacle positions
        # -----------------------------
        if t > 0.1:
            o1 = config.o1  # np.array([-2.1, 0, -3],)           # obstacle 1 (x,y,z)
            o2 = config.o2  # np.array([2, -1.2, 0.9])           # obstacle 2 (x,y,z)
            o3 = config.o3  # np.array([0, 2.5, -2.5])           # obstacle 2 (x,y,z)
            obsPF.Po = np.vstack((o1,o2,o3,quad2.state[0:3])).transpose()     # stack obstacles (include other vehicle)
            obsPF2.Po = np.vstack((o1,o2,o3,quad.state[0:3])).transpose() 
        
        # Integrate through the dynamics
        # ------------------------------
        config.PIC = 0      # turn off PIC
        t = quad_sim(t, config.Ts, quad, ctrl, wind, traj, fala, obsPF, config)
        config.PIC = 1      # turn on PIC
        t2 = quad_sim(t, config.Ts, quad2, ctrl2, wind, traj2, fala, obsPF2, config)
        
        # Collect data from this timestep
        # -------------------------------
        myData.collect(t, quad, traj, ctrl, fala, i)
        myData2.collect(t2, quad2, traj2, ctrl2, fala, i)
        
        i += 1
    
    end_time = time.time()
    print("Simulated {:.2f}s in {:.6f}s.".format(t, end_time - start_time))
    

    # View Results
    # ---------------------------    
    # save data
    if config.ifsave:
        # text
        np.savetxt("Data/Qtable.csv", fala.Qtable, delimiter=",",header=" ")
        np.savetxt("Data/errors.csv", myData.falaError_all, delimiter=",",header=" ")
        #plots
        #utils.makeFigures(quad.params, myData)
        #sameAxisAnimation2(config, myData, traj, quad.params, obsPF, myColour = 'blue')
        #sameAxisAnimation2(config, myData2, traj2, quad2.params, obsPF, myColour = 'red')
        sameAxisAnimation2(config, myData, traj, quad.params, myData2, traj2, quad2.params, obsPF, 'blue', 'green')
        plt.show()



#%% COMMAND LINE/TERMINAL
# just type:  > python run_3D_simulation.py

if __name__ == "__main__":
    print('running...')
    main()
    

   
        
        
 #%% LEGACY Code
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
        #np.savetxt("Data/errors.csv", falaError_all, delimiter=",",header=" ")



    #utils.makeFigures(quad.params, t_all, pos_all, vel_all, quat_all, omega_all, euler_all, w_cmd_all, wMotor_all, thr_all, tor_all, sDes_traj_all, sDes_calc_all)
    #ani = utils.sameAxisAnimation(t_all, traj.wps, pos_all, quat_all, sDes_traj_all, Ts, quad.params, traj.xyzType, traj.yawType, ifsave, obsPF.Po, obsPF.obsRad)
    #utils.makeFigures(quad.params, myData.t_all, myData.pos_all, myData.vel_all, myData.quat_all, myData.omega_all, myData.euler_all, myData.w_cmd_all, myData.wMotor_all, myData.thr_all, myData.tor_all, myData.sDes_traj_all, myData.sDes_calc_all)       
    #utils.sameAxisAnimation(config, myData.t_all, traj.wps, myData.pos_all, myData.quat_all, myData.sDes_traj_all, config.Ts, quad.params, traj.xyzType, traj.yawType, config.ifsave, obsPF.Po, obsPF.obsRad)
        
    #ani2 = utils.sameAxisAnimation(t_all, traj.wps, pos_all, quat_all, sDes_traj_all, Ts, quad.params, traj.xyzType, traj.yawType, ifsave)
    #plt.show()        
        
    #utils.fullprint(myData.sDes_traj_all[:,3:6])
        