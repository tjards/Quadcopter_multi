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
from utils.animation import sameAxisAnimation as sameAxisAnimation
from utils.animation2 import sameAxisAnimation2 as sameAxisAnimation2



# this is called by each vehicle, so no need to send in  two vehicle data
def quad_sim(t, Ts, quad, ctrl, wind, traj, fala, obsPF, config):
    
    # Dynamics (using last timestep's commands)
    # ---------------------------
    quad.update(t, Ts, ctrl.w_cmd, wind, config)
    t += Ts

    # Learn from the trial 
    # ---------------------
    if fala != 0:
        fala.learn(quad,traj,ctrl,Ts,t)
    
    # Trajectory for Desired States (for next iteration)
    # -------------------------------------------------
    sDes = traj.desiredState(t, Ts, quad)

    # Use PIC shifting to move target
    # -------------------------------
    if config.PIC:
        xv = np.reshape(quad.state[0:3], (1,3))
        xt = np.reshape(traj.sDes[0:3], (1,3))
        cx = QP.moveTarget(quad.state, obsPF.Po, xv, xt, 0.1, 0.1, 0.5)
        traj.sDes[0:3] = np.array(cx['x'][:]) 

    # Avoid obstacles using potential fields 
    # --------------------------------------       
    if config.PF:
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
    wind = Wind('None', 2.0, 90, -15)
    
    # Initialize list for objects
    # ---------------------------
    numTimeStep = int(config.Tf/config.Ts+1)
    quadList = []
    trajList = []
    ctrlList = []
    sDesList = []
    obsPFList = []
    myDataList = []   
    
    for objectIndex in range(0,config.nVeh):
        
        quadList.append(Quadcopter(config))
        trajList.append(Trajectory(quadList[objectIndex], config.ctrlType, config.trajSelect, config))
        ctrlList.append(Control(quadList[objectIndex], trajList[objectIndex].yawType))
        sDesList.append(trajList[objectIndex].desiredState(0, config.Ts, quadList[objectIndex]) )
        obsPFList.append(pf(trajList[objectIndex], np.vstack((config.o1,config.o2,config.o3,quadList[objectIndex-1].state[0:3])).transpose() , gamma=1, eta=0.2, obsRad=0.5))
        # generate first command
        ctrlList[objectIndex].controller(trajList[objectIndex], quadList[objectIndex], sDesList[objectIndex], config)

    # Create learning object
    # ---------------------------
    fala = falaObj(config)
    
    # Initialize Result Matrixes
    # ---------------------------
    for objectIndex in range(0,config.nVeh):
        # initialize result matrices
        myDataList.append(collect.quadata(quadList[objectIndex], trajList[objectIndex], ctrlList[objectIndex], fala, config.Ti, numTimeStep))
    
    # Run Simulation
    # ---------------------------
    t = config.Ti*np.ones(config.nVeh) 
     
    i = 1
    while round(t[0],3) < config.Tf:
        
        # Update the obstacle positions
        # -----------------------------
        if t[0] > 0.1:
            o1 = config.o1  # np.array([-2.1, 0, -3],)           # obstacle 1 (x,y,z)
            o2 = config.o2  # np.array([2, -1.2, 0.9])           # obstacle 2 (x,y,z)
            o3 = config.o3  # np.array([0, 2.5, -2.5])           # obstacle 2 (x,y,z)
            obsPFList[0].Po = np.vstack((o1,o2,o3,quadList[1].state[0:3])).transpose() 
            obsPFList[1].Po = np.vstack((o1,o2,o3,quadList[0].state[0:3])).transpose()
 
        # Integrate through the dynamics
        # ------------------------------
        for objectIndex in range(0,config.nVeh):
            
            t[objectIndex] = quad_sim(t[objectIndex], config.Ts, quadList[objectIndex], ctrlList[objectIndex], wind, trajList[objectIndex], fala, obsPFList[objectIndex], config)
            myDataList[objectIndex].collect(t[objectIndex], quadList[objectIndex], trajList[objectIndex], ctrlList[objectIndex], fala, i)

        
        i += 1
    
    end_time = time.time()
    print("Simulated {:.2f}s in {:.6f}s.".format(t[0], end_time - start_time))
    

    # View Results
    # ---------------------------    
    # save data
    if config.ifsave:
        # text
        #np.savetxt("Data/Qtable.csv", fala.Qtable, delimiter=",",header=" ")
        #np.savetxt("Data/errors.csv", myData.falaError_all, delimiter=",",header=" ")
        #plots
        #utils.makeFigures(quad.params, myData)
        utils.makeFigures(quadList[0].params, myDataList[0])
        #sameAxisAnimation2(config, myData, traj, quad.params, obsPF, myColour = 'blue')
        #sameAxisAnimation2(config, myData2, traj2, quad2.params, obsPF, myColour = 'red')
        #ani = sameAxisAnimation2(config, myData, traj, quad.params, myData2, traj2, quad2.params, obsPF, 'blue', 'green')
        if config.nVeh == 1:
            ani = sameAxisAnimation(config, myDataList[0], trajList[0], quadList[0].params, obsPFList[0], myColour = 'blue')      
        if config.nVeh == 2:
            ani = sameAxisAnimation2(config, myDataList[0], trajList[0], quadList[0].params, myDataList[1], trajList[1], quadList[1].params, obsPFList[0], 'blue', 'green')
        plt.show()
    
        if config.doLearn:
            np.savetxt("Data/Qtable.csv", fala.Qtable, delimiter=",",header=" ")
            #np.savetxt("Data/errors.csv", myData.falaError_all, delimiter=",",header=" ")
            np.savetxt("Data/errors.csv", myDataList[0].falaError_all, delimiter=",",header=" ")
            
        # save energy draw    
        torque0 = myDataList[0].tor_all 
        eDraw0 = np.cumsum(np.cumsum(torque0, axis=0), axis = 1)[:,3]
        np.save('Data/energyDepletion_veh0',eDraw0)
        torque1 = myDataList[1].tor_all 
        eDraw1 = np.cumsum(np.cumsum(torque1, axis=0), axis = 1)[:,3]
        np.save('Data/energyDepletion_veh1',eDraw1)
        
        
        if config.ifsavedata:
            #save states
            x    = myDataList[0].pos_all[:,0]
            y    = myDataList[0].pos_all[:,1]
            z    = myDataList[0].pos_all[:,2]
            x_sp  = myDataList[0].sDes_calc_all[:,0]
            y_sp  = myDataList[0].sDes_calc_all[:,1]
            z_sp  = myDataList[0].sDes_calc_all[:,2]
            states0 = np.vstack([x,y,z,x_sp,y_sp,z_sp]).transpose()
            np.save('Data/states0',states0)
            x    = myDataList[1].pos_all[:,0]
            y    = myDataList[1].pos_all[:,1]
            z    = myDataList[1].pos_all[:,2]
            x_sp  = myDataList[1].sDes_calc_all[:,0]
            y_sp  = myDataList[1].sDes_calc_all[:,1]
            z_sp  = myDataList[1].sDes_calc_all[:,2]
            states1 = np.vstack([x,y,z,x_sp,y_sp,z_sp]).transpose()
            np.save('Data/states1',states1)
        
        
    #return ani



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
        