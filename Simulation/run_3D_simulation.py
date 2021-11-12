# -*- coding: utf-8 -*-
"""
The project implements Reinforcement Learning
(specifically, Finite Action-set Learning Automata) 
to tune the PID controller gains of a simulated Quadcopter
+ multi-vehicle 
+ obstacle avoidance using potential fields
+ plus an improvement on potential fields using shifting planar inequalities

Full documentation here:
    https://github.com/tjards/Quadcopter_multi


editing author: P. Travis Jardine, PhD
email: travis.jardine@gmail.com 

The vehicle dynamics, controllers, and other files have been modified from the
original implementation of:

author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!

"""

# Import stuff
# ------------

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
import utils.shiftingPIC as QP
from utils.animation import sameAxisAnimation as sameAxisAnimation
#from utils.animation2 import sameAxisAnimation2 as sameAxisAnimation2
from utils.animation_n import sameAxisAnimationN as sameAxisAnimationN


# This function integrates the control inputs through the dynamics
# ----------------------------------------------------------------
def quad_sim(t, Ts, quad, ctrl, wind, traj, fala, obsPF, config):
    
    
    # Hard code the learned params
    # ----------------------------
    # these are bad ones:
    # #selPars = np.array([1,6,3,6,0,6,0,2,0,1,5,4,0,2])
    # selPars = np.array([1,1,1,1,1,1,1,1,1,1,1,1,1,1])

    # for i in range(0,config.nParams):
    #     fala.selectedVals[0,i] = fala.optionsTable[selPars[i],i] 
    # ctrl.tune(fala.selectedVals.transpose().ravel(), [1, 1, 1, 1])
    
    
    # Dynamics (using last timestep's commands)
    # ---------------------------
    quad.update(t, Ts, ctrl.w_cmd, wind, config)
    t += Ts

    # Learn from the trial (if selected) 
    # ---------------------------------
    if fala.doLearn != 0:
        fala.learn(quad,traj,ctrl,Ts,t)
    
    # Trajectory for Desired States (for next iteration)
    # -------------------------------------------------
    sDes = traj.desiredState(t, Ts, quad)

    # Use PIC shifting to move target (if enabled)
    # ---------------------------------------------
    if config.PIC:
        xv = np.reshape(quad.state[0:3], (1,3))
        xt = np.reshape(traj.sDes[0:3], (1,3))
        cx = QP.moveTarget(quad.state, obsPF.Po, xv, xt, 0.1, 0.1, config.obsRad)
        traj.sDes[0:3] = np.array(cx['x'][:]) 

    # Avoid obstacles using potential fields (if enabled)
    # ---------------------------------------------------       
    if config.PF:
        obsPF.updateTraj(quad.state[0:3],traj.sDes[0:3],traj)

    # Generate Commands (for next iteration)
    # --------------------------------------
    ctrl.controller(traj, quad, sDes, config)

    return t
    
# This is the main function
# --------------------------
def main():
    
    start_time = time.time()

    # Import the simulation setup
    # --------------------------- 
    config=simConfig.config()

    # Initialize wind (untested) 
    # --------------------------
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
    
    # For the number of vehicles
    # --------------------------
    for objectIndex in range(0,config.nVeh):
        
        quadList.append(Quadcopter(config))
        trajList.append(Trajectory(quadList[objectIndex], config.ctrlType, config.trajSelect, config))
        ctrlList.append(Control(quadList[objectIndex], trajList[objectIndex].yawType))
        sDesList.append(trajList[objectIndex].desiredState(0, config.Ts, quadList[objectIndex]) )
        obsPFList.append(pf(trajList[objectIndex], np.vstack((config.o1,config.o2,config.o3,quadList[objectIndex-1].state[0:3])).transpose() , gamma=config.gamma, eta=config.eta, obsRad=config.obsRad, obsRange = config.obsRange))
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
            
            # recall these, as they could move with time
            o1 = config.o1  # np.array([-2.1, 0, -3],)           # obstacle 1 (x,y,z)
            o2 = config.o2  # np.array([2, -1.2, 0.9])           # obstacle 2 (x,y,z)
            o3 = config.o3  # np.array([0, 2.5, -2.5])           # obstacle 2 (x,y,z)
            
            # if just one vehicle
            if config.nVeh == 1:
                obsPFList[0].Po = np.vstack((o1,o2,o3)).transpose() 
             
            # if two vehicles, make sure to avoid other dudeBot   
            #if config.nVeh == 2:
            #    obsPFList[0].Po = np.vstack((o1,o2,o3,quadList[1].state[0:3])).transpose() 
            #    obsPFList[1].Po = np.vstack((o1,o2,o3,quadList[0].state[0:3])).transpose()
            
            # if more than one vehicle, make sure to avoid other dudeBot   
            for io in range(0,config.nVeh):
                for jo in range(0,config.nVeh-1):
                    obsPFList[io].Po = np.vstack((o1,o2,o3,quadList[io-jo].state[0:3])).transpose() 
  
                
 
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
    if config.ifsave:

        # make figures 
        utils.makeFigures(quadList[0].params, myDataList[0])
        
        # make animation (this should be generalized later)
        if config.ifsaveplots:
            if config.nVeh == 1:
                ani = sameAxisAnimation(config, myDataList[0], trajList[0], quadList[0].params, obsPFList[0], myColour = 'blue')      
            #if config.nVeh == 2:
            #    ani = sameAxisAnimation2(config, myDataList[0], trajList[0], quadList[0].params, myDataList[1], trajList[1], quadList[1].params, obsPFList[0], 'blue', 'green')
            if config.nVeh > 1:
                ani = sameAxisAnimationN(config, myDataList, trajList, quadList, obsPFList[0], 'blue')
            
           
        plt.show()
        
        # dump the learned parameters 
        if config.doLearn:
            np.savetxt("Data/Qtable.csv", fala.Qtable, delimiter=",",header=" ")
            np.savetxt("Data/errors.csv", myDataList[0].falaError_all, delimiter=",",header=" ")
            
        # save energy draw    
        torque0 = myDataList[0].tor_all 
        eDraw0 = np.cumsum(np.cumsum(torque0, axis=0), axis = 1)[:,3]
        np.save('Data/energyDepletion_veh0',eDraw0)
        if config.nVeh == 2:
            torque1 = myDataList[1].tor_all 
            eDraw1 = np.cumsum(np.cumsum(torque1, axis=0), axis = 1)[:,3]
            np.save('Data/energyDepletion_veh1',eDraw1)
        
        # save the data       
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
            if config.nVeh == 2:
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
    

   