#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 25 14:38:09 2020

@author: tjards
"""
import QP
import numpy as np


#%% initial Setup
# ---------------
nTrials = 20
ro      = 0.1   # obstacle safety buffer
r       = 0.1   # vehicle safety buffer
rb      = 0.3   # addition buffer
rb = np.minimum(1-(ro+r),rb)    # total buffer must be < 1
# xv = np.array([-3.7, -4.7])   # vehicle position
# xo = np.array([-1, -1.5])     # obstacle position
# xt = np.array([0,0])          # target position 
xv = np.array([-3.7, -4.7])   # vehicle position
xo = np.array([-1, -1.5])     # obstacle position
xt = np.array([0, 0])          # target position 
nStates=len(xv)

#%% update the constraints
# ------------------------
xp, d = QP.computeXp(xv,xo,r,ro,rb) # compute constraint centerpoint 
#A = np.zeros([1,2])                 # inequalities
A = np.zeros([1,nStates])                 # inequalities
A[0,:] = xo - xp
b = np.dot(xp,(xo-xp))
ux = xt                             # target setpoint
cx = xt                             # target setpoint 
nQP = nStates*6 + 1
myData_QP = np.zeros([nTrials, nQP]) # data store

# move the vehicles and/or obstacle around a bit
# ----------------------------------------------
for i in range(0,nTrials):

    # pseudo-random movement
    # ----------------------
    xv[0] = xv[0] + 0.1*i*0.2
    xv[1] = xv[1] + 0.3
    if i > 8:    
        xv[1] = xv[1] - 0.1
     
    # Update constraints    
    # ------------------
    xp, d = QP.computeXp(xv,xo,r,ro,rb) 
    A[0,:] = xo - xp
    b = np.dot(xp,(xo-xp))
    
    # find new optimal point 
    # ----------------------
    cx = QP.runOpt(xv, xt, xp, A, b)
    ux = cx

    # update data store
    # -----------------
    myData_QP[i,0:nStates] = np.array(xv)
    myData_QP[i,nStates:2*nStates] = np.array(xo)
    myData_QP[i,2*nStates:3*nStates] = np.array(xt)
    myData_QP[i,3*nStates:4*nStates] = np.array(A)
    myData_QP[i,4*nStates:4*nStates+1] = np.array(b)
    myData_QP[i,4*nStates+1:5*nStates+1] = np.array(ux['x'][:])    # unconstrained case (delete later to save time)
    myData_QP[i,5*nStates+1:6*nStates+1] = np.array(cx['x'][:])   # constrained case
    
    # we would feed these new points into here
    # ---> feed cx here
 
QP.myAnimation(myData_QP,nStates)