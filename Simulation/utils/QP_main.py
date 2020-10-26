#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 25 14:38:09 2020

@author: tjards
"""
import QP
import numpy as np


#%% Setup
nTrials = 20
ro      = 0.1   # obstacle safety buffer
r       = 0.1   # vehicle safety buffer
rb      = 0.3   # addition buffer
rb = np.minimum(1-(ro+r),rb)    # total buffer must be < 1

# initial positions
xv = np.array([-3.7, -4.7])   # vehicle position
xo = np.array([-1, -1.5])     # obstacle position
xt = np.array([0,0])          # target position 

#%% Initialize 

xp, d = QP.computeXp(xv,xo,r,ro,rb) # compute constraint centerpoint 
A = np.zeros([1,2])                 # inequalities
A[0,:] = xo - xp
b = np.dot(xp,(xo-xp))
ux = xt                             # target setpoint
cx = xt                             # target setpoint 
myData_QP = np.zeros([nTrials, 13]) # data store

# move the vehicles and/or obstacle around a bit
for i in range(0,nTrials):

    # pseudo-random movement 
    xv[0] = xv[0] + 0.1*i*0.2
    xv[1] = xv[1] + 0.3
    if i > 8:    
        xv[1] = xv[1] - 0.1
     
    # Update constraints     
    xp, d = QP.computeXp(xv,xo,r,ro,rb) 
    A[0,:] = xo - xp
    b = np.dot(xp,(xo-xp))
    
    # find new optimal point 
    cx = QP.runOpt(xv, xt, xp, A, b)
    ux = cx

    # update data store
    myData_QP[i,0:2] = np.array(xv)
    myData_QP[i,2:4] = np.array(xo)
    myData_QP[i,4:6] = np.array(xt)
    myData_QP[i,6:8] = np.array(A)
    myData_QP[i,8:9] = np.array(b)
    myData_QP[i,9:11] = np.array(ux['x'][:])    # unconstrained case (delete later to save time)
    myData_QP[i,11:13] = np.array(cx['x'][:])   # constrained case
    
    # we would feed these new points into here
    # ---> feed cx here
 
QP.myAnimation(myData_QP)