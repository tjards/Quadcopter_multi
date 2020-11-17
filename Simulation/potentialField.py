#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 18 13:40:20 2020

@author: tjards

This module implements potential fields for obstacle avoidance
based on the technique @ ref: https://arxiv.org/pdf/1704.04672.pdf


"""

import numpy as np

class potentialField:

    def __init__(self, traj, Po, gamma=1, eta=1 ,obsRad=1, obsRange = 10):
    
        self.Po = Po
        self.gamma = gamma
        self.eta = eta
        self.obsRad = obsRad
        self.default_ctrlType = traj.ctrlType
        self.obsRange = obsRange


    # def overlap(self, type, flag):
    #     print("collision detected: error (%s)" % (type))
        
    # self.alert_overLap = np.seterrcall(self.overlap)
    # self.alert = np.seterr(divide='call')
    
    
    def computeAttract(self):
        
        #   pd is the vehicle position (vector)
        #   pt is the target position (vector)
        #   gamma is the scaling factor
        
        self.va = -np.multiply(self.gamma,(self.pd-self.pt))
        
        #return va
        
    
    # execute this only if with obstacle avoidance region 
    def computeRepulse(self):
        
        # pd is the vehicle position (vector)  
        # Po is the obstacle position(s) (array of vectors):
        #   xo_1 xo_2 ... xo_n
        #   yo_1 yo_2 ... yo_n
        #   zo_1 zo_2 ... zo_n
        #   example: Po = np.array([[x1,x2,x3],[y1,y2,y3],[z1,z2,z3]])
        # eta is the scaling factor
       
       self.nObs = len(self.Po[1])                # number of obstacles
       self.Pdo = np.zeros((3,self.nObs))         # initialize array of differences
       self.Pdo_mags = np.zeros((1,self.nObs))    # intialize magnitudes
       self.vr = np.zeros((3,1))
       self.flag = 0                         # count the obstacles in range
       
       for i in range(0,self.nObs): 
           
           self.Pdo[:,[i]] = self.pd - self.Po[:,[i]]
           self.Pdo_mags[0,i] = np.linalg.norm(self.Pdo[:,i])
           
           # only count the obstacle if inside the radius
           #if 0 < self.Pdo_mags[0,i] < self.obsRad:            
           if 0 < self.Pdo_mags[0,i] < self.obsRange:            
               self.vr += self.eta*np.multiply(self.Pdo[:,[i]],np.divide(1,np.power(self.Pdo_mags[0,i],4)))
               self.flag += 1
    
       #return Pdo, Pdo_mags, vr, flag
    
    def updateTraj(self, pd, pt, traj):
        
        
        self.pd = np.array(pd,ndmin=2).transpose()
        self.pt = np.array(pt,ndmin=2).transpose()
           
        self.computeAttract()
        #va = computeAttract(pd,pt,gamma)
        self.computeRepulse()
        #Pdo, Pdo_mags, vr, flag = computeRepulse(pd,Po,eta,obsRad)
        
        self.vd = self.va - self.vr
              
        if self.flag > 0:
            #print('obstacle detected')
            traj.ctrlType = "xyz_vel"               # use velocity control
            traj.sDes[3:6] = np.squeeze(self.vd)
        else:
            traj.ctrlType = self.default_ctrlType   # return to user-defined control type
    
       
       
#%% Test

# pd = np.array([0,0,0],ndmin=2).transpose()
# #Po = np.array([[x1,x2,x3],[y1,y2,y3],[z1,z2,z3]])
# Po = np.array([[2,1,1],[1.2,1.1,1.1],[1.3,1,1.7]])
# pt = np.array([5,5,5],ndmin=2).transpose()

# gamma = 1
# eta = 0.2
# obsRad = 5  #only count obstacles in this radius

# vd, flag = computeDesVel(pd,pt,Po,gamma,eta,obsRad)
  