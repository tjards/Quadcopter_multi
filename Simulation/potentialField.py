#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 18 13:40:20 2020

@author: tjards

ref: https://arxiv.org/pdf/1704.04672.pdf


"""

import numpy as np


def overlap(type, flag):
    print("collision detected: error (%s)" % (type))
    
alert_overLap = np.seterrcall(overlap)
alert = np.seterr(divide='call')


def computeAttract(pd,pt,gamma):
    
    #   pd is the vehicle position (vector)
    #   pt is the target position (vector)
    #   gamma is the scaling factor
    
    va = -np.multiply(gamma,(pd-pt))
    
    return va
    

# execute this only if with obstacle avoidance region 
def computeRepulse(pd,Po,eta):
    
    # pd is the vehicle position (vector)  
    # po is the obstacle position(s) (array of vectors):
    #   xo_1 xo_2 ... xo_n
    #   yo_1 yo_2 ... yo_n
    #   zo_1 zo_2 ... zo_n
    # eta is the scaling factor
   
   nObs = len(Po[1])                # number of obstacles
   Pdo = np.zeros((3,nObs))         # initialize array of differences
   Pdo_mags = np.zeros((1,nObs))    # intialize magnitudes
   vr = np.zeros((3,1))
   
   for i in range(0,nObs): 
       
       Pdo[:,[i]] = pd - Po[:,[i]]
       Pdo_mags[0,i] = np.linalg.norm(Pdo[:,i])
       vr += np.multiply(Pdo[:,[i]],np.divide(1,np.power(Pdo_mags[0,i],4)))
       
   return Pdo, Pdo_mags, vr

def computeDesVel(pd,pt,Po,gamma,eta):
    
    va = computeAttract(pd,pt,gamma)
    Pdo, Pdo_mags,vr = computeRepulse(pd,Po,gamma)
    vd = va - vr
    
    return vd
    
    
       
       
#test
pd = np.array([0,0,0],ndmin=2).transpose()
Po = np.array([[2,1,1],[1.2,3.4,1.1],[1.3,1,0.8]])
pt = np.array([5,5,5],ndmin=2).transpose()

gamma = 0.5
eta = 0.2

vd = computeDesVel(pd,pt,Po,gamma,eta)
  