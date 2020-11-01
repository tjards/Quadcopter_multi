# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!



"""

import numpy as np
from numpy import pi
import random 
#import config

deg2rad = pi/180.0

wpType = 0 #0 = fixed, 1 = random


def makeWaypoints():
    t_ini = 0
    wp_ini = np.array([0, 0, 0])
    yaw_ini = 0
    v_average = 1.6

    if wpType == 0:

        t = np.array([3, 6, 9, 12, 15, 17, 20, 23])
        #t = np.array([5, 10, 15, 20, 25, 30, 35])
        wp = np.array([np.multiply(random.uniform(-2, 2),[2, 2, 1]),
                        np.multiply(random.uniform(-2, 2),[-2, 3, -3]),
                        np.multiply(random.uniform(-2, 2),[-2, -1, -3]),
                        np.multiply(random.uniform(-2, 2),[3, -2, 1]),
                        np.multiply(random.uniform(-2, 2),[0, 0, 0]),
                        np.multiply(random.uniform(-2, 2),[1.5, 2.5, -2.3]),
                        np.multiply(random.uniform(-2, 2),[-2, 3, -1]),
                        np.multiply(random.uniform(-2, 2),[2, 2, 1])])
    
    if wpType == 1:
    
        nTrials=10
        trialLen=3
        t = np.arange(trialLen,nTrials*trialLen+trialLen,trialLen)
        wp = np.random.randint(-2,2, size=(t.shape[0],3))
    
        
    #yaw = np.array([20, -90, 120, 45, 0])
    yaw = np.random.randint(-45,45, size=(t.shape[0]))

    t = np.hstack((t_ini, t)).astype(float)
    wp = np.vstack((wp_ini, wp)).astype(float)
    yaw = np.hstack((yaw_ini, yaw)).astype(float)*deg2rad

    return t, wp, yaw, v_average
