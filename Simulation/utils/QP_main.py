#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 25 14:38:09 2020

@author: tjards
"""
import QP
import numpy as np




xv = np.array([-3.7, -4.7])   # vehicle position
xo = np.array([-1, -1.5])  # obstacle position
xt = np.array([0,0])  # target position 
ro = 0.1
r = 0.1
rb = 0.3
#buffer = 0.3


xp, d = QP.computeXp(xv,xo,r,ro,rb)

# A = np.array([[ 1., 1.],
#               [-1., 2.],
#               [-1., 0.],
#               [0., -1.],
#               [0.,  1.]])

A = np.zeros([1,2])
A[0,:] = xo - xp

#b = np.array([7., 4., 0., 0., 4.])
b = np.dot(xp,(xo-xp))


QP.myAnimation(xv,xo,xt,A,b,r,ro,rb)