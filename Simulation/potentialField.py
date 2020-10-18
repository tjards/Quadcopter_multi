#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 18 13:40:20 2020

@author: tjards

ref: https://arxiv.org/pdf/1704.04672.pdf


"""

def computeAttract(gamma,pd,pt):
    
    #   pd is the vehicle position (vector)
    #   pt is the target position (vector)
    #   gamma is the scaling factor
    
    va = -gamma(pd-pt)
    
def computeRepulse(pd,Po):
    
    # pd is the vehicle position (vector)  
    # po is the obstacle position(s) (array of vectors)
    #   xo_1 xo_2 ... xo_n
    #   yo_1 yo_2 ... yo_n
    #   zo_1 zo_2 ... zo_n
    