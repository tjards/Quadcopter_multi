#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This program implements finite action-set learning automata 


Created on Mon Jul  6 20:54:06 2020

@author: tjards


Start by generating random tuning parameters, feeding them into controller



"""

import numpy as np


class falaObj:
    
    def __init__(self,nParams,nOptions,optionsInterval):
        
        #attributes
        self.nParams = nParams
        self.nOptions = nOptions
        self.OptionsTable = np.zeros((nParams,nOptions))
        self.error_pos    = np.zeros(3)
        self.error_vel   = np.zeros(3)
    
        #generate table of options (states, actions)
        self.OptionsTable[:,:] = np.linspace(optionsInterval[0],optionsInterval[1],num=nOptions,axis=0)
        
        print('FALA Object created')
        print('FALA Object has ',nParams, ' parameters, each with ',nOptions,' options')
        #print('Options Table:',self.OptionsTable)
        
        self.selPars=1*np.ones((14,),dtype=int) #default to one
        
        
    # Produce a new set of parameters (should be drawn from the distribution)
    # --------------------------------
    def getParams(self,t):
    
        #dev - manually force tuner values
        self.selPars=1*np.ones((14,),dtype=int)
        
        return self.selPars
    
    # Compute the error signal
    # ------------------------
    def computeError(self,quad,traj):
        
        self.error_pos[0:3] = traj.sDes[0:3]-quad.pos[0:3]
        self.error_vel[0:3] = traj.sDes[3:6]-quad.vel[0:3]
        
    
    
       
        
        