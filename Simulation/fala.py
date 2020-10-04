#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This program implements finite action-set learning automata 


Created on Mon Jul  6 20:54:06 2020

@author: tjards





"""

import numpy as np


class falaObj:
    
    def __init__(self,nParams,nOptions,optionsInterval):
        
        # dev note: right now optionsInterval only accepts one interval [a,b]
        #   later, this should be expanded to pass different intervals 
        #   for each parameter
        
        #attributes
        self.nParams = nParams                              # how many parameters are being tuned
        self.nOptions = nOptions                            # how many options for each parameter
        OptionsTable = np.zeros((nParams,nOptions))         # tables of possible values (will transpose after fill)
        OptionsTable[:,:] = np.linspace(optionsInterval[0],optionsInterval[1],num=nOptions,axis=0) # generate options
        self.OptionsTable=OptionsTable.transpose()          # because i like options (rows) x parameters (cols)
        self.QTable = np.zeros((nOptions,nParams))          # this stores the probabilities (note orientation of matrix)
        self.error_pos    = np.zeros(3)
        self.error_vel   = np.zeros(3)
    

        
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
        
    
#%% develop the learn method down here, add to class later

#items of the class that will be used (be cautious of the correct timestep!)
# (i.e. self.{below})

nParams=14
nOptions=10
optionsInterval=[0,5]
OptionsTable = np.zeros((nParams,nOptions))
OptionsTable[:,:] = np.linspace(optionsInterval[0],optionsInterval[1],num=nOptions,axis=0)
OptionsTable=OptionsTable.transpose()

#other items that need to be passed into this new method
   

#other initialization parameters



 
       
        
        