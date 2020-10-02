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
        
        #methods
        
        
        #generate table of options (states, actions)
        self.OptionsTable[:,:] = np.linspace(optionsInterval[0],optionsInterval[1],num=nOptions,axis=0)
        
        print('FALA Object created')
        print('FALA Object has ',nParams, ' parameters, each with ',nOptions,' options')
        #print('Options Table:',self.OptionsTable)
        
        self.selPars=1*np.ones((14,),dtype=int) #default to one
        
        
    def getParams(self,t):
    
        #dev - manually force tuner values
        if 10>t>5: #shut all off for 5 seconds
            self.selPars=0.01*np.ones((14,),dtype=int) 
        if t>10: # turn thenm back on after 10
            self.selPars=1*np.ones((14,),dtype=int)
        
        
        
       
        
        