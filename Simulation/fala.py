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
        
        #attributes
        self.nParams = nParams
        self.nOptions = nOptions
        self.OptionsTable = np.zeros((nParams,nOptions))
        
        #methods
        
        
        #generate table of options (states, actions)
        self.OptionsTable[:,:] = np.linspace(optionsInterval[0],optionsInterval[1],num=nOptions,axis=0)
        
        print('FALA Object created')
        print('Options Table:',self.OptionsTable)
        