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
    
    # Compute the error signal (this will need to accumulate and then reset after trial)
    # ------------------------
    def computeError(self,quad,traj):
        
        self.error_pos[0:3] = traj.sDes[0:3]-quad.pos[0:3]
        self.error_vel[0:3] = traj.sDes[3:6]-quad.vel[0:3]
        






    
#%% develop the reward signal computation method here (after total trials accumulateded @ 0.005), run this

#itialize 

costMin = 1000000     # the minimum observed cost thus far (persistent valriable, start high)
costAvg = 0           # the average observed cosr thus far (persistent valriable)
costIn = 0.3          # this cost will be passed in at the end of each trial 
countSample = 0       # need to keep track of samples to compute average
reward_temp = 0       # for interim calculation
reward_b = 1          # modulation for reward signal (optional, default 1)
reward = 0            # this is the reward signal 
eps = 0.00000001      # small value to avoid dividing by zero

#def computeReward(costIn):

# update stuff
countSample += 1                                            # increment the sample
costMin=np.minimum(costMin,costIn)                          # update the minimum cost
costAvg=costAvg+np.divide((costIn-costAvg),np.maximum(countSample,eps))     # update the average

# compute reward signal 
reward_temp = np.minimum(np.maximum(0,(costAvg-costIn)/(costAvg-costMin+eps)),1)
if reward_temp == 1:
    reward = reward_temp
elif 1 > reward_temp > 0:
    reward=reward_temp*reward_b
else:
    reward=0
    
#return reward


#%% develop the learn method down here, add to class later

#items that have to be passed in (Reward signal?)

#items of the class above that will be used (be cautious of the correct timestep!)

nParams=14
nOptions=10
optionsInterval=[0,5]
optionsTable = np.zeros((1,nOptions))       #assume all same (the 1 would need to change to nParams for variable options)
optionsTable[:,:] = np.linspace(optionsInterval[0],optionsInterval[1],num=nOptions,axis=0)
optionsTable=optionsTable.transpose()
Qtable=np.zeros((nOptions,nParams)) 

#new items to add to the class

Qtable[:,:]=np.divide(1,nOptions)*np.ones((nOptions,nParams)) #set all values to have the same confidence (could use heuristics)
pVector=np.zeros((1,nParams))
pVector[0,:]=Qtable[0,:]        # initialize pVector using first row of Q table
selectedIndex=np.zeros((1,nParams),dtype=int) 
selectedVals=np.zeros((1,nParams))
selectedVals[0,:]=optionsTable[0,:]

#other initialization parameters (where to go?)


#%% update probability distribution

#initialize
learnRate = 0.1
probLimit = 1       # maximum probabillity (default 1, related to exploit vs explore)
a = 1               # weight of positive reinforcement (default one)
b = 0               # weight of negative reinforcement (default zero)

#dev: test
selectedIndex=np.array([0,1,2,3,4,5,6,7,7,7,7,7,9,8],ndmin=2)
reward=0.6

#update the distributions
#pVector=pVector+learnRate*reward*pVector
pVector=pVector+a*learnRate*reward*pVector+b*learnRate*(1-reward)*pVector

for i in range(0,nParams):
    Qtable[selectedIndex[0,i],i]=np.minimum(pVector[0,i],probLimit)
    
#normalize
for i in range(0,nParams):
    Qtable[:,i]=Qtable[:,i]/np.sum(Qtable[:,i])
 
#%% randomly select new automata

#reset inside here (after being used)
#selectedIndex=np.zeros((1,nParams),dtype=int) 
#selectedVals=OptionsTable[0,:]

#new terms
#probsC=np.zeros((nOptions,1))


for i in range(0,nParams):
    #randomly select an option according to the current distribution in the Q table
    #probsC[:,0]=np.cumsum(Qtable[:,i])  #compute cumulative sum
    selectedIndex[0,i] = (np.cumsum(Qtable[:,i]) >= np.random.random()).argmax()
    # selected value cooresponding to that index
    selectedVals[0,i] = optionsTable[selectedIndex[0,i],0]


       
        
        