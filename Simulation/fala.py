#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This program implements finite action-set learning automata 

author: P. Travis Jardine, PhD
email: travis.jardine@gmail.com 

This technique is described in detail here:
"Leveraging Data Engineering to Improve Unmanned Aerial Vehicle Control Design"
https://ieeexplore.ieee.org/document/9130726

and has been adopted for application on Quadcopter_SimCom by John Bass, 
john.bobzwik@gmail.com

"""

import numpy as np

# Learning setup
#---------------

nParams=14                  # number of controller parameters that need tuning
nOptions=10                 # number of options per controller (assume all =)
optionsInterval=[0.1,2]     # range for parameters
learnRate=0.15              # learning rate (<1)
trialLen=3                  # lenght of each trial 
doLearn = 0                 # 1 = yes, 0 = no

class falaObj:
    
    def __init__(self):
        
        # dev note: right now optionsInterval only accepts one interval [a,b]
        #   later, this should be expanded to pass different intervals 
        #   for each parameter
        
        #attributes
        self.nParams = nParams                              # how many parameters are being tuned
        self.nOptions = nOptions                            # how many options for each parameter
        optionsTable = np.zeros((self.nParams,self.nOptions))         # tables of possible values (will transpose after fill)
        optionsTable[:,:] = np.linspace(optionsInterval[0],optionsInterval[1],num=self.nOptions,axis=0) # generate options
        self.optionsTable=optionsTable.transpose()          # because i like options (rows) x parameters (cols)
        self.Qtable = np.zeros((self.nOptions,self.nParams))          # this stores the probabilities (note orientation of matrix)
        self.Qtable[:,:]=np.divide(1,self.nOptions)*np.ones((self.nOptions,self.nParams)) #set all values to have the same confidence (could use heuristics)
        self.error_pos    = np.zeros(3)
        self.error_vel   = np.zeros(3)
        self.trialLen = trialLen
        self.trialCounter = 0 
        self.error_accumulated = 0.0
        self.doLearn = doLearn
        
        # initialize attributes used to compute reward
        self.costMin = 1000000     # the minimum observed cost thus far (persistent valriable, start high)
        self.costAvg = 0           # the average observed cosr thus far (persistent valriable)
        #self.costIn = 0          # this cost will be passed in at the end of each trial 
        self.countSample = 0       # need to keep track of samples to compute average
        #self.reward_temp = 0       # for interim calculation
        self.reward_b = 1          # modulation for reward signal (optional, default 1)
        self.reward = 0.1            # this is the reward signal 
        self.eps = 0.00000001      # small value to avoid dividing by zero
        
        #initialize attributes used to update the Qtable
        self.learnRate = learnRate
        self.probLimit = 1       # maximum probabillity (default 1, related to exploit vs explore)
        self.a = 1               # weight of positive reinforcement (default one)
        self.b = 0               # weight of negative reinforcement (default zero)
        self.pVector=np.zeros((1,self.nParams))
        self.pVector[0,:]=self.Qtable[0,:]        # initialize pVector using first row of Q table
        self.selectedIndex=np.zeros((1,self.nParams),dtype=int) 
        self.selectedVals=np.zeros((1,self.nParams))
        self.selectedVals[0,:]=self.optionsTable[0,:]
        
    
        print('FALA Object created')
        print('FALA Object has ',nParams, ' parameters, each with ',nOptions,' options')
        #print('Options Table:',self.OptionsTable)
        
        
        #self.selPars=1*np.ones((14,),dtype=int) #default to one #LEGACY notation
        
    
    # Compute the error signal (this will need to accumulate and then reset after trial)
    # ------------------------
    def computeError(self,quad,traj):
        
        self.error_pos[0:3] = np.sqrt(np.square(traj.sDes[0:3]-quad.pos[0:3]))
        self.error_vel[0:3] = np.sqrt(np.square(traj.sDes[3:6]-quad.vel[0:3]))
        
        # this is an unsophisticated error calc (fix later)
        self.error=np.sum(self.error_pos)
        self.error_accumulated += self.error #this resets after trials
        

    # Compute reward signal (after total trials accumulateded @ 0.005, remember to reset whatever trial clock)
    # --------------------
    def computeReward(self,costIn):
    
        # update stuff
        self.countSample += 1                                                       # increment the sample
        self.costMin=np.minimum(self.costMin,costIn)                           # update the minimum cost
        self.costAvg=self.costAvg+np.divide((costIn-self.costAvg),np.maximum(self.countSample,self.eps))     # update the average
        
        # compute reward signal 
        reward_temp = np.minimum(np.maximum(0,(self.costAvg-costIn)/(self.costAvg-self.costMin+self.eps)),1)
        if reward_temp == 1:
            self.reward = reward_temp
        elif 1 > reward_temp > 0:
            self.reward=reward_temp*self.reward_b
        else:
            self.reward=0
                    
    # Update probability distribution
    # -------------------------------
    def updateProbs(self):
    
        #update the distributions
        #pVector=pVector+learnRate*reward*pVector
        self.pVector=self.pVector+self.a*self.learnRate*self.reward*self.pVector+self.b*self.learnRate*(1-self.reward)*self.pVector
        
        for i in range(0,self.nParams):
            self.Qtable[self.selectedIndex[0,i],i]=np.minimum(self.pVector[0,i],self.probLimit)
            
        #normalize
        for i in range(0,self.nParams):
            self.Qtable[:,i]=self.Qtable[:,i]/np.sum(self.Qtable[:,i])
            
    # Randomly select an option 
    # --------------------------       
    def getParams(self):
    
        for i in range(0,self.nParams):
            # randomly select an option according to the current distribution in the Q table
            self.selectedIndex[0,i] = (np.cumsum(self.Qtable[:,i]) >= np.random.random()).argmax()
            # selected value cooresponding to that index
            self.selectedVals[0,i] = self.optionsTable[self.selectedIndex[0,i],0]
            # build a new vector composed of the corresponding probabilities 
            self.pVector[0,i]=self.Qtable[self.selectedIndex[0,i],0]
            #Return the parameters
        
        return self.selectedVals.transpose().ravel()

    # Run the learning
    #------------------------
    def learn(self,quad,traj,ctrl,Ts,t):
        
        if self.doLearn == 1:
        
            # Compute error for learning (using last timestep's params)
            self.computeError(quad,traj)
            self.trialCounter += Ts
            if self.trialCounter > self.trialLen: #if the trial is over
                # compute the reward signal
                self.computeReward(self.error_accumulated)
                # update the probabilities
                self.updateProbs()
                # update tuning parameters from fala (for next iteration)
                selPars = self.getParams()
                # send tuning parameters to controller (for next iteration)
                ctrl.tune(selPars)
                #print
                print('Trial completed: ', t)
                #print('Trial completed at ', t, 'secs,',' max prob = ',np.amax(self.Qtable,axis=0))
                #print('selected values: ',self.selectedVals)
                #print('reward signal: ',self.reward)
                #print('error = ',self.error_accumulated)
                # reset counter
                self.trialCounter = 0
                # reset accumulated error
                self.error_accumulated = 0   
       
        




       
        
        