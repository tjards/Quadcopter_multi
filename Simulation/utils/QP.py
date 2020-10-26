#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 21 20:19:56 2020

@author: tjards
"""
import numpy as np
from scipy import optimize as opt
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation 

# computes the centerpoint of the constraint
# ------------------------------------------
def computeXp(x,xo,r,ro,rb):
        
        d = np.linalg.norm(x-xo)                            # distance between obs and veh    
        xp = xo + np.multiply(np.divide(ro+r+rb,d),(x-xo))  # compute centerpoint

        return xp, d


# find an optimal point around obstacle 
# -------------------------------------    
def runOpt(x, xt, xp, A, b):
    
    cons = {'type':'ineq',
            'fun':lambda x: b - np.dot(A,x),
            'jac':lambda x: -A}
               
    # unconstrained case
    # ux = opt.minimize(f, x, constraints=None, args = [xt[0], xt[1]])
    
    # constrained case
    cx = opt.minimize(f, x, constraints=cons, args = [xt[0], xt[1]])
    #cx = opt.minimize(f, x, constraints=cons, args = [xt[0], xt[1], xt[2]])
    #cx = opt.minimize(f, x0, bounds=bnds, constraints=cons)

    return cx


# the objective function
# ----------------------
def f(x, xt):
    
    return ((x[0]-xt[0])**2 + (x[1]-xt[1])**2)
    #return ((x[0]-xt[0])**2 + (x[1]-xt[1])**2 + (x[2]-xt[2])**2)


# animate the obstacle avoidance
# -----------------------------
def myAnimation(myData_QP,nStates):

    # initialize data store 
    # ---------------------
    xv = myData_QP[0,0:nStates] 
    xo = myData_QP[0,nStates:2*nStates] 
    xt = myData_QP[0,2*nStates:3*nStates] 
    A = myData_QP[0,3*nStates:4*nStates] 
    b = myData_QP[0,4*nStates:4*nStates+1] 
    ux = myData_QP[0,4*nStates+1:5*nStates+1] 
    cx = myData_QP[0,5*nStates+1:6*nStates+1]

    # initiate the figure
    # -------------------
    fig, ax = plt.subplots()
    plotSize = 5
    x = np.linspace(-plotSize, plotSize, 100)
    y = np.linspace(-plotSize, plotSize, 100)
    X, Y = np.meshgrid(x, y)
    Z = f(np.vstack([X.ravel(), Y.ravel()]), xt).reshape((100,100))
    ax.contour(X, Y, Z, np.arange(-2,20, 1), alpha=0.3)
    
    # initialize lines that need updating
    # ------------------------------------
    line_obs = ax.scatter(xo[0], xo[1], s=400, marker = 'o', alpha = 0.2, color = 'blue')
    line_veh = ax.scatter(xv[0], xv[1], s=400, marker = 'o', alpha = 0.2, color = 'green')
    line_tar = ax.scatter(xt[0], xt[1], s=400, marker = 'o', alpha = 0.2, color = 'black')
    line_ux = ax.scatter(xt[0], xt[1], s=100, marker = 'x', alpha = 1, color = 'black')
    line_cx = ax.scatter(xt[0], xt[1], s=100, marker = 'x', alpha = 1, color = 'red')
    line_xv2cx, = ax.plot([xv[0],xt[0]],[xv[1],xt[1]], 'g:', linewidth=1)
    line_xt2cx, = ax.plot([xt[0],xt[0]],[xt[1],xt[1]], 'b:', linewidth=1)

    #%% this is the constraint illustration
    # --------------------------------------
    #yy = -np.divide(A[0,0],A[0,1])*x+np.divide(b,A[0,1])
    yy = -np.divide(A[0],A[1])*x+np.divide(b,A[1])
    line_const, = ax.plot(x, yy, 'r:', linewidth=1)
    
    plt.axis([-plotSize,plotSize,-plotSize,plotSize])
    plt.xlabel('x-direction')
    plt.ylabel('y-direction')
    plt.title('obstacle avoid')
    #plt.show()
    
    #%% update the lines
    # ------------------
    def update(i):
      
        # extract data
        # --------------
        xv = myData_QP[i,0:2] 
        xo = myData_QP[i,2:4] 
        xt = myData_QP[i,4:6] 
        A = myData_QP[i,6:8] 
        b = myData_QP[i,8:9] 
        ux = myData_QP[i,9:11] 
        cx = myData_QP[i,11:13]
        
        # update the lines
        # -----------------
        line_obs.set_offsets([xo[0],xo[1]])
        line_veh.set_offsets([xv[0],xv[1]])
        line_tar.set_offsets([xt[0],xt[1]])
        line_ux.set_offsets([ux[0], ux[1]])
        line_cx.set_offsets([cx[0], cx[1]])
        line_xv2cx.set_xdata([xv[0],cx[0]])
        line_xv2cx.set_ydata([xv[1],cx[1]])
        line_xt2cx.set_xdata([xt[0],cx[0]])
        line_xt2cx.set_ydata([xt[1],cx[1]])
        #yy = -np.divide(A[0,0],A[0,1])*x+np.divide(b,A[0,1])
        yy = -np.divide(A[0],A[1])*x+np.divide(b,A[1])
        line_const.set_xdata(x)
        line_const.set_ydata(yy)    
        
    
    #%% build the animation
    # ---------------------
    anim = FuncAnimation(fig, update, frames=np.arange(0, 20), interval=200, blit=False)
    anim.save('test.gif', writer='ffmpeg') #my add







#%% LEGACY stuff

    # cons = ({'type': 'eq',
    #          'fun' : lambda x: np.array([x[0]**3 - x[1]]),
    #          'jac' : lambda x: np.array([3.0*(x[0]**2.0), -1.0])},
    #         {'type': 'ineq',
    #          'fun' : lambda x: np.array([x[1] - (x[0]-1)**4 - 2])})    
    # bnds = ((0.5, 1.5), (1.5, 2.5))




