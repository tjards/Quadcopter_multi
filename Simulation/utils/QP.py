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
import sys 




def computeXp(x,xo,r,ro,rb):
        
        # compute distance between the two
        d = np.linalg.norm(x-xo)
        # find xp
        
        xp = xo + np.multiply(np.divide(ro+r+rb,d),(x-xo))
        #xp = xo + np.multiply(buffer,(x-xo))
        
        return xp, d

    
def runOpt(x, xt, xp, A, b):
    

    cons = {'type':'ineq',
            'fun':lambda x: b - np.dot(A,x),
            'jac':lambda x: -A}
           
   # x0 = np.array([0, 2.5])
    x0 = x
    
    # unconstrained case
    ux = opt.minimize(f, x0, constraints=None, args = [xt[0], xt[1]])
    # constrained case
    #cx = opt.minimize(f, x0, bounds=bnds, constraints=cons)
    cx = opt.minimize(f, x0, constraints=cons, args = [xt[0], xt[1]])
    
    return ux, cx, A, b

def f(x, xt):
    
    #dx = x - xt  
    #return -(2*x[0]*x[1] + 2*x[0] - x[0]**2 - 2*x[1]**2)
    return ((x[0]-xt[0])**2 + (x[1]-xt[1])**2)



def myAnimation(xv,xo,xt,A,b,r,ro,rb):
  
    # def computeXp(x,xo,r,ro,rb):
        
    #     # compute distance between the two
    #     d = np.linalg.norm(x-xo)
    #     # find xp
        
    #     xp = xo + np.multiply(np.divide(ro+r+rb,d),(x-xo))
    #     #xp = xo + np.multiply(buffer,(x-xo))
        
    #     return xp, d
    
    # def runOpt(x, xt, xp):
        
    #     # A = np.array([[ 1., 1.],
    #     #               [-1., 2.],
    #     #               [-1., 0.],
    #     #               [0., -1.],
    #     #               [0.,  1.]])
        
    #     A = np.zeros([1,2])
    #     A[0,:] = xo - xp
        
    #     #b = np.array([7., 4., 0., 0., 4.])
    #     b = np.dot(xp,(xo-xp))
        
        
        
    #     cons = {'type':'ineq',
    #             'fun':lambda x: b - np.dot(A,x),
    #             'jac':lambda x: -A}
               
    #    # x0 = np.array([0, 2.5])
    #     x0 = x
        
    #     # unconstrained case
    #     ux = opt.minimize(f, x0, constraints=None, args = [xt[0], xt[1]])
    #     # constrained case
    #     #cx = opt.minimize(f, x0, bounds=bnds, constraints=cons)
    #     cx = opt.minimize(f, x0, constraints=cons, args = [xt[0], xt[1]])
        
    #     return ux, cx, A, b
    
    # def f(x, xt):
        
    #     #dx = x - xt  
    #     #return -(2*x[0]*x[1] + 2*x[0] - x[0]**2 - 2*x[1]**2)
    #     return ((x[0]-xt[0])**2 + (x[1]-xt[1])**2)
    
    #%% Initialize Parameters 
    

    
    fig, ax = plt.subplots()
    plotSize = 5
    x = np.linspace(-plotSize, plotSize, 100)
    y = np.linspace(-plotSize, plotSize, 100)
    X, Y = np.meshgrid(x, y)
    Z = f(np.vstack([X.ravel(), Y.ravel()]), xt).reshape((100,100))
    ax.contour(X, Y, Z, np.arange(-2,20, 1), alpha=0.3)
    # initialize lines that need updating 
    line_obs = ax.scatter(xo[0], xo[1], s=4000*ro, marker = 'o', alpha = 0.2, color = 'blue')
    line_veh = ax.scatter(xv[0], xv[1], s=4000*ro, marker = 'o', alpha = 0.2, color = 'green')
    line_tar = ax.scatter(xt[0], xt[1], s=4000*ro, marker = 'o', alpha = 0.2, color = 'black')
    line_ux = ax.scatter(xt[0], xt[1], s=1000*ro, marker = 'x', alpha = 1, color = 'black')
    line_cx = ax.scatter(xt[0], xt[1], s=1000*ro, marker = 'x', alpha = 1, color = 'red')
    line_xv2cx, = ax.plot([xv[0],xt[0]],[xv[1],xt[1]], 'g:', linewidth=1)
    line_xt2cx, = ax.plot([xt[0],xt[0]],[xt[1],xt[1]], 'b:', linewidth=1)
    #A = np.zeros([1,2])
    #b = 0
    yy = -np.divide(A[0,0],A[0,1])*x+np.divide(b,A[0,1])
    line_const, = ax.plot(x, yy, 'r:', linewidth=1)
    plt.axis([-plotSize,plotSize,-plotSize,plotSize])
    plt.xlabel('x-direction')
    plt.ylabel('y-direction')
    plt.title('obstacle avoid')
    plt.show()
    
    
    def update(i):
      
        #%% Move 
        xv[0] = xv[0] + 0.1*i*0.2
        xv[1] = xv[1] + 0.3
        if i > 8:    
            xv[1] = xv[1] - 0.1
    
        # my new ones
        line_obs.set_offsets([xo[0],xo[1]])
        line_veh.set_offsets([xv[0],xv[1]])
        line_tar.set_offsets([xt[0],xt[1]])
    
    
        #%% Compute point on edge of obstacle 
        
        xp, d = computeXp(xv,xo,r,ro,rb)
        
        #update constraints 
        A = np.zeros([1,2])
        A[0,:] = xo - xp
        b = np.dot(xp,(xo-xp))
        
        #%% Run the optimization 
        
        ux, cx, A, b = runOpt(xv, xt, xp, A, b)
        
        # my new one
        line_ux.set_offsets([ux['x'][0], ux['x'][1]])
        line_cx.set_offsets([cx['x'][0], cx['x'][1]])
        line_xv2cx.set_xdata([xv[0],cx['x'][0]])
        line_xv2cx.set_ydata([xv[1],cx['x'][1]])
        line_xt2cx.set_xdata([xt[0],cx['x'][0]])
        line_xt2cx.set_ydata([xt[1],cx['x'][1]])
        
        yy = -np.divide(A[0,0],A[0,1])*x+np.divide(b,A[0,1])
        line_const.set_xdata(x)
        line_const.set_ydata(yy)    
        
    
    
    anim = FuncAnimation(fig, update, frames=np.arange(0, 20), interval=200, blit=False)
    anim.save('test.gif', writer='ffmpeg') #my add

#%% LEGACY stuff

    # cons = ({'type': 'eq',
    #          'fun' : lambda x: np.array([x[0]**3 - x[1]]),
    #          'jac' : lambda x: np.array([3.0*(x[0]**2.0), -1.0])},
    #         {'type': 'ineq',
    #          'fun' : lambda x: np.array([x[1] - (x[0]-1)**4 - 2])})    
    # bnds = ((0.5, 1.5), (1.5, 2.5))




