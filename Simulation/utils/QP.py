#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 21 20:19:56 2020

@author: tjards
"""
import numpy as np
from scipy import optimize as opt
from matplotlib import pyplot as plt





def computeXp(x,xo,r,ro,rb):
    
    # compute distance between the two
    d = np.linalg.norm(x-xo)
    # find xp
    xp = xo + np.multiply(np.divide(ro+r+rb,d),(x-xo))
    
    return xp

def runOpt(x, xt, xp):
    
    # A = np.array([[ 1., 1.],
    #               [-1., 2.],
    #               [-1., 0.],
    #               [0., -1.],
    #               [0.,  1.]])
    
    A = np.zeros([1,2])
    A[0,:] = xo - xp
    
    #b = np.array([7., 4., 0., 0., 4.])
    b = np.dot(xp,(xo-xp))
    
    
    
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




#%% Initialize Parameters 

xv = np.array([1,1.5])   # vehicle position
xo = np.array([2.5,2])  # obstacle position
xt = np.array([3,3])  # target position 
r = 0.1
ro = 0.1
rb = 0.5

#%% Compute point on edge of obstacle 

xp = computeXp(xv,xo,r,ro,rb)

#%% Run the optimization 

ux, cx, A, b = runOpt(xv, xt, xp)

#%% plot
plt.figure()
plotSize = 5
x = np.linspace(0, plotSize, 100)
y = np.linspace(0, plotSize, 100)
X, Y = np.meshgrid(x, y)
Z = f(np.vstack([X.ravel(), Y.ravel()]), xt).reshape((100,100))
plt.contour(X, Y, Z, np.arange(-2,10, 1), alpha=0.3)
#plt.plot(x, x**3, 'k:', linewidth=1)
#plt.plot(x, (x-1)**4+2, 'k:', linewidth=1)
plt.text(ux['x'][0], ux['x'][1], 'x', va='center', ha='center', size=20, color='blue')
plt.text(cx['x'][0], cx['x'][1], 'x', va='center', ha='center', size=20, color='red')
plt.text(xv[0],xv[1], 'v', va='center', ha='center', size=20, color='green')
plt.scatter(xv[0], xv[1], s=4000*ro, marker = 'o', alpha = 0.2, color = 'green')
plt.text(xo[0],xo[1], 'o', va='center', ha='center', size=20, color = 'blue')
plt.scatter(xo[0], xo[1], s=4000*ro, marker = 'o', alpha = 0.2, color = 'blue')
yy = -np.divide(A[0,0],A[0,1])*x+np.divide(b,A[0,1])
plt.plot(x, yy, 'r:', linewidth=1)
#plt.fill([0.5,0.5,1.5,1.5], [2.5,1.5,1.5,2.5], alpha=0.3)
#plt.fill([0.5,0.5,1.5,1.5], [2.5,1.5,1.5,2.5], alpha=0.3)
plt.plot([xv[0],cx['x'][0]],[xv[1],cx['x'][1]], 'g:', linewidth=1)
plt.plot([xt[0],cx['x'][0]],[xt[1],cx['x'][1]], 'b:', linewidth=1)
plt.axis([0,plotSize,0,plotSize])
plt.xlabel('x-direction')
plt.ylabel('y-direction')
plt.title('obstacle avoid')
plt.show()

#%% LEGACY stuff

    # cons = ({'type': 'eq',
    #          'fun' : lambda x: np.array([x[0]**3 - x[1]]),
    #          'jac' : lambda x: np.array([3.0*(x[0]**2.0), -1.0])},
    #         {'type': 'ineq',
    #          'fun' : lambda x: np.array([x[1] - (x[0]-1)**4 - 2])})    
    # bnds = ((0.5, 1.5), (1.5, 2.5))




