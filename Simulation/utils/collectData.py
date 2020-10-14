#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 13 20:02:04 2020

@author: tjards
"""

import numpy as np


def collect_init(quad, traj, ctrl, fala, Ti, numTimeStep):
    
    # Initialize Result Matrixes
    # ---------------------------
    #numTimeStep = int(Tf/Ts+1)

    t_all          = np.zeros(numTimeStep)
    s_all          = np.zeros([numTimeStep, len(quad.state)])
    pos_all        = np.zeros([numTimeStep, len(quad.pos)])
    vel_all        = np.zeros([numTimeStep, len(quad.vel)])
    quat_all       = np.zeros([numTimeStep, len(quad.quat)])
    omega_all      = np.zeros([numTimeStep, len(quad.omega)])
    euler_all      = np.zeros([numTimeStep, len(quad.euler)])
    sDes_traj_all  = np.zeros([numTimeStep, len(traj.sDes)])
    sDes_calc_all  = np.zeros([numTimeStep, len(ctrl.sDesCalc)])
    w_cmd_all      = np.zeros([numTimeStep, len(ctrl.w_cmd)])
    wMotor_all     = np.zeros([numTimeStep, len(quad.wMotor)])
    thr_all        = np.zeros([numTimeStep, len(quad.thr)])
    tor_all        = np.zeros([numTimeStep, len(quad.tor)])
    # learning items
    #falaError_all  = np.zeros([numTimeStep, len(fala.error_accumulated)])
    falaError_all  = np.zeros([numTimeStep, 1])
    

    t_all[0]            = Ti
    s_all[0,:]          = quad.state
    pos_all[0,:]        = quad.pos
    vel_all[0,:]        = quad.vel
    quat_all[0,:]       = quad.quat
    omega_all[0,:]      = quad.omega
    euler_all[0,:]      = quad.euler
    sDes_traj_all[0,:]  = traj.sDes
    sDes_calc_all[0,:]  = ctrl.sDesCalc
    w_cmd_all[0,:]      = ctrl.w_cmd
    wMotor_all[0,:]     = quad.wMotor
    thr_all[0,:]        = quad.thr
    tor_all[0,:]        = quad.tor
    #learning items
    falaError_all[0,:]  = fala.error_accumulated




def collect(t, quad, traj, ctrl, fala):
    
    
    # print("{:.3f}".format(t))
    t_all[i]             = t
    s_all[i,:]           = quad.state
    pos_all[i,:]         = quad.pos
    vel_all[i,:]         = quad.vel
    quat_all[i,:]        = quad.quat
    omega_all[i,:]       = quad.omega
    euler_all[i,:]       = quad.euler
    sDes_traj_all[i,:]   = traj.sDes
    sDes_calc_all[i,:]   = ctrl.sDesCalc
    w_cmd_all[i,:]       = ctrl.w_cmd
    wMotor_all[i,:]      = quad.wMotor
    thr_all[i,:]         = quad.thr
    tor_all[i,:]         = quad.tor
    # learning items
    falaError_all[i,:]  = fala.error_accumulated
    
    
    print('collecting...')