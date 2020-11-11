#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov  1 19:07:22 2020

@author: tjards
"""
import matplotlib.pyplot as plt
import csv
import pandas as pd
import numpy as np



a = np.load('pf_energyDepletion_veh0.npy')
b = np.load('pf_energyDepletion_veh1.npy')
c = np.load('pic_energyDepletion_veh0.npy')
d = np.load('pic_energyDepletion_veh1.npy')

sa = np.load('pf_states0.npy')
sb = np.load('pf_states1.npy')
sc = np.load('pic_states0.npy')
sd = np.load('pic_states1.npy')

error_a = np.cumsum(np.sqrt(np.square(sa[:,0:3]-sa[:,3:6])))
error_b = np.cumsum(np.sqrt(np.square(sb[:,0:3]-sb[:,3:6])))
error_c = np.cumsum(np.sqrt(np.square(sc[:,0:3]-sc[:,3:6])))
error_d = np.cumsum(np.sqrt(np.square(sd[:,0:3]-sd[:,3:6])))
#error = np.vstack([error_a, error_b, error_c, error_d]).transpose()
#error_total = np.cumsum(error, axis = 1)[:,3]


# plt.figure()
# plt.plot(-a/np.max(b))
# plt.plot(-b/np.max(b))
# plt.plot(-c/np.max(b))
# plt.plot(-d/np.max(b))
# plt.grid(True)
# plt.legend(['PF_0','PF_1','PIC_0','PIC_1'])
# plt.xlabel('Time (s)')
# plt.ylabel('Normalized Energy Depletion (J)')
# plt.draw()


plt.figure()
plt.plot(a-c,'b')
plt.plot(b-d,'g')
plt.grid(True)
plt.legend(['Vehicle 0','Vehicle 1'])
plt.xlabel('Time (s)')
plt.ylabel('Energy Savings due to Shifting (J)')
plt.draw()


plt.figure()
plt.plot(error_a/np.max(error_b),'--b')
plt.plot(error_b/np.max(error_b),'--g')
plt.plot(error_c/np.max(error_b),'b')
plt.plot(error_d/np.max(error_b),'g')
plt.grid(True)
plt.legend(['No shifting [vehicle 0]','No shifting [vehicle 1]','shifting [vehicle 0]','shifting [vehicle 1]'])
plt.xlabel('Time (s)')
plt.ylabel('Error Accumulation (Normalized)')
plt.draw()






#df = pd.read_csv("errors.csv")
#df = pd.read("energyDepletion.npy")
#df.plot()


