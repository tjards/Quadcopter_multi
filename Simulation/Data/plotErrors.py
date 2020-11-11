#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov  1 19:07:22 2020

@author: tjards
"""
import matplotlib.pyplot as plt
import csv
import pandas as pd


df = pd.read_csv("errors.csv")
df.plot()


