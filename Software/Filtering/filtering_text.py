# -*- coding: utf-8 -*-
"""
Created on Wed Sep 20 22:59:21 2017

@author: Daryl
"""

import pandas as pd
import glob as glob



import os
csv_list = []
for root, dirs,files in os.walk("C:/Users/Daryl/Desktop/Year 3 Sem 1/CG3002/dataset/data/data/a01", topdown=True):
    for name in files:
        csv_list.append(os.path.join(root, name))
combined_csv = pd.concat([ pd.read_csv(f, header=None) for f in csv_list ], ignore_index=True)  

combined_csv.to_csv('filtered_sit.csv')        

csv_list = []
for root, dirs,files in os.walk("C:/Users/Daryl/Desktop/Year 3 Sem 1/CG3002/dataset/data/data/a09", topdown=True):
    for name in files:
        csv_list.append(os.path.join(root, name))
combined_csv = pd.concat([ pd.read_csv(f, header=None) for f in csv_list ], ignore_index=True)  

combined_csv.to_csv('filtered_walk.csv')        

csv_list = []
for root, dirs,files in os.walk("C:/Users/Daryl/Desktop/Year 3 Sem 1/CG3002/dataset/data/data/a18", topdown=True):
    for name in files:
        csv_list.append(os.path.join(root, name))
combined_csv = pd.concat([ pd.read_csv(f, header=None) for f in csv_list ], ignore_index=True)  

combined_csv.to_csv('filtered_jump.csv') 
        