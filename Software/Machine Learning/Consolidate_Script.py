# -*- coding: utf-8 -*-
"""
Created on Mon Oct 23 01:10:35 2017

@author: Daryl
"""
import pandas as pd
import glob as glob



import os
csv_list = []
for root, dirs,files in os.walk("file:///C:/Users/Daryl/Desktop/busdriver", topdown=True):
    for name in files:
        csv_list.append(os.path.join(root, name))
combined_csv = pd.concat([ pd.read_csv(f, header=None) for f in csv_list ], ignore_index=True)  

combined_csv.to_csv('filtered_sit.csv')        