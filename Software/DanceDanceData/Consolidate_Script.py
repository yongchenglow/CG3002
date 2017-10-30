# -*- coding: utf-8 -*-
"""
Created on Mon Oct 23 01:10:35 2017

@author: Daryl
"""
import pandas as pd
import glob as glob


from os import walk
import os

csv_list = []
for root, dirs,files in os.walk(r"data231017", topdown=True):
    for name in files:
        if name.endswith(".csv"):
            csv_list.append(os.path.join(root, name))
print (csv_list)
combined_csv = pd.concat([ pd.read_csv(f, header=None) for f in csv_list ], ignore_index=True)

combined_csv.to_csv('filtered_activities_30OCT2017.csv')   