# -*- coding: utf-8 -*-
"""
Created on Thu Oct  5 12:46:02 2017

@author: zhuangyufeng
"""
from scipy import signal
import pandas as pd
import numpy as np
from sklearn import preprocessing, cross_validation, neighbors, metrics
from sklearn.svm import SVC
from sklearn.preprocessing import label_binarize
from sklearn.cross_validation import cross_val_score
from sklearn.metrics import confusion_matrix
from numpy import array, zeros, argmin, inf
from numpy.linalg import norm

def dtw(x, y, dist):
    """
    Computes Dynamic Time Warping (DTW) of two sequences.

    :param array x: N1*M array
    :param array y: N2*M array
    :param func dist: distance used as cost measure

    Returns the minimum distance, the cost matrix, the accumulated cost matrix, and the wrap path.
    """
    assert len(x)
    assert len(y)
    r, c = len(x), len(y)
    D0 = zeros((r + 1, c + 1))
    D0[0, 1:] = inf
    D0[1:, 0] = inf
    D1 = D0[1:, 1:] # view
    for i in range(r):
        for j in range(c):
            D1[i, j] = dist(x[i], y[j])
    C = D1.copy()
    for i in range(r):
        for j in range(c):
            D1[i, j] += min(D0[i, j], D0[i, j+1], D0[i+1, j])
    if len(x)==1:
        path = zeros(len(y)), range(len(y))
    elif len(y) == 1:
        path = range(len(x)), zeros(len(x))
    else:
        path = _traceback(D0)
    return D1[-1, -1] / sum(D1.shape), C, D1, path

def _traceback(D):
    i, j = array(D.shape) - 2
    p, q = [i], [j]
    while ((i > 0) or (j > 0)):
        tb = argmin((D[i, j], D[i, j+1], D[i+1, j]))
        if (tb == 0):
            i -= 1
            j -= 1
        elif (tb == 1):
            i -= 1
        else: # (tb == 2):
            j -= 1
        p.insert(0, i)
        q.insert(0, j)
    return array(p), array(q)

 
i =0;
j =0;
df_bd1 = pd.read_csv('busdriver.csv')
df_bd2 = pd.read_csv('busdriver2.csv')
df_bd3 = pd.read_csv('busdriver3.csv')
df_bd4 = pd.read_csv('busdriver4.csv')

bd1 = np.array(df_bd1)
bd2 = np.array(df_bd2)
bd3 = np.array(df_bd3)
bd4 = np.array(df_bd4)
#x_list = []
final_1 = []
final_2 = []
final_3 = []
final_4 = []
ultimate_1 = []
collate = []
while(i<9):
    x_1 = bd1[:,0+i].reshape(-1,1)
    x_2 = bd2[:,0+i].reshape(-1,1)
    x_3 = bd3[:,0+i].reshape(-1,1)
    x_4 = bd4[:,0+i].reshape(-1,1)

    dist, cost, acc, path = dtw(x_1, x_2, dist=lambda x_1, x_2: norm(x_1 - x_2, ord=1))
    path_1, path_2 = path   
    for numbers in path_1:
        final_1.append(bd1[numbers,0+i])
       
    for numbers in path_2:
        final_2.append(bd2[numbers,0+i])
        
    dist, cost, acc, path = dtw(x_1, x_3, dist=lambda x_1, x_3: norm(x_1 - x_3, ord=1))
    path_1, path_2 = path
    for numbers in path_2:
        final_3.append(bd3[numbers,0+i])
        
    dist, cost, acc, path = dtw(x_1, x_4, dist=lambda x_1, x_4: norm(x_1 - x_4, ord=1))
    path_1, path_2 = path
    for numbers in path_2:
        final_4.append(bd4[numbers,0+i])    
    
    ultimate_1 = np.hstack((final_1, final_2, final_3, final_4))
    collate = np.hstack(ultimate_1)
        
    i = i+1
    
print(collate)

