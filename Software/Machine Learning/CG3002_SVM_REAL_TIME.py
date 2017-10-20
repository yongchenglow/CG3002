# -*- coding: utf-8 -*-
"""
Created on Thu Oct 19 16:48:13 2017

@author: Daryl
"""

import sys
import time
import csv
import socket
import base64
from Crypto.Cipher import AES
from Crypto import Random

from scipy import signal
import pandas as pd
import numpy as np
from sklearn import preprocessing, cross_validation, metrics
from sklearn.svm import SVC
from sklearn.preprocessing import label_binarize
from sklearn.cross_validation import cross_val_score
from sklearn.metrics import confusion_matrix
from numpy import array, zeros, argmin, inf
from numpy.linalg import norm
import time


from multiprocessing import Process

rawData = []
arduinoFlag = True

start = time.time()
##### buffer #####
bufFlag = 0
if(time.time()-start == 20.0):
    bufFlag = 1
    start = time.time()     
else:
    bufFlag = 0
    
if(bufFlag == 1):               #begin preprocessing to predictive analysis
    X = pd.DataFrame(rawData)

##### label encoder #####
y = pd.DataFrame(df['LABELS'])
le = preprocessing.LabelEncoder()
le.fit(df['LABELS'])
label = list(le.classes_)
df['LABELS'] = le.transform(df['LABELS'])
y = np.array(df['LABELS'])
print (time.time()-start)
X = np.array(df.drop(['LABELS'], 1)) #removing labels
X = preprocessing.normalize(X) #normalize the dataset

def segment_signal(df, window_size):
    N = df.shape[0]
    dim = df.shape[1]
    K =int(N/window_size)
    segments = np.empty((K, window_size, dim))
    for i in range(K):
        segment = df[i*window_size : i*window_size+window_size,:]
        segments[i] = np.vstack(segment)
    return segments

segmented_df = segment_signal(X, 50)

print (time.time()-start)
nLayers = segmented_df.shape[0]
nRows = segmented_df.shape[1]
nColumns = segmented_df.shape[2]

##### Mean of raw data #####
mean_list = []
for i in range(nLayers):
    sliceLayer = segmented_df[i,::] 
    row = []
    for j in range(nColumns):
        temp = sliceLayer[:,j] 
        mean = np.mean(temp)
        row = np.append(row, [mean])
    mean_list = np.append(mean_list, row)
mean_list = mean_list.reshape((nLayers, nColumns))

##### std of raw data #####
std_list = []
for i in range(nLayers):
    sliceLayer = segmented_df[i,::] 
    row = []
    for j in range(nColumns):
        temp = sliceLayer[:,j] 
        std = np.std(temp)
        row = np.append(row, [std])
    std_list = np.append(std_list, row)
std_list = std_list.reshape((nLayers, nColumns))

median_list = []
for i in range(nLayers):
    sliceLayer = segmented_df[i,::] 
    row = []
    for j in range(nColumns):
        temp = sliceLayer[:,j] 
        median = np.median(temp)
        row = np.append(row, [median])
    median_list = np.append(median_list, row)
median_list = median_list.reshape((nLayers, nColumns))


#####Feature List#####
feature_list = np.hstack((mean_list, std_list, median_list))    

##### labels #####
y_list = []
y = y.reshape(35250, 1)
y = segment_signal(y, 50)

nLayers = y.shape[0]
nRows = y.shape[1]
nColumns = y.shape[2]

##### Convert 3d to 2d #####
for i in range(nLayers):
    sliceLayer = y[i,::] #loop through each layer
    row = []
    for j in range(nColumns): #nColumns = 3, loop through each column in each layer of 125 rows
        temp = sliceLayer[:,j:] #temp = 125 rows for the jth column
        mean = np.mean(temp)
        row = np.append(row, [mean])
    y_list = np.append(y_list, row)

y_list = np.floor(y_list)


X_train, X_test, y_train, y_test = cross_validation.train_test_split(feature_list,
                                                   y_list, test_size = 0.25)
clf = SVC()
clf.fit(X_train, y_train)
accuracy_rate_1 = clf.score(X_test, y_test)

##### Applying model to test set #####
y_predict = clf.predict(X_test)
##### Average score achieved in validation K=10 #####
validate_score = cross_val_score(clf, mean_list, y_list, cv= 10).mean()
##### Confusion Matrix #####
matrix = metrics.confusion_matrix(y_test, y_predict)


results = pd.DataFrame(matrix, columns = ['busdriver', 'frontback', 'jumping', 'jumpingjack', 'sidestep', 'squatturnclap',\
                                          'turnclap', 'wavehands', 'window', 'window360'])
    
results.rename(index ={0:'busdriver', 1:'frontback', 2:'jumping', 3:'jumpingjack', 4:'sidestep', 5:'squatturnclap',\
                       6:'turnclap', 7:'wavehands', 8:'window', 9:'window360' }, inplace = True)
accuracy = pd.DataFrame([accuracy_rate_1],columns = ['ACCURACY'])
accuracy.rename(index ={0:'ACCURACY'}, inplace = True)
results = results.append(accuracy)
results = results.fillna('')
results.to_csv('Accuracy_Matrix_SVM.csv')
#print (time.time()-start)
