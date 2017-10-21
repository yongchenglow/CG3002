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

from ML_FUNCTIONS import time_features, segment_signal

from multiprocessing import Process

rawData = []
arduinoFlag = True

start = time.time()
while (arduinoFlag == True):
    ##### buffer #####
    bufFlag = 0
    if(time.time()-start == 20.0):
        bufFlag = 1
        start = time.time()     
    else:
        bufFlag = 0
        
    if(bufFlag == 1):               #begin preprocessing to predictive analysis
        X = pd.DataFrame(rawData)
        
        X = preprocessing.normalize(X) #normalize the dataset
        
        X = segment_signal(X, 50)
        
        time_feature_list = []
        
        time_feature_list = time_features(X, time_feature_list)
        
        
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
