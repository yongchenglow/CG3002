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
from scipy import stats
import pandas as pd
import numpy as np
from sklearn.svm import SVC
from sklearn.externals import joblib
from sklearn import preprocessing
import time

import ML_FUNCTIONS as ml

from multiprocessing import Process

rawData = []
final_result = None
arduinoFlag = True
start = time.time()

##### load model #####
clf = joblib.load('trained_model.sav') 

##### Software system start ##### 
while (arduinoFlag == True):
    
    ##### buffer #####
    bufFlag = 0
    if(time.time()-start == 20.0):
        bufFlag = 1
        start = time.time()     
    else:
        bufFlag = 0
        
    if(bufFlag == 1):               #begin preprocessing to predictive analysis
        X = np.array(rawData)
        
        X = pd.DataFrame(rawData)
        
        X = preprocessing.normalize(X) #normalize the dataset
        
        X = ml.segment_signal(X, 50) #segmentation to 3d for feature extraction
        
        time_feature_list = []
        
        time_feature_list = ml.time_features(X, time_feature_list) #feature extraction and conver to 2d
        
       
        ##### Predict #####
        result = clf.predict(X)   
        result = stats.mode(result) #find the mode in result
        result = ml.result_output(result) #output the result as string
        bufFlag = 0  #reset flag to take in next dataset
        
def dataFromArduino():
    global rawData
    count = 0
    flag = True
    print('run dataFromArduino')
    
    while flag:
        for i in range(0, 9):
            raw = []
            for j in range(0, 9):
                raw.append(0)
            rawData.append(raw)
        
        print(rawData)
        
        if (count < 20):
            count += 1
        else:
            flag = False
            print('stop running dataFromArduino')

Process(target=dataFromArduino, args=()).start()
Process(target=learn, args=()).start()