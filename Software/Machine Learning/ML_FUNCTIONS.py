# -*- coding: utf-8 -*-
"""
Created on Sat Oct 21 20:46:27 2017

@author: Daryl
"""
import pandas as pd 
import numpy as np   
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

def segment_signal(df, window_size):
            N = df.shape[0]
            dim = df.shape[1]
            K =int(N/window_size)
            segments = np.empty((K, window_size, dim))
            for i in range(K):
                segment = df[i*window_size : i*window_size+window_size,:]
                segments[i] = np.vstack(segment)
            return segments
        
def time_features(segmented_df, feature_list):
    nLayers = segmented_df.shape[0]
    #nRows = segmented_df.shape[1]
    nColumns = segmented_df.shape[2]
    
    mean_list = []
    std_list = []
    median_list = []
    
    for i in range(nLayers):
        sliceLayer = segmented_df[i,::] 
        mean_row = []
        std_row =[]
        median_row = []
        for j in range(nColumns):
            temp = sliceLayer[:,j] 
            mean = np.mean(temp)
            std = np.std(temp)
            median = np.median(temp)
            
            mean_row = np.append(mean_row, [mean])
            std_row = np.append(std_row, [std])
            median_row = np.append(median_row, [median])
        mean_list = np.append(mean_list, mean_row)
        std_list = np.append(std_list, std_row)
        median_list = np.append(median_list, median_row)
    mean_list = mean_list.reshape((nLayers, nColumns))
    std_list = std_list.reshape((nLayers, nColumns))
    median_list = median_list.reshape((nLayers, nColumns))

    feature_list = np.hstack((mean_list, std_list, median_list))  
    
    return feature_list


def butter_lowpass(lowcut, highcut, order):
    nyq = 0.5 * highcut
    normal_cutoff = lowcut / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filtfilt(data, lowcut, highcut, order):
    b, a = butter_lowpass(lowcut, highcut, order=order)
    y = filtfilt(b, a, data)
    return y