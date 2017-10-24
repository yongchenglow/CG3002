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


def butter_lowpass(cutoff, fs, order):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filtfilt(data, cutoff, fs, order):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = filtfilt(b, a, data)
    return y

def result_output(result):
    action_map = {
            '0.': 'wavehands',
            '1.': 'busdriver',
            '2.': 'frontback',
            '3.': 'sidestep',
            '4.': 'jumping',
            '5.': 'jumpingjack',
            '6.': 'turnclap',
            '7.': 'squatturnclap',
            '8.': 'window',
            '9.': 'window360'
            }
    result = action_map[result]
    return result

    