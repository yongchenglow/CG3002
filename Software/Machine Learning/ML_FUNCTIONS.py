# -*- coding: utf-8 -*-
"""
Created on Sat Oct 21 20:46:27 2017

@author: Daryl
"""
import pandas as pd 
import numpy as np   
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
from scipy import stats

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
    variance_list =[]
    
    for i in range(nLayers):
        sliceLayer = segmented_df[i,::] 
        mean_row = []
        std_row =[]
        median_row = []
        variance_row = []
        for j in range(nColumns):
            temp = sliceLayer[:,j] 
            mean = np.mean(temp)
            std = np.std(temp)
            median = np.median(temp)
            variance = np.var(temp)
            
            mean_row = np.append(mean_row, [mean])
            std_row = np.append(std_row, [std])
            median_row = np.append(median_row, [median])
            variance_row = np.append(variance_row, [variance])
        mean_list = np.append(mean_list, mean_row)
        std_list = np.append(std_list, std_row)
        median_list = np.append(median_list, median_row)
        variance_list = np.append(variance_list, variance_row)
    mean_list = mean_list.reshape((nLayers, nColumns))
    std_list = std_list.reshape((nLayers, nColumns))
    median_list = median_list.reshape((nLayers, nColumns))
    variance_list = variance_list.reshape((nLayers, nColumns))

    feature_list = np.hstack((mean_list, std_list, median_list, variance_list))  
    
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
            '0': 'busdriver',
            '1': 'frontback',
            '2': 'jumping',
            '3': 'jumpingjack',
            '4': 'sidestep',
            '5': 'squatturnclap',
            '6': 'turnclap',
            '7': 'wavehands',
            '8': 'window',
            '9': 'window360'
            }
    return action_map[result]


    return action_map[result]
'''def result_output(result):
    action_map = {
            '0': 'Wavehands',
            '1': 'Busdriver',
            '2': 'Frontback',
            '3': 'Sidestep',
            '4': 'Jumping',
            '5': 'Jumpingjacks',
            '6': 'Turnclap',
            '7': 'Squaturnclap',
            '8': 'Window',
            '9': 'windowSpin'
            }
    return action_map[result]'''
    
def final_check(array, move):
    count = 0 
    if(move in array):
        count = count+1
    check1 = stats.mode(array[:3:1])
    check2 = stats.mode(array[4:6:1])
    check3 = stats.mode(array[7::1])
    final = check1 + check2 + check3
    if(count == 3):
        return move #if all 3 windows return 3 of the same mode, return the same move
    else:
        return stats.mode(final) # returns the mode of the 3 windows, still not a reliable check
    
    

    
    
    
    
    
    
    