# -*- coding: utf-8 -*-


import pandas as pd
import numpy as np
import scipy
from scipy import stats

import matplotlib.pyplot as plt
from sklearn import preprocessing, cross_validation
from sklearn.svm import SVC
from sklearn import metrics
from sklearn.preprocessing import label_binarize
from sklearn.cross_validation import cross_val_score
from sklearn.metrics import confusion_matrix

df = pd.read_csv('file:///C:/Users/Daryl/Desktop/Year 3 Sem 1/CG3002/Code/Filtering/filtered_activities.csv')


y = pd.DataFrame(df['LABELS'])
le = preprocessing.LabelEncoder()
le.fit(df['LABELS'])
label = list(le.classes_)
df['LABELS'] = le.transform(df['LABELS'])
y = np.array(df['LABELS'])

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

segmented_df = segment_signal(X, 125)

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
slice_mean = mean_list[:]
slice_std = std_list[:]
slice_median = median_list[:]
feature_list = np.hstack((slice_mean, slice_std, slice_median))
    



##### labels #####
y_list = []
y = y.reshape(180000, 1)
y = segment_signal(y, 125)

nLayers = y.shape[0]
nRows = y.shape[1]
nColumns = y.shape[2]
print(nLayers)
##### Convert 3d to 2d #####
for i in range(nLayers):
    sliceLayer = y[i,::] #loop through each layer
    row = []
    for j in range(nColumns): #nColumns = 3, loop through each column in each layer of 125 rows
        temp = sliceLayer[:,j:] #temp = 125 rows for the jth column
        mean = np.mean(temp)
        row = np.append(row, [mean])
    y_list = np.append(y_list, row)







X_train, X_test, y_train, y_test = cross_validation.train_test_split(mean_list,
                                                   y_list, test_size = 0.5)

clf = SVC()
clf.fit(X_train, y_train)
accuracy_rate_1 = clf.score(X_test, y_test)
validate_score = cross_val_score(clf, mean_list, y_list, cv= 5)
y_predict = clf.predict(X_test)

cm = confusion_matrix(y_test, y_predict)
matrix = metrics.confusion_matrix(y_test, y_predict)
accuracy_rate_2 = metrics.accuracy_score(y_test, y_predict)
sensitivity_rate = metrics.recall_score(y_test, y_predict, average= 'macro')
precision_rate = metrics.precision_score(y_test, y_predict, average='macro')
f1_score = metrics.f1_score(y_test, y_predict, average= 'macro')

sitting_sitting = matrix[0,0]
sitting_walking = matrix[1,0]
sitting_jumping = matrix[2,0]
walking_walking = matrix[1,1]
walking_sitting = matrix[1,0]
walking_jumping = matrix[1,2]
jumping_jumping = matrix[2,2]
jumping_sitting = matrix[2,0]
jumping_walking = matrix[2,1]

print(sitting_sitting)
print(walking_walking)
print(jumping_jumping)
print("")
print(accuracy_rate_1)
print (validate_score.mean())
print(accuracy_rate_2)
print("")
print(matrix)
print(sensitivity_rate)
print(precision_rate)
print(f1_score)

results = pd.DataFrame(cm, columns = ['SITTING', 'WALKING', 'JUMPING'])

results.rename(index ={0:'SITTING', 1:'WALKING', 2: 'JUMPING' }, inplace = True)
accuracy = pd.DataFrame([accuracy_rate_1],columns = ['ACCURACY'])
accuracy.rename(index ={0:'ACCURACY'}, inplace = True)
results = results.append(accuracy)
results = results.fillna('')
results.to_csv('accuracy_matrix_SVM.csv')



print (results)
