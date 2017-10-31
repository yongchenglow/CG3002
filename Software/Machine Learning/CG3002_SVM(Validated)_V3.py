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
import pickle
from sklearn import preprocessing, cross_validation, metrics, neighbors
from sklearn.svm import SVC
from sklearn.cross_validation import cross_val_score
from sklearn.metrics import confusion_matrix
from sklearn.externals import joblib
import time

from ML_FUNCTIONS import time_features, segment_signal

from multiprocessing import Process

start = time.time()
df = pd.read_csv('/home/pi/Desktop/CG3002/Software/DanceDanceData/data311017/Consolidated_311017.csv') 

##### label encoder #####
y = pd.DataFrame(df['LABELS'])
le = preprocessing.LabelEncoder()
le.fit(df['LABELS'])
label = list(le.classes_)
df['LABELS'] = le.transform(df['LABELS'])
y = np.array(df['LABELS'])
X = np.array(df.drop(['LABELS'], 1)) #removing labels


##### normalize #####
X = preprocessing.normalize(X) #normalize the dataset

##### segmentation #####
segmented_df = segment_signal(X, 50)

##### time feature #####
time_feature_list = []       
time_feature_list = time_features(segmented_df, time_feature_list)

##### labels #####
y_list = []
y = y.reshape(df.shape[0], 1)
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

##### Training and Validation #####
X_train, X_test, y_train, y_test = cross_validation.train_test_split(time_feature_list,
                                                   y_list, test_size = 0.25)
clf = neighbors.KNeighborsClassifier(n_neighbors = 10)
#clf = SVC()
clf.fit(X_train, y_train)
accuracy_rate_1 = clf.score(X_test, y_test)

##### Save model #####
with open('my_trained_classifier_joblib_31Oct.pkl', 'wb') as fid:
    joblib.dump(clf, fid)    


##### Applying model to test set #####
y_predict = clf.predict(X_test)

#print(y_predict)

##### Average score achieved in validation K=10 #####
validate_score = cross_val_score(clf, time_feature_list, y_list, cv= 10).mean()

##### Confusion Matrix #####
matrix = metrics.confusion_matrix(y_test, y_predict)


#results = pd.DataFrame(matrix, columns = ['busdriver', 'frontback', 'jumping', 'jumpingjack', 'sidestep', 'squatturnclap',\
#                                          'turnclap', 'wavehands', 'window', 'window360'])
    
#results.rename(index ={0:'busdriver', 1:'frontback', 2:'jumping', 3:'jumpingjack', 4:'sidestep', 5:'squatturnclap',\
#                       6:'turnclap', 7:'wavehands', 8:'window', 9:'window360' }, inplace = True)
    
results = pd.DataFrame(matrix, columns = ['Wavehands', 'Busdriver', 'Frontback', 'Sidestep', 'Jumping', 'Jumpingjacks',\
                                          'Turnclap', 'Squatturnclap', 'Window', 'Windowspin'])
    
results.rename(index ={0:'Wavehands', 1:'Busdriver', 2:'Frontback', 3:'Sidestep', 4:'Jumping', 5:'Jumpingjacks',\
                       6:'Turnclap', 7:'Squatturnclap', 8:'window', 9:'Windowspin' }, inplace = True)
accuracy = pd.DataFrame([accuracy_rate_1],columns = ['ACCURACY'])
accuracy.rename(index ={0:'ACCURACY'}, inplace = True)
results = results.append(accuracy)
results = results.fillna('')
results.to_csv('Accuracy_Matrix_SVM.csv')
#print (time.time()-start)
