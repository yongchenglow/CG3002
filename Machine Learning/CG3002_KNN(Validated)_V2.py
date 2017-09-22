# -*- coding: utf-8 -*-
"""
Created on Sat Sep 16 13:27:26 2017

@author: Daryl
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn import preprocessing, cross_validation, neighbors
from sklearn.neighbors import NearestNeighbors
from sklearn import metrics
from sklearn.preprocessing import label_binarize
from sklearn.cross_validation import cross_val_score

df = pd.read_csv('cg3002_train.csv')
y = pd.DataFrame(df['activity_label'])

le = preprocessing.LabelEncoder()
le.fit(df['activity_label'])
label = list(le.classes_)
df['activity_label'] = le.transform(df['activity_label'])
y = np.array([label])

X = np.array(df.drop(['activity_label', 'volunteer_id'], 1)) #removing volunteer_id n labels
X = preprocessing.scale(X) #normalize the dataset

y = np.array(df['activity_label']) #dataframe for labels



X_train, X_test, y_train, y_test = cross_validation.train_test_split(X,
                                                           y, test_size = 0.4)


clf = neighbors.KNeighborsClassifier()
clf.fit(X_train, y_train)
accuracy_rate_1 = clf.score(X_test, y_test)
validate_score = cross_val_score(clf, X, y, cv= 5)

y_predict = clf.predict(X_test)
matrix = metrics.confusion_matrix(y_test, y_predict)
accuracy_rate_2 = metrics.accuracy_score(y_test, y_predict)
sensitivity_rate = metrics.recall_score(y_test, y_predict, average= 'macro')
precision_rate = metrics.precision_score(y_test, y_predict, average='macro')
f1_score = metrics.f1_score(y_test, y_predict, average= 'macro')

walkingN_walkingN = matrix[0,0]
walkingN_walkingUp = matrix[1,0]
walkingN_sitting = matrix[2,0]
walkingUp_walkingUp = matrix[1,1]
walkingUp_walkingN = matrix[1,0]
walkingUp_sitting = matrix[1,2]
sitting_sitting = matrix[2,2]
sitting_walkingN = matrix[2,0]
sitting_walkingUp = matrix[2,1]

print(walkingN_walkingN)
print(walkingUp_walkingUp)
print(sitting_sitting)
print("")
print(accuracy_rate_1)
print (validate_score.mean())
print(accuracy_rate_2)
print("")
print(matrix)
print(sensitivity_rate)
print(precision_rate)
print(f1_score)