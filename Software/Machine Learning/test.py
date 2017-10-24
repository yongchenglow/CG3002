import sys

from multiprocessing import Process, Queue, Value, Lock
import time
import pandas as pd
import numpy as np

from scipy import signal
from scipy import stats
import pandas as pd
import numpy as np
from sklearn.svm import SVC
from sklearn.externals import joblib
from sklearn import preprocessing
import ML_FUNCTIONS as ml

def handshake(handshake_flag):
    print('Checking handshake')
    while (handshake_flag):
        handshake_flag = False
    print('Handshake completed')

def learn(queue):
    print('Running learn')
    clf = joblib.load('trained_model.pkl') # load model
    start = time.time()
    learnflag = False
    
    while (time.time() - start < 3):
        pass
        
    rawData = []
    count = 0
    while (count < 3000):
        rawData.append(queue.get())
        count += 1
    learnflag = True
    
    print(rawData)

    while (learnflag):
        print('tryingngg')
        ##### buffer #####
        bufFlag = 1
        if(bufFlag == 1):               #begin preprocessing to predictive analysis
            X = np.array(rawData)
            X = pd.DataFrame(rawData)
            #X = preprocessing.normalize(X) #normalize the dataset
            X = ml.segment_signal(X, 50) #segmentation to 3d for feature extraction
            
            time_feature_list = []
            time_feature_list = ml.time_features(X, time_feature_list) #feature extraction and conver to 2d

            ##### Predict #####
            result = clf.predict(X)   
            result = stats.mode(result) #find the mode in result
            result = ml.result_output(result) #output the result as string
            bufFlag = 0  #reset flag to take in next dataset
            print(result)
        learnflag = False
    print('Stopping learn')

def dataFromArduino(queue):
    print('Running dataFromArduino')
    count = 0
    
    while (count < 3000):
        queue.put([count, count])
        count += 1
        
    print('Stopping dataFromArduino')
    
if __name__ == '__main__':
    queue = Queue()
    
    handshake(True)
    p1 = Process(target=dataFromArduino, args=(queue,))
    p1.start()
    p1.join()

    p2 = Process(target=learn, args=(queue,))
    p2.start()
    p2.join()
    sys.exit()