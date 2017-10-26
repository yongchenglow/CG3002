import sys
import time
import socket
import base64
from Crypto.Cipher import AES
from Crypto import Random

from multiprocessing import Process, Queue, Manager
from ctypes import c_char_p
import numpy as np

from scipy import stats
from sklearn.externals import joblib
from sklearn import preprocessing
import ML_FUNCTIONS as ml

def learn(queue):
    with open('trained_model.pkl') as input:
        clf = joblib.load(input) # load model
    i = 0

    while (flags['logout'] == False):
        if (flags['dataReady'] == True):
            rawData = []
            count = 0
            while (count < 3000):
                rawData.append(queue.get())
                count += 1

            #X = np.array(rawData)
            X = np.array(rawData, dtype=int, copy=True) 
            X = preprocessing.normalize(X) #normalize the dataset
            X = ml.segment_signal(X, 50) #segmentation to 3d for feature extraction
            
            time_feature_list = []
            time_feature_list = ml.time_features(X, time_feature_list) #feature extraction and conver to 2d
            
            ##### Predict #####
            result = clf.predict(time_feature_list)   
            result = stats.mode(result) #find the mode in result
            result = np.array(result[0])
            result = str(int(result))
            
            result = ml.result_output(result) #output the result as string
            action.value = result
            if (i == 20):
                action.value = 'logout  '
            i += 1
            flags['mlReady'] = True

def toServer(s):
    secret_key = b'leader daryl goh'
    i = 0
    while (flags['logout'] == False):
        if (flags['mlReady']):
            msg = '#' + action.value + '|' + str(i) + '|' + str(i) + '|' + str(i) + '|' + str(i)    
            length = 16 - (len(msg) % 16);
            msg += length * ' '
            print(msg)
    
            iv = Random.new().read(AES.block_size)
            cipher = AES.new(secret_key, AES.MODE_CBC, iv)                
            encoded = base64.b64encode(iv + cipher.encrypt(msg))
            s.send(encoded)
            
            flags['mlReady'] = False
            i += 1
            if (action.value == 'logout  '):
                flags['logout'] = True

def dataFromArduino(queue):
    count = 0
    
    while (flags['logout'] == False & count < 3000):
        queue.put([count, count, count, count, count, count, count, count, count])
        count += 1
        if (count == 3000):
            flags['dataReady'] = True
            count = 0
        else:
            flags['dataReady'] = False
    
if __name__ == '__main__':
    queue = Queue()                         # Store real-time data from Arduino
    s = socket.socket()
    
    manager = Manager()
    action = manager.Value(c_char_p, "")    # Store action based on ML
    flags = manager.dict({'logout' : False, 'dataReady' : False, 'mlReady' : False})
    
    ip = sys.argv[1]
    port = int(sys.argv[2])
    s.connect((ip,port))
    
    p1 = Process(target=dataFromArduino, args=(queue,))
    p2 = Process(target=learn, args=(queue,))
    p3 = Process(target=toServer, args=(s,))
    p1.start()
    p2.start()
    p3.start()

    p2.join()
    p3.join()
    while (queue.empty() == False):
        queue.get()
    p1.join()
    sys.exit()