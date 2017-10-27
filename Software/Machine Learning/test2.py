import sys
import time
import serial
import csv

from multiprocessing import Process, Queue, Manager
from ctypes import c_char_p
import numpy as np
import pickle
import pandas as pd
from scipy import stats
from sklearn.externals import joblib
from sklearn import preprocessing
import ML_FUNCTIONS as ml

def handshake(handshake_flag):
    ACK = b"\x00"
    HELLO = b"\x02"
    
    while handshake_flag:
        time.sleep(1)                       # 1 second pause timing
        ser.write(HELLO)                    # Send Hello to Arduino
        string = ser.read()                 # Read Arduino's response
        reply = int.from_bytes(string, byteorder='big', signed=True)
        if (reply == 0):                    # Check if the reply is an ACK
            handshake_flag = False
            ser.write(ACK)                  # If true, change flag and ACK
            print('Handshake completed')

def learn(queue):
    print('starting learn')
    
    with open('my_trained_classifier.pkl', 'rb') as fid:
        clf = pickle.load(fid)

    while (flags['logout'] == False):
        if (flags['dataReady'] == True):
            rawData = []
            count = 0
            '''while (count < 3000):
                rawData.append(queue.get())
                count += 1'''
            flags['dataReady'] = False
            flags['takeData'] = True
            
            test = pd.read_csv("/home/pi/Desktop/CG3002/Software/DanceDanceData/data231017/turnclap/turnclap6.csv")
            test = preprocessing.normalize(test)
            test = ml.segment_signal(test, 50) #segmentation to 3d for feature extraction
            feature_list = []
            feature_list = ml.time_features(test,feature_list) #feature extraction and conver to 2d
            
            ##### Applying model to test set #####
            result = clf.predict(feature_list)
            
            result = stats.mode(result) #find the mode in result
            result = np.array(result[0])
            result = str(int(result))
            result = ml.result_output(result)
            print(result)
            '''X = np.array(rawData, dtype=int, copy=True) 
            X = preprocessing.normalize(X) #normalize the dataset
            X = ml.segment_signal(X, 50) #segmentation to 3d for feature extraction
            
            time_feature_list = []
            time_feature_list = ml.time_features(X, time_feature_list) #feature extraction and conver to 2d
            
            ##### Predict #####
            result = clf.predict(X)   
            result = stats.mode(result) #find the mode in result
            result = np.array(result[1])
            result = str(int(result))
            
            result = ml.result_output(result) #output the result as string
            action.value = result
            print(result)'''
            flags['mlReady'] = True

def dataFromArduino(queue):
    print('starting dataFromArduino')
    ACK = b"\x00"
    NACK = b"\x01"
    STOP = b"\x06"

    sampleSize = 3000
    numSample = 0
    cumilativeCurrent = 0
    cumilativeVoltage = 0
    cumilativePower = 0
    energyConsumption = 0
    totalTime = 0
    
    while (flags['logout'] == False):
        if (flags['takeData'] == True):
            checkSum = 0
            dataList = []
            length = 18
    
            number1 = ser.read()
            number2 = ser.read()
            packetNumber = int.from_bytes(number2+number1, byteorder='big', signed=True)
            
            # Read in the other values of the Data Packet
            for i in range(0, length):
                item1 = ser.read()
                item2 = ser.read()
                item = int.from_bytes(item2 + item1, byteorder='big', signed=True)      # Read in the data send by the Arduino
                dataList.append(item)          # Store the data into a list
                checkSum = checkSum ^ item      # Calculate the checksum by taking XOR
    
            # Read in the Checksum
            check1 = ser.read()
            check2 = ser.read()
            dataList.append(int.from_bytes(check2 + check1, byteorder='big', signed=True))
            
            print(dataList) 
            #print(checkSum) 
    
            if (dataList[length] == checkSum):
                ser.write(ACK)                  # Send ACK to arduino if everything is received
                #print('Successful Transmission')
                #print(dataList)                 # For Debugging/Demo purposes
                
                '''with open('data.csv','a') as file:
                    writer = csv.writer(file)
                    data = [dataList[2], dataList[3], dataList[4],
                            dataList[6], dataList[7], dataList[8],
                            dataList[10], dataList[11], dataList[12]]
                    writer.writerow(data)'''
                
                queue.put([numSample,numSample,numSample,numSample,numSample,numSample,numSample,numSample,numSample])
            
                numSample += 1
                cumilativeVoltage += dataList[14]
                cumilativeCurrent += dataList[15]
                cumilativePower += dataList[16]
                totalTime += dataList[17]
                
                print(numSample)
        
            else:
                #print('Transmission Failed')
                print(dataList) 
                ser.write(NACK)                 # Send NACK if an kind of error occurs

            if (numSample == sampleSize):
                flags['dataReady'] = True
                #ser.write(STOP)
                flags['takeData'] = False
                numSample = 0
    
if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyS0',115200)
    queue = Queue()                         # Store real-time data from Arduino
    
    manager = Manager()
    action = manager.Value(c_char_p, "")    # Store action based on ML
    flags = manager.dict({'logout' : False, 'dataReady' : False, 'mlReady' : False, 'takeData' : True})
    
    p1 = Process(target=dataFromArduino, args=(queue,))
    p2 = Process(target=learn, args=(queue,))

    handshake(True)
    p1.start()
    p2.start()

    p2.join()
    while (queue.empty() == False):
        queue.get()
    p1.join()
    sys.exit()