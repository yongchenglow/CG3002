import sys
import time
import csv
import socket
import base64
import serial
import queue
from Crypto.Cipher import AES
from Crypto import Random

from scipy import signal
import pandas as pd
import numpy as np
import pickle
from scipy import stats
from sklearn import preprocessing, cross_validation, metrics, neighbors
from sklearn.svm import SVC
from sklearn.cross_validation import cross_val_score
from sklearn.metrics import confusion_matrix
from sklearn.externals import joblib
import time
import ML_FUNCTIONS as ml

from multiprocessing import Process

training_label_list = pd.read_csv("y_list.csv")
training_feature_list = pd.read_csv("time_feature_list.csv")

##### Training and Validation #####
X_train, X_test, y_train, y_test = cross_validation.train_test_split(training_feature_list,
                                                   training_label_list, test_size = 0.25)
clf = neighbors.KNeighborsClassifier(n_neighbors = 10)
#clf = SVC()
clf.fit(X_train, y_train)

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

def connectServer(ip, port):
    s.connect((ip,port))
    print('Server connected')
    
def dataToServer(action, voltage, current, power, cumPower):
    msg = '#' + action + '|' + str(current) + '|' + str(voltage) + '|' + str(power) + '|' + str(cumPower)    
    length = 16 - (len(msg) % 16);
    msg += length * ' '
    
    iv = Random.new().read(AES.block_size)
    cipher = AES.new(secret_key, AES.MODE_CBC, iv)                
    encoded = base64.b64encode(iv + cipher.encrypt(msg))
    s.send(encoded)
    if (action == 'logout  '):
        s.close()
        sys.exit()

def learn(X):  
    print('learn')
        
    '''test = pd.read_csv("/home/pi/Desktop/CG3002/Software/DanceDanceData/halloweenData/frontback/frontback3.csv")
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
    print(result)'''
    
    X = preprocessing.normalize(X) #normalize the dataset
    X = ml.segment_signal(X, 50) #segmentation to 3d for feature extraction   
    time_feature_list = []
    time_feature_list = ml.time_features(X, time_feature_list) #feature extraction and conver to 2d
    ##### Predict #####
    result = clf.predict(time_feature_list) 
    print(result)
    result = stats.mode(result) #find the mode in result
    result = np.array(result[0])
    result = str(int(result))   
    result = ml.result_output(result) #output the result as string
    print(result)
    return result

def dataFromArduino():
    ACK = b"\x00"
    NACK = b"\x01"
    
    checkSum = 0
    dataList = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    length = 18
    
    number1 = ser.read()
    number2 = ser.read()
    packetNumber = int.from_bytes(number2+number1, byteorder='big', signed=True)
            
    # Read in the other values of the Data Packet
    for i in range(0, length):
        item1 = ser.read()
        item2 = ser.read()
        item = int.from_bytes(item2 + item1, byteorder='big', signed=True)      # Read in the data send by the Arduino
        dataList[i] = item          # Store the data into a list
        checkSum = checkSum ^ item      # Calculate the checksum by taking XOR
    # Read in the Checksum
    check1 = ser.read()
    check2 = ser.read()
    arduinoChecksum = int.from_bytes(check2 + check1, byteorder='big', signed=True)
            
    #print(dataList) 
    
    if (arduinoChecksum == checkSum):
        ser.write(ACK)                  # Send ACK to arduino if everything is received
        '''buffer.append([dataList[2], dataList[3], dataList[4],
                       dataList[6], dataList[7], dataList[8],
                       dataList[10], dataList[11], dataList[12]])'''
        queue.put([dataList[2], dataList[3], dataList[4],
                   dataList[6], dataList[7], dataList[8],
                   dataList[10], dataList[11], dataList[12]])
        #print('Successful Transmission')
        sampleSize = 1
        cumVoltage = dataList[14]
        cumCurrent = dataList[15]
        cumPower = dataList[16]
        
    else:
        print('Transmission Failed')
        sampleSize = 1
        cumVoltage = 0
        cumCurrent = 0
        cumPower = 0
        ser.write(NACK)                 # Send NACK if an kind of error occurs
    
    return {'buffer': queue, 'size' : sampleSize, 'cumVoltage': cumVoltage, 'cumCurrent': cumCurrent, 'cumPower': cumPower}

if len(sys.argv) != 3:
    print('Invalid number of arguments')
    print('python client_pi.py [IP address] [Port]')
    sys.exit()
    
ip = sys.argv[1]
port = int(sys.argv[2])

ser = serial.Serial('/dev/ttyS0', 115200)
secret_key = b'leader daryl goh'
s = socket.socket()
buffer = []
queue = queue.Queue(3000)
size = 0
cumulativeCurrent = 0
cumulativeVoltage = 0
cumulativePower = 0
energyConsumption = 0
totalTime = 0
    
handshake(True)
connectServer(ip, port)
time.sleep(50)
print('Start moving!')
while (True):
    results = dataFromArduino()
    queue = results['buffer']
    size += results['size']
    cumulativeCurrent += results['cumCurrent']
    cumulativeVoltage += results['cumVoltage']
    cumulativePower += results['cumPower']
    print(size)
    if (size == 3000):
        rawData = []
        while (size > 0):
            rawData.append(queue.get())
            size -= 1
        X = np.array(rawData)
        action = learn(X)
        
        voltage = round((cumulativeVoltage/3000)/1000,2)
        current = round((cumulativeCurrent/3000)/1000,2)
        power = round((cumulativePower/3000)/1000,2)
        cumPower = round(((cumulativePower/3000)/1000)*(totalTime/1000/60/60),2)
        dataToServer(action, voltage, current, power, cumPower)
        time.sleep(3)
        sys.exit()