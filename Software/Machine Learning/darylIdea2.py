import sys
import time
import socket
import base64
import serial
import queue
import csv
from Crypto.Cipher import AES
from Crypto import Random

import pandas as pd
import numpy as np
from scipy import stats
from sklearn import preprocessing, cross_validation, metrics, neighbors
from sklearn.svm import SVC
from sklearn.cross_validation import cross_val_score
from sklearn.metrics import confusion_matrix
from sklearn.externals import joblib
import ML_FUNCTIONS as ml

training_label_list = pd.read_csv("fever_y_list2.csv")
training_feature_list = pd.read_csv("fever_feature_list2.csv")

##### Training and Validation ####
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
    if (action == 'final'):
        action = 'logout'
    msg = '#' + action + '|' + str(voltage) + '|' + str(current) + '|' + str(power) + '|' + str(cumPower)    
    length = 16 - (len(msg) % 16);
    msg += length * ' '
    
    iv = Random.new().read(AES.block_size)
    cipher = AES.new(secret_key, AES.MODE_CBC, iv)                
    encoded = base64.b64encode(iv + cipher.encrypt(msg))
    s.send(encoded)
    if (action == 'logout'):
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
    X = ml.segment_signal(X, 100) #segmentation to 3d for feature extraction   
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
    
    code1 = ser.read()
    code2 = ser.read()
    packetCode = int.from_bytes(code2+code1, byteorder='big', signed=True)
    
    id1 = ser.read()
    id2 = ser.read()
    accelerometer1code = int.from_bytes(id2+id1, byteorder='big', signed=True)
    
    if(packetCode != 4 and accelerometer1code != 0):
        print('Out of sync')
        ser.write(NACK)
        
        # discard all the values
        prev = 0
        while True:
            discard1 = ser.read()
            discard2 = ser.read()
            receive = int.from_bytes(discard2+discard1, byteorder='big', signed=True)
            if(receive == 0 and prev == 4):
                dataList[0] = prev
                dataList[1] = receive
                break
            else:
                prev = receive
    else:
        dataList[0] = packetCode
        dataList[1] = accelerometer1code
    
    checkSum = checkSum ^ dataList[0]
    checkSum = checkSum ^ dataList[1]
    # Read in the other values of the Data Packet
    for i in range(2, length):
        item1 = ser.read()
        item2 = ser.read()
        item = int.from_bytes(item2 + item1, byteorder='big', signed=True)      # Read in the data send by the Arduino
        dataList[i] = item                                                      # Store the data into a list
        checkSum = checkSum ^ item                                              # Calculate the checksum by taking XOR
    
    # Read in the Checksum
    check1 = ser.read()
    check2 = ser.read()
    dataList[length] = (int.from_bytes(check2 + check1, byteorder='big', signed=True))
    
    #print(dataList) 
    #print(checkSum) 
    
    if (dataList[length] == checkSum):
        ser.write(ACK)                  # Send ACK to arduino if everything is received
        #print('Successful Transmission')
        queue.put([dataList[2], dataList[3], dataList[4],
                   dataList[6], dataList[7], dataList[8],
                   dataList[10], dataList[11], dataList[12]])
            
        sampleSize = 1
        cumVoltage = dataList[14]
        cumCurrent = dataList[15]
        cumPower = dataList[16]
        totalTime = dataList[17]
        
    else:
        sampleSize = 0
        cumVoltage = 0
        cumCurrent = 0
        cumPower = 0
        totalTime = 0
        print('Transmission Failed')
        ser.write(NACK)                 # Send NACK if an kind of error occurs
        
    return {'buffer': queue, 'size' : sampleSize, 'cumVoltage': cumVoltage, 'cumCurrent': cumCurrent, 'cumPower': cumPower, 'totalTime': totalTime}

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
totalRunTime = 0
div = 0
    
handshake(True)
connectServer(ip, port)
time.sleep(50)
print('Start moving!')
start = time.time()
while (True):
    results = dataFromArduino()
    queue = results['buffer']
    size += results['size']
    cumulativeCurrent += results['cumCurrent']
    cumulativeVoltage += results['cumVoltage']
    cumulativePower += results['cumPower']
    totalTime += results['totalTime']
    if(size % 100 == 0):
        print(size)
        
    if (size == 1000):
        #print(size)
        rawData = []
        while (size > 0):
            rawData.append(queue.get())
            size -= 1
        X = np.array(rawData)
        action = learn(X)
        totalRunTime += 1
        div = totalRunTime * 1000
        
        voltage = round((cumulativeVoltage/div)/1000,3)
        current = round((cumulativeCurrent/div)/1000,3)
        power = round((cumulativePower/div)/1000,3)
        cumPower = round(((cumulativePower/div)/1000)*(totalTime/1000/60/60),5)
        dataToServer(action, voltage, current, power, cumPower)
        print(time.time() - start)
        start = time.time()
        time.sleep(3)
        #sys.exit()