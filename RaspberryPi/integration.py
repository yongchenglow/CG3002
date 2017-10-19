import sys
import time
import serial
import csv
import socket
import base64
from Crypto.Cipher import AES
from Crypto import Random

ser = serial.Serial('/dev/ttyS0', 115200)
secret_key = b'leader daryl goh'
actions = ['busdriver', 'frontback', 'jumping', 'jumpingjack', 'sidestep',
           'squatturnclap', 'turnclap', 'wavehands', 'windowcleaner360',
           'windowcleaning', 'logout  ']
i = 0.0

# Packet codes for each packet type
ACK = b"\x00"
NACK = b"\x01"
HELLO = b"\x02"

handshake_flag = True
s = socket.socket()

current= 0
voltage = 0
power = 0

def handshake(handshake_flag):
    print('Checking handshake')
    while handshake_flag:
        time.sleep(1)
        ser.write(HELLO)
        string = ser.read()
        reply = int.from_bytes(string, byteorder='big', signed=True)
        if (reply == 0):
            handshake_flag = False
            ser.write(ACK)
            print('Handshake completed')

def connectServer(ip, port):
    print('Checking TCP socket')
    s.connect((ip,port))
    print('Server connected')

def dataFromArduino():
    global current, voltage, power
    startTime = time.time()
    while True:
        checkSum = 0
        dataList = []
        length = 18
    
        number = ser.read()
        packetNumber = int.from_bytes(number, byteorder='big', signed=True)
        # Read in the other values of the Data Packet
        for i in range(0, length):
            item = int.from_bytes(ser.read(), byteorder='big', signed=True)      # Read in the data send by the Arduino
            dataList.append(item)       # Store the data into a list
            checkSum = checkSum ^ item  # Calculate the checksum by taking XOR
    
        # Read in the Checksum
        dataList.append(int.from_bytes(ser.read(), byteorder='big', signed=True))
        
        if (dataList[length] == checkSum):
            ser.write(ACK)                  # Send ACK to arduino if everything is received
            print('Successful Transmission')
            with open('data.csv','a') as file:
                writer = csv.writer(file)
                data = [dataList[2], dataList[3], dataList[4],
                        dataList[6], dataList[7], dataList[8],
                        dataList[10], dataList[11], dataList[12]]
                writer.writerow(data)
            current = dataList[14]
            voltage = dataList[15]
            power = dataList[16]
        else:
            print('Failed Transmission')
            ser.write(NACK)                  # Send NACK if an kind of error occurs
        
        if (time.time() - startTime > 1):
            dataToServer()
            startTime = time.time()

def dataToServer():
    global i
    
    msg = '#' + actions[int(i)] + '|' + str(current) + '|' + str(voltage) + '|' + str(power) + '|' + repr(i)    
    length = 16 - (len(msg) % 16);
    msg += length * ' '
    
    iv = Random.new().read(AES.block_size)
    cipher = AES.new(secret_key, AES.MODE_CBC, iv)                
    encoded = base64.b64encode(iv + cipher.encrypt(msg))
    s.send(encoded)
            
    i += 1
    if (i == 11):
        print('Connection closed')
        s.close()
        sys.exit()

if len(sys.argv) != 3:
    print('Invalid number of arguments')
    print('python client_pi.py [IP address] [Port]')
    sys.exit()

ip = sys.argv[1]
port = int(sys.argv[2])

handshake(handshake_flag)
connectServer(ip, port)
dataFromArduino()