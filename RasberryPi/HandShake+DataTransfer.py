import serial
import time

ser = serial.Serial('/dev/ttyACM1',9600)
HELLO = b"\x02"
ACK = b"\x00"
handshake_flag = True

while handshake_flag:
    time.sleep(1)
    ser.write(HELLO)
    str = ser.readline()
    reply = int(str)
    if (reply == 0):
        handshake_flag = False
        ser.write(ACK)
        print('Handshake completed')
        
while True:
    checkSum = 0
    dataList = []
    dataList.append(int(ser.readline()))
    
    for i in range(1, 17):
        item = int(ser.readline())
        dataList.append(item)
        checkSum = checkSum ^ item 
    
    dataList.append(int(ser.readline()))
    if (dataList[17] == checkSum):
        print('Succesfull Transmission')
    else:
        print('fail')
        print(dataList)
        print(checkSum)