import serial
import time
import csv

# Set up th serial port
ser = serial.Serial('/dev/ttyS0',9600)

# Declare the Packet Codes for the Packet Type
ACK = b"\x00"
NACK = b"\x01"
HELLO = b"\x02"

# Handshake Flags
handshake_flag = True

# Handshake condition
while handshake_flag:
    time.sleep(1)                       # 1 second pause timing
    ser.write(HELLO)                    # Send Hello to Arduino
    str = ser.readline()                # Read Arduino's response
    reply = int(str)
    if (reply == 0):                    # Check if the reply is an ACK
        handshake_flag = False
        ser.write(ACK)                  # If true, change flag and ACK
        print('Handshake completed')

        
# While Loop to receive data
while True:
    # Variables
    checkSum = 0
    dataList = []
    
    # Read in the other values of the Data Packet
    for i in range(0, 17):
        item = int(ser.readline())      # Read in the data send by the Arduino
        dataList.append(item)           # Store the data into a list
        checkSum = checkSum ^ item      # Calculate the checksum by taking XOR
    
    # Read in the Checksum
    dataList.append(int(ser.readline()))
    
    if (dataList[17] == checkSum):
        print('Succesfull Transmission')
        with open('data.csv','a') as file:
            writer = csv.writer(file)
            data = [dataList[3], dataList[4], dataList[5],
                    dataList[7], dataList[8], dataList[9],
                    dataList[11], dataList[12], dataList[13]]
            writer.writerow(data)
        print(dataList)
        print(checkSum)
        ser.write(ACK)                  # Send ACK to arduino if everything is received
    else:
        ser.write(NACK)                 # Send NACK if an kind of error occurs
        print('fail')
        print(dataList)
        print(checkSum)
