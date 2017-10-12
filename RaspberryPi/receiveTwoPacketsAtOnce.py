import serial
import time
import csv
#from client_pi import save

# Set up th serial port
ser = serial.Serial('/dev/ttyS0',115200)

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
    string = ser.read()             # Read Arduino's response
    reply = int.from_bytes(string, byteorder='big', signed=True)
    if (reply == 0):                    # Check if the reply is an ACK
        handshake_flag = False
        ser.write(ACK)                  # If true, change flag and ACK
        print('Handshake completed')

        
# While Loop to receive data
while True:
    # Variables
    checkSum = 0
    dataList = []
    
    numberOfData = int.from_bytes(ser.read(), byteorder='big', signed=True)
    length = numberOfData*16+2
    #print(numberOfData)
    #print(length)
    # Read in the other values of the Data Packet
    for i in range(0, length):
        item = int.from_bytes(ser.read(), byteorder='big', signed=True)      # Read in the data send by the Arduino
        dataList.append(item)           # Store the data into a list
        checkSum = checkSum ^ item      # Calculate the checksum by taking XOR
    
    # Read in the Checksum
    dataList.append(int.from_bytes(ser.read(), byteorder='big', signed=True))
    
    #print(dataList) 
    #print(checkSum) 
    
    if (dataList[length] == checkSum):
        ser.write(ACK)                  # Send ACK to arduino if everything is received
        print('Successful Transmission')
        #print(dataList)                 # For Debugging/Demo purposes
        #print(checkSum)                 # For Debugging/Demo purposes
        with open('data.csv','a') as file:
            writer = csv.writer(file)
            data = [dataList[2], dataList[3], dataList[4],
                    dataList[6], dataList[7], dataList[8],
                    dataList[10], dataList[11], dataList[12]]
            writer.writerow(data)
        #save(dataList[14], dataList[15], dataList[16])
        
        if(numberOfData > 1):
            with open('data.csv','a') as file:
                writer = csv.writer(file)
                data = [dataList[19], dataList[20], dataList[21],
                        dataList[23], dataList[24], dataList[25],
                        dataList[27], dataList[28], dataList[29]]
                writer.writerow(data)
        #save(dataList[31], dataList[32], dataList[33])
        
    else:
        print('Transmission Failed')
        ser.write(NACK)                 # Send NACK if an kind of error occurs