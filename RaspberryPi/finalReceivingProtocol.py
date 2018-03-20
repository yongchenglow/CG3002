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

# Constants
SAMPLE_SIZE = 3000

# Global Variables
handshake_flag = True
numberOfSamples = 0
cumilativeCurrent = 0
cumilativeVoltage = 0
cumilativePower = 0
energyConsumption = 0
totalTime = 0

# Handshake condition
while handshake_flag:
    time.sleep(1)                                                 # 1 second pause timing
    ser.write(HELLO)                                              # Send Hello to Arduino
    string = ser.read()                                           # Read Arduino's response
    reply = int.from_bytes(string, byteorder='big', signed=True)
    if (reply == 0):                                              # Check if the reply is an ACK
        handshake_flag = False
        ser.write(ACK)                                            # If true, change flag and ACK
        print('Handshake completed')

        
# While Loop to receive data
while numberOfSamples < SAMPLE_SIZE:
    # Variables
    checkSum = 0
    dataList = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    length = 18

    # Read in the Packet Code    
    code1 = ser.read()
    code2 = ser.read()
    packetCode = int.from_bytes(code2+code1, byteorder='big', signed=True)
    
    # Read in the left hand accelerometer code
    id1 = ser.read()
    id2 = ser.read()
    leftHandAccelerometerCode = int.from_bytes(id2+id1, byteorder='big', signed=True)
    
    if(packetCode != 4 and leftHandAccelerometerCode != 0):
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
        dataList[1] = leftHandAccelerometerCode
    
    checkSum = checkSum ^ dataList[0]
    checkSum = checkSum ^ dataList[1]
   
    # Read the other values of the Data Packet
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
    
    # Print Statements for Debugging
    #print(dataList) 
    #print(checkSum) 
    
    if (dataList[length] == checkSum):
        ser.write(ACK)
        print('Successful Transmission')
        with open('data.csv','a') as file:
            writer = csv.writer(file)
            data = [dataList[2], dataList[3], dataList[4],
                    dataList[6], dataList[7], dataList[8],
                    dataList[10], dataList[11], dataList[12]]
            writer.writerow(data)
            
        numberOfSamples += 1
        cumilativeVoltage += dataList[14]
        cumilativeCurrent += dataList[15]
        cumilativePower += dataList[16]
        totalTime += dataList[17]
        
    else:
        print('Transmission Failed')
        ser.write(NACK)
        
print('Average Voltage: ', round((cumilativeVoltage/numberOfSamples)/1000,2), 'V')
print('Average Current: ', round((cumilativeCurrent/numberOfSamples)/1000,2), 'A')
print('Average Power: ', round((cumilativePower/numberOfSamples)/1000,2), 'W')
energyConsumption = round(((cumilativePower/numberOfSamples)/1000)*(totalTime/1000/60/60),2)
print('Energy Comsumption: ', energyConsumption, 'Wh')