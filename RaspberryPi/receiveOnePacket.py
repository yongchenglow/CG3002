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

# Constatns
sampleSize = 3000
numberOfSamples = 0
cumilativeCurrent = 0
cumilativeVoltage = 0
cumilativePower = 0
energyConsumption = 0
totalTime = 0


# Handshake condition
while handshake_flag:
    time.sleep(1)                       # 1 second pause timing
    ser.write(HELLO)                    # Send Hello to Arduino
    string = ser.read()                 # Read Arduino's response
    reply = int.from_bytes(string, byteorder='big', signed=True)
    if (reply == 0):                    # Check if the reply is an ACK
        handshake_flag = False
        ser.write(ACK)                  # If true, change flag and ACK
        print('Handshake completed')

        
# While Loop to receive data
while numberOfSamples < sampleSize:
    # Variables
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
            
        numberOfSamples += 1
        cumilativeVoltage += dataList[14]
        cumilativeCurrent += dataList[15]
        cumilativePower += dataList[16]
        totalTime += dataList[17]
        
    else:
        print('Transmission Failed')
        ser.write(NACK)                 # Send NACK if an kind of error occurs
        
print('Average Voltage: ', round((cumilativeVoltage/numberOfSamples),2), 'mV')
print('Average Current: ', round((cumilativeCurrent/numberOfSamples),2), 'mA')
print('Average Power: ', round((cumilativePower/numberOfSamples),2), 'W')
energyConsumption = round((cumilativePower/numberOfSamples)*(totalTime/1000/60/60),2)
print('Energy Comsumption: ', energyConsumption, 'kWh')