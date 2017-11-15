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

# Data Structure
waitingData = []
unacknowledgePackets = []
windowSize = 0
expectedPacket = 0
unacknowledgePacket = 0
halt = 0

# Constants
sampleSize = 3000
numberOfSamples = 0
cumilativeCurrent = 0
cumilativeVoltage = 0
cumilativePower = 0
totalTime = 0
energyConsumption = 0

def writeData(dataReceived):
    global numberOfSamples
    global cumilativeVoltage
    global cumilativeCurrent
    global cumilativePower
    global totalTime
    with open('data.csv','a') as file:
        writer = csv.writer(file)
        dataToStore = [dataReceived[2], dataReceived[3], dataReceived[4],
                dataReceived[6], dataReceived[7], dataReceived[8],
                dataReceived[10], dataReceived[11], dataReceived[12]]
        writer.writerow(dataToStore)
    numberOfSamples += 1
    cumilativeVoltage += dataReceived[14]
    cumilativeCurrent += dataReceived[15]
    cumilativePower += dataReceived[16]
    totalTime += dataReceived[17]
    
# Handshake condition
while handshake_flag:
    time.sleep(1)                       # 1 second pause timing
    ser.write(HELLO)                    # Send Hello to Arduino
    
    string1 = ser.read()                # Read Arduino's response
    string2 = ser.read()                # Read Arduino's response
    reply = int.from_bytes(string2 + string1, byteorder='big', signed=True)
    
    size1 = ser.read()                # Read Arduino's response
    size2 = ser.read()                # Read Arduino's response
    windowSize = int.from_bytes(size2 + size1, byteorder='big', signed=True)
    if (reply == 0):                    # Check if the reply is an ACK
        handshake_flag = False
        ser.write(ACK)                  # If true, change flag and ACK
        print('Handshake completed')
        print('Window Size = ' + str(windowSize))

        
# While Loop to receive data
while numberOfSamples < sampleSize:
    # Variables
    checkSum = 0
    dataReceived = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    length = 17
    
    # Packet number (Depending on window)
    number1 = ser.read()
    number2 = ser.read()
    packetNumber = int.from_bytes(number2+number1, byteorder='big', signed=True)
    
    code1 = ser.read()
    code2 = ser.read()
    packetCode = int.from_bytes(code2+code1, byteorder='big', signed=True)
    
    print(str(packetNumber))
    print(str(packetCode))
    
    if((packetNumber != expectedPacket or (packetNumber != unacknowledgePacket and halt == 1)) and packetCode != 4):
        print('Out of sync')
        ser.write(NACK)
        ser.write(expectedPacket.to_bytes(1, byteorder='big'))                 # Send NACK if an kind of error occurs
        halt = 1
        receiver = 0
        # discaerd all the values
        while True:
            line1 = ser.read()
            line2 = ser.read()
            receive = int.from_bytes(code2+code1, byteorder='big', signed=True)
            if(receive == 8):
                discard1 = ser.read()
                discard2 = ser.read()
                break
    else:
        checkSum = checkSum ^ packetCode
        # Read in the other values of the Data Packet
        for i in range(0, length):
            item1 = ser.read()
            item2 = ser.read()
            item = int.from_bytes(item2 + item1, byteorder='big', signed=True)      # Read in the data send by the Arduino
            dataReceived[i] = (item)          # Store the data into a list
            checkSum = checkSum ^ item      # Calculate the checksum by taking XOR
        
        # Read in the Checksum
        check1 = ser.read()
        check2 = ser.read()
        dataReceived[length] = int.from_bytes(check2 + check1, byteorder='big', signed=True)
        
        print(dataReceived) 
        print(checkSum) 
        
        if (dataReceived[length] == checkSum):
            if(halt == 0):
                ser.write(ACK)
                ser.write(packetNumber.to_bytes(1, byteorder='big'))                     # Send ACK to arduino if everything is received
                print('Successful Transmission')
                #print(dataList)                                # For Debugging/Demo purposes
                #print(checkSum)                                # For Debugging/Demo purposes
                writeData(dataReceived)
                expectedPacket = (expectedPacket + 1) % windowSize
                
            else:
                # if the received packet the unacknowledgePacket
                if(unacknowledgePackets[0] == packetNumber):
                    print('Successful Transmission')
                    writeData(dataReceived)
                    unacknowledgePackets[0].remove()
                    
                    # Write the waiting data into the list
                    while(len(waitingData) != 0 and waitingData[0].dataReceived[0] < unacknowledgePackets[0]):
                        lastPacketNumberPop = waitingData[0][0]
                        writeData(waitingData.pop(0))
                    
                    # Change the halt flag
                    if (len(unacknowledgePackets) == 0):
                        ser.write(ACK)
                        ser.write(expectedPacket.to_bytes(1, byteorder='big'))
                        halt = 0
                    else:
                        ser.write(ACK)
                        ser.write(lastPacketNumberPop.to_bytes(1, byteorder='big'))
                        unacknowledgePacket = unacknowledgePackets[0]
                else:
                    # Else put it into a quite
                    waitingData.append(dataReceived)
                    expectedPacket = (expectedPacket + 1) % windowSize
            

            
        else:
            print('Transmission Failed')
            unacknowledgePackets.append(packetNumber)
            ser.write(NACK)
            ser.write(packetNumber.to_bytes(1, byteorder='big'))                 # Send NACK if an kind of error occurs
            halt = 1
            unacknowledgePacket = packetNumber
        
print('Average Voltage: ', round((cumilativeVoltage/numberOfSamples),2), 'mV')
print('Average Current: ', round((cumilativeCurrent/numberOfSamples),2), 'mA')
print('Average Power: ', round((cumilativePower/numberOfSamples),2), 'W')
energyConsumption = round((cumilativePower/numberOfSamples)*(totalTime/1000/60/60),2)
print('Energy Comsumption: ', energyConsumption, 'kWh')