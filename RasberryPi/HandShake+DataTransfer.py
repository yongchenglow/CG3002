import serial
import time

# Set up th serial port
ser = serial.Serial('/dev/ttyACM1',9600)

# Declare the Packet Codes for the Packet Type
ACK = b"\x00"
NACK = b"\x01"
HELLO = b"\x02"

# Handshake Flags
handshake_flag = True

# Handshaek condition
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
    
    # Read the length of the Data Packet
    dataList.append(int(ser.readline()))
    
    # Read in the other values of the Data Packet
    for i in range(1, 17):
        item = int(ser.readline())      # Read in the data send by the Arduino
        dataList.append(item)           # Store the data into a list
        checkSum = checkSum ^ item      # Calculate the checksum by taking XOR
    
    # Read in the Checksum
    dataList.append(int(ser.readline()))
    
    if (dataList[17] == checkSum):
        ser.write(ACK)                  # Send ACK to arduino if everything is received
        print('Succesfull Transmission')
        print(dataList)
        print(checkSum)
    else:
        ser.write(NACK)                 # Send NACK if an kind of error occurs
        print('fail')
        print(dataList)
        print(checkSum)