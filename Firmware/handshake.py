import serial
import time

ser = serial.Serial('/dev/ttyACM1',9600)
HELLO = "\x02"
ACK = "\x00"
handshake_flag = True

while handshake_flag:
    time.sleep(1)
    ser.write(HELLO)
    str = ser.readline()
    reply = int(str)
    if (reply == 0):
        handshake_flag = False
        ser.write(ACK)
        print "Handshake completed"