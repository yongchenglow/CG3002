import serial
import time

ser = serial.Serial('/dev/ttyS0',9600)
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