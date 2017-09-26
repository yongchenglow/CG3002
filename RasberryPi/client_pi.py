import socket
import sys
from Crypto.Cipher import AES
from Crypto import Random
import base64
import time

secret_key = b'leader daryl goh'
actions = ['busdriver', 'frontback', 'jumping', 'jumpingjack', 'sidestep',
           'squatturnclap', 'turnclap', 'wavehands', 'windowcleaner360',
           'windowcleaning', 'logout  ']

def read(port):
        i = 0
        s = socket.socket()
        host = '10.213.28.223'
        s.connect((host,port))
        print('connected')
        time.sleep(4)
        
        while True:
            data = str(i)
            msg = '#' + actions[i] + '|' + data + '|' + data + '|' + data + '|' + data        
            length = 16 - (len(msg) % 16);
            msg += length * ' '
            
            iv = Random.new().read(AES.block_size)
            cipher = AES.new(secret_key, AES.MODE_CBC, iv)                
            encoded = base64.b64encode(iv + cipher.encrypt(msg))
            s.send(encoded)
            
            i += 1
            if (i == 11):
                s.close()
                sys.exit()
            else:
                time.sleep(1)

if __name__ == '__main__':
    port = 58198
    print('Checking TCP socket')
    read(port)