import socket
import sys

def read(port):
        s = socket.socket()
        host = '192.168.0.105'
        s.connect((host,port))
        try:
            print 'connection done'
            s.send(b'Testing')
            s.close()
        except socket.error, msg:
            sys.stderr.write('error %s'%msg[1])
            s.close()
            print 'close'
            sys.exit(2)

if __name__ == '__main__':
    port = 58198
    print 'hey, checking TCP socket'
    read(port)