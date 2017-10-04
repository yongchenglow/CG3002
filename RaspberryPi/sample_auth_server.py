import socket
import sys
import server_auth
import random
import time
import threading
import os
import pandas as pd

class server:
    def __init__(self, ip_addr, port_num):
        global action
        global action_set_time

        # init server
        self.auth = server_auth.server_auth()
        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Bind the socket to the port
        server_address = (ip_addr, port_num)
        print('starting up on %s port %s' % server_address, file=sys.stderr)
        self.sock.bind(server_address)
        # Listen for incoming connections
        self.sock.listen(3)
        self.actions = ['busdriver', 'frontback', 'jumping', 'jumpingjack', 'sidestep',
                'squatturnclap', 'turnclap', 'wavehands', 'windowcleaner360',
                'windowcleaning']
        self.filename = "logServer.csv"
        self.columns = ['timestamp', 'action',
                'goal', 'time_delta',
                'correct', 'voltage', 'current', 'power', 'cumpower']
        self.df = pd.DataFrame(columns=self.columns)
        self.df = self.df.set_index('timestamp')
        action = None
        action_set_time = None
        self.timeout = 60
        self.getAction()

        while True:
            # Wait for a connection
            print('waiting for a connection', file=sys.stderr)
            connection, client_address = self.sock.accept()
            self.secret_key = input("Enter the secret key: ")

            print('connection from', client_address, file=sys.stderr)

            if len(self.secret_key) == 16 or len(self.secret_key) == 24 or len(self.secret_key) == 32:
              pass
            else:
              print ("AES key must be either 16, 24, or 32 bytes long")
              break

            # Receive the data in small chunks and retransmit it
            while True:
                data = connection.recv(1024)
                if data:
                        try:
                            msg = data.decode()
                            decodedmsg = self.auth.decryptText(msg,self.secret_key)
                            if decodedmsg['action'] == "logout  ":
                                print("bye bye")
                            elif len(decodedmsg['action']) == 0:
                                pass

                            self.logMoveMade(decodedmsg['action'], decodedmsg['voltage'],decodedmsg['current'],decodedmsg['power'],decodedmsg['cumpower'])
                            print("{} :: {} :: {} :: {} :: {}".format(decodedmsg['action'], decodedmsg['voltage'],decodedmsg['current'],decodedmsg['power'],decodedmsg['cumpower']))
                                
                        except Exception as e:
                            print(e)
                else:
                     print('no more data from', client_address, file=sys.stderr)
                     connection.close()
                     break


    def getAction(self):
        global action
        global action_set_time
        action = random.choice(self.actions)
        
        action_set_time = time.time()
        print("NEW ACTION :: {}".format(action))
        threading.Timer(self.timeout, self.getAction).start()       

    def logMoveMade(self, action_made, voltage, current, power, cumpower):
        file = "log.csv";
        if not os.path.isfile(file):
            with open(file, 'w') as f:
                self.df.to_csv(f)
        with open(file, 'a') as f:
            data = {}
            data['timestamp'] = time.time()
            data['action'] = action_made
            data['goal'] = action
            data['time_delta'] = data['timestamp'] - action_set_time
            data['voltage'] = voltage
            data['current'] = current
            data['power'] = power
            data['cumpower'] = cumpower
            data['correct'] = (action == action_made)
            self.df = pd.DataFrame(data, index=[0])[self.columns].set_index('timestamp')
            self.df.to_csv(f, header=False)

if len(sys.argv) != 3:
    print('Invalid number of arguments')
    print('python server.py [IP address] [Port]')
    sys.exit()

ip_addr = sys.argv[1]
port_num = int(sys.argv[2])

#IP address = 'x.x.x.x'
#Port = 8888


my_server = server(ip_addr,port_num)

