from tkinter import *
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
        self.actions = ['wavehands', 'busdriver', 'frontback', 'sidestep', 'jumping']
#[                'jumpingjack', 'turnclap', 'squatturnclap', 'windowcleaning', 'windowcleaner360']
        self.filename = "logServer.csv"
        self.columns = ['timestamp', 'action',
                'goal', 'time_delta',
                'correct', 'voltage', 'current', 'power', 'cumpower']
        self.df = pd.DataFrame(columns=self.columns)
        self.df = self.df.set_index('timestamp')
        action = None
        action_set_time = None
        self.timeout = 60
        self.no_response = False
        
    def start_server(self):
        self.timer = threading.Timer(self.timeout, self.getAction)
        self.timer.start()
        print ("No actions for 60 seconds to give time to connect")
        while True:
            # Wait for a connection
            print('waiting for a connection', file=sys.stderr)
            connection, client_address = self.sock.accept()
            #self.secret_key = input("Enter the secret key: ")
            print("Enter the secret key: ")
            self.secret_key = sys.stdin.readline().strip()

            print('connection from', client_address, file=sys.stderr)

            if len(self.secret_key) == 16 or len(self.secret_key) == 24 or len(self.secret_key) == 32:
              pass
            else:
              print ("AES key must be either 16, 24, or 32 bytes long")
              break

            # Receive the data in small chunks and retransmit it
            # while True: Change to 20 actions
            for x in range(21):
                data = connection.recv(1024)
                if data:
                        try:
                            msg = data.decode()
                            decodedmsg = self.auth.decryptText(msg,self.secret_key)
                            if decodedmsg['action'] == "logout  ":
                                print("bye bye")
                            elif len(decodedmsg['action']) == 0:
                                pass
                            elif action == None: # Ignore if no action has been set yet
                                pass
                            else:   # If action is available log it, and then...
                                self.no_response = False
                                self.logMoveMade(decodedmsg['action'], decodedmsg['voltage'],decodedmsg['current'],decodedmsg['power'],decodedmsg['cumpower'])
                                print("{} :: {} :: {} :: {} :: {}".format(decodedmsg['action'], decodedmsg['voltage'],decodedmsg['current'],decodedmsg['power'],decodedmsg['cumpower']))
                                self.getAction() # ...get new action
                        except Exception as e:
                            print(e)
                else:
                     print('no more data from', client_address, file=sys.stderr)
                     connection.close()
                     break


    def getAction(self):
        self.timer.cancel()
        if self.no_response: # If no response was sent
            self.logMoveMade("None", 0,0,0,0)
            print("ACTION TIMEOUT")
        
        global action
        global action_set_time
        action = random.choice(self.actions)
        action_set_time = time.time()
        print("NEW ACTION :: {}".format(action))
        self.timer = threading.Timer(self.timeout, self.getAction)
        self.no_response = True
        self.timer.start()

    def logMoveMade(self, action_made, voltage, current, power, cumpower):
        file = "log"+str(groupID)+".csv";
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

if len(sys.argv) != 4:
    print('Invalid number of arguments')
    print('python server.py [IP address] [Port] [groupID]')
    sys.exit()

ip_addr = sys.argv[1]
port_num = int(sys.argv[2])
groupID = sys.argv[3]

#IP address = 'x.x.x.x'
#Port = 8888


my_server = server(ip_addr,port_num)

threading.Thread(target=my_server.start_server).start()

global action

# Create action display window
display_window = Tk()
display_label = Label(display_window, text = str(action))
display_label.config(font=('times', 150, 'bold'))
display_label.pack(expand=True)
display_window.update()

while True: #Display new task
    display_label.config(text=str(action))
    display_window.update()
    time.sleep(0.2)
