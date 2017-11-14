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
        self.sock.listen(1)
        self.actions = ['wavehands', 'wavehands', 'wavehands', 'wavehands',
                        'busdriver', 'busdriver', 'busdriver', 'busdriver',
                        'frontback', 'frontback', 'frontback', 'frontback',
                        'sidestep', 'sidestep', 'sidestep', 'sidestep', 
                        'jumping', 'jumping', 'jumping', 'jumping',
                        'jumpingjack', 'jumpingjack', 'jumpingjack', 'jumpingjack',
                        'turnclap', 'turnclap', 'turnclap', 'turnclap',
                        'squatturnclap', 'squatturnclap', 'squatturnclap', 'squatturnclap',
                        'windowcleaning', 'windowcleaning', 'windowcleaning', 'windowcleaning',
                        'windowcleaner360','windowcleaner360','windowcleaner360','windowcleaner360']
        self.indices= [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39]
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
        random.shuffle(self.indices)
 
        self.timer = threading.Timer(self.timeout, self.getAction)
        self.timer.start()
        print ("No actions for 60 seconds to give time to connect")

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
           sys.exit(1) 

        while True:
           data = connection.recv(1024)
           if data:
               try:
                   msg = data.decode()
                   decodedmsg = self.auth.decryptText(msg,self.secret_key)

                   if decodedmsg['action'] == "logout":
                      print("bye bye")
                      connection.close()
                      sys.exit(0)
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
              sys.exit(1) 


    def getAction(self):
        self.timer.cancel()
        if self.no_response: # If no response was sent
            self.logMoveMade("None", 0,0,0,0)
            print("ACTION TIMEOUT")
        
        global x
        global action
        global action_set_time
        
        if (x<40):
          index = self.indices[x]
        else:
          index = 39
        action = self.actions[index]
        x=x+1
        action_set_time = time.time()
#        print("NEW ACTION :: {}".format(action))
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

global x
x=0

my_server = server(ip_addr,port_num)

threading.Thread(target=my_server.start_server).start()

global action

# Create action display window
display_window = Tk()
display_label = Label(display_window, text = str(action))
display_label.config(font=('times', 130, 'bold'))
display_label.pack(expand=True)
display_window.update()

while x<=40: #Display new task
  display_label.config(text=str(x)+":"+str(action))
  display_window.update()
  time.sleep(0.2)

sys.exit(0)

