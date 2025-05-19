import network
import socket
from time import sleep
import machine

class WIFI:
    
    def __init__(self, ssid, pwd):
        #print("Initialising WIFI class for " + ssid)
        self.ssid = ssid
        self.pwd = pwd
        self.wlan = None
        
    def connect(self):
        #Connect to WLAN
        self.wlan = network.WLAN(network.STA_IF)
        self.wlan.active(True)
        self.wlan.connect(self.ssid, self.pwd)
        #print('Waiting for connection...')
        while self.wlan.isconnected() == False:
            #print('Connection status:' + self.wlan.status())
            sleep(1)
        #print(self.wlan.ifconfig())
        return self.wlan.ifconfig()[0] # Return IP address

    def isconnected(self):
        if self.wlan is None:
            return False
        return self.wlan.isconnected() 
    
