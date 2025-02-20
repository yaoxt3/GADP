from pymodbus.client.sync import ModbusTcpClient

import time
import struct
import sys

class Gripper(object):

    def __init__(self, address, port=502):
        ''' Initializes a new instance.

        :param address: string containing the ip address of the ModbusTcp device (e.g. 192.168.0.10)
        :param port: Integer value of the port number to connect to (usually 502)
        '''
        self.client = ModbusTcpClient(address, port=port)
        print("__init__ parent class successful")
        print(address)
 
        # Modbus output parameter
        self.c_VACUUM               = 0
        self.c_DROP                 = 1
     
 
  


# functions --> handle Modbus write registers
     
    def connect(self):
        if self.client.connect():
            print("connection successful")
        else:
            print("connection failed")
   
    def close(self):
        if self.client.close():
            print("close successful")
        else:
            print("close failed")     


        
    def suction(self,state):
        ''' suction will start with 1 and stop with 0 
        '''     
        if state == "on":
            self.client.write_registers(self.c_VACUUM,1)
            print('suction on')
        elif state == "off":
            self.client.write_registers(self.c_VACUUM,0)
            print('suction off')
        else:
            print("Command not accepted")


    def drop(self,state):
        ''' drop function will start with 1 and stop with 0 
        '''     
        if state == "on":
            self.client.write_registers(self.c_DROP,1)
            print('drop on')
        elif state == "off":
            self.client.write_registers(self.c_DROP,0)
            print('drop off')
        else:
            print("Command not accepted")



    
  

# functions --> handle Modbus read registers




