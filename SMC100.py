"""

This code was a adption of the original work done by Steve Bian that can be found in:
https://github.com/freespace/pySMC100 

The orginal code was writen in python 2.X so I need to update and correct the diferences.

Also with the manifacture User manuel of the SMC100:
https://www.newport.com/mam/celum/celum_assets/resources/SMC100CC_and_SMC100PP_-_User_Manual.pdf?3

It was expanse the amount of comands avaiable to the user.

Author of this version: Pedro Rossa 

"""

'''
Motors:
    Move a given number of steps                            DONE
    Set current position to zero                            DONE
    Get current position                                    DONE

'''
# ref: https://files.xisupport.com/other_files/JupyterNotebook/Standa_8SMC5_USB_Python_tutorial.html
import pathlib
import os
import time
import json

from tango import AttrQuality, AttrWriteType, DevState, DispLevel, AttReqType, Database
from tango.server import Device, attribute, command
from tango.server import class_property, device_property


import SMC100_Lib 

class SMC100(Device):
    Controllers = {}

    host = device_property(dtype=str, default_value="localhost")
    port = class_property(dtype=int, default_value=10000)
    #@command(dtype_in=float, dtype_out=str)
    #cmd_list = { 'move_calibrat' : [  [float , "Number" ], [ str, "Number * 2" ] ] }

    def init_device(self):
        super().init_device()
        self.info_stream(f"Power supply connection details: {self.host}:{self.port}")
        self.set_state(DevState.ON)
		# Devices search

        self.set_status("Standa Motor is ON")
        self.info_stream("\r Standa Motor is ON \r")



    def delete_device(self):
        return

    
    
        
    '''
        userInfoController =  {
                                  "Name"                  : <user_name_given_on Connect>,
                                  "COM"                   : 0,
                                  "Number_of_controllers" : 3
                              }
    '''

    
    @command(dtype_in=str,dtype_out=str)  
    def ConnectCamera(self,userInfoController):
        # print(infoCamera)
        uIC =  json.loads(userInfoController)
        print(uIC)
        try:
          self.Controllers[uIC["Name"]] = SMC100_Lib.init_connection('COM'+str(uIC["COM"]),uIC["Number_of_controllers"])
          return "Controller has been connected successfully COM"+str(uIC["COM"])
        except:
          return "Could not connect to the Controller"



        
if __name__ == "__main__":
    SMC100.run_server()



#  Because we are using the 'with' statement context-manager, disposal has been taken care of.
