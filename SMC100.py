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



from SMC100_Lib import init_connection, SMC100
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
                                  "Number_of_SMC100" : 3
                              }
    '''



    @command(dtype_in=str,dtype_out=str)  
    def ConnectCamera(self,userInfoController):
        # print(infoCamera)
        uIC =  json.loads(userInfoController)
        print(uIC)
        try:
          self.Controllers[uIC["Name"]] = init_connection('COM'+str(uIC["COM"]),uIC["Number_of_controllers"])
          print("All good")
          return "Controller has been connected successfully COM"+str(uIC["COM"])
        except:
          return "Could not connect to the Controller"

    '''
        userInfoP = {
                        "Name" : <user_name_given_on Connect>,
                        "Axis" : 3
                    }
    '''

    
    @command(dtype_in=str,dtype_out=float)  
    def GetPosition(self,userInfoP):
        # print(infoCamera)
        uIP =  json.loads(userInfoP)
        print(uIP)
        return self.Controllers[uIP["Name"]].get_position_mm(uIP["Axis"])
    '''
        userInfoMA = {
                        "Name"              : <user_name_given_on Connect>,
                        "Axis"              : 3,
                        "Position"          : 3,
                        "Wait_to_finish"    ; True
                     }
    '''

    
    @command(dtype_in=str,dtype_out=str)  
    def MoveAbsolute_mm(self,userInfoMA):
        # print(infoCamera)
        uIMA =  json.loads(userInfoMA)
        print(uIMA)
        self.Controllers[uIMA["Name"]].move_absolute_mm(uIMA["Axis"],uIMA["Position"],uIMA["Wait_to_finish"])

        return "Motor is moving to the position"
    
    '''
        userInfoMTA = {
                        "Name"              : <user_name_given_on Connect>,
                        "Axis"              : 3,
                        "Position"          : 3
                     }
    '''

    
    @command(dtype_in=str,dtype_out=float)  
    def GetMotionTimeForAbsoluteMove(self,userInfoMTA):
        uIMTA=  json.loads(userInfoMTA)
        print(uIMTA)
        current_position = float(self.Controllers[uIMTA["Name"]].get_position_mm(uIMTA["Axis"]))
        print("relative movement: ")
        relative_p = abs(uIMTA["Position"]-float(current_position))
        time_for_move = float(self.Controllers[uIMTA["Name"]].get_motion_time_for_relative_move(uIMTA["Axis"],relative_p))
        return time_for_move
    
    '''
        userInfoStop= {
                        "Name"              : <user_name_given_on Connect>,
                        "Axis"              : 3 or [1,2,3]
                     }
    '''

    @command(dtype_in=str,dtype_out=str)  
    def Stop(self,userInfoStop):
        uStop=  json.loads(userInfoStop)
        print(uStop)
        for i in uStop["Axis"]:
            self.Controllers[uStop["Name"]].stop(i)
        return ""

    '''
        userInfoS = {
                        "Name"              : <user_name_given_on Connect>,
                        "Axis"              : 3
                     }
    '''


    def what_is_stattus(self,state):
        if state == '0A':
            return '  state: NOT REFERENCED from reset'
        elif state == '0B':
            return '  state: NOT REFERENCED from HOMING'
        elif state == '0C':
            return '  state: NOT REFERENCED from CONFIGURATION'
        elif state == '0D':
            return '  state: NOT REFERENCED from DISABLE'
        elif state == '0E':
            return '  state: NOT REFERENCED from READY'
        elif state == '0F':
            return '  state: NOT REFERENCED from MOVING'
        elif state == '10':
            return '  state: NOT REFERENCED ESP stage error'
        elif state == '11':
            return '  state: NOT REFERENCED from JOGGING'
        elif state == '14':
            return '  state: CONFIGURATION'
        elif state == '1E':
            return '  state: HOMING commanded from RS-232-C'
        elif state == '1F':
            return '  state: HOMING commanded by SMC-RC'
        elif state == '28':
            return '  state: MOVING'
        elif state == '32':
            return '  state: READY from HOMING'
        elif state == '33':
            return '  state: READY from MOVING'
        elif state == '34':
            return '  state: READY from DISABLE'
        elif state == '35':
            return '  state: READY from JOGGING'
        elif state == '3C':
            return '  state: DISABLE from READY'
        elif state == '3D':
            return '  state: DISABLE from MOVING'
        elif state == '3E':
            return '  state: DISABLE from JOGGING'
        elif state == '46':
            return '  state: JOGGING from READY'
        elif state == '47':
            return '  state: JOGGING from DISABLE'

    @command(dtype_in=str,dtype_out=str)  
    def GetStatus(self,userInfoS):
        uIS=  json.loads(userInfoS)
        print(uIS)
        response = self.Controllers[uIS["Name"]].get_status(uIS["Axis"])
        info_motor = self.what_is_stattus(response[1])
        return info_motor
    
    
    '''
        userInfoMR = {
                        "Name"              : <user_name_given_on Connect>,
                        "Axis"              : 3,
                        "Position"          : 3,
                        "Wait_to_finish"    ; True
                     }
    '''

    @command(dtype_in=str,dtype_out=str)  
    def MoveRelative_mm(self,userInfoMR):
        uIMR=  json.loads(userInfoMR)
        print(uIMR)
        self.Controllers[uIMR["Name"]].move_relative_mm(uIMR["Axis"],uIMR["Position"],uIMR["Wait_to_finish"])
        return "The motor is moving"
    
    '''
        userInfoMTR = {
                        "Name"              : <user_name_given_on Connect>,
                        "Axis"              : 3,
                        "Position"          : 3
                     }
    '''

    
    @command(dtype_in=str,dtype_out=float)  
    def GetMotionTimeForRelativeMove(self,userInfoMTR):
        uIMTR=  json.loads(userInfoMTR)
        print(uIMTR)
        time_for_move = self.Controllers[uIMTR["Name"]].get_motion_time_for_relative_move(uIMTR["Axis"],uIMTR["Position"])
        return float(time_for_move)
    

    '''
        userInfoH = {
                        "Name"              : <user_name_given_on Connect>,
                        "Axis"              : 3,
                        "Wait_to_finish"    : True
                     }
    '''
    
    @command(dtype_in=str,dtype_out=str)  
    def Home(self,userInfoH):
        uIH=  json.loads(userInfoH)
        print(uIH)
        self.Controllers[uIH["Name"]].home(uIH["Axis"],uIH["Wait_to_finish"])
        return "Motor was moved to the Home."
    

    @command(dtype_in=str,dtype_out=str)  
    def Reset(self,userInfoH):
        uIH=  json.loads(userInfoH)
        print(uIH)
        self.Controllers[uIH["Name"]].reset_and_configure(uIH["Axis"])
        return "Motor was moved to the Home."
    
    '''
        userInfoGHST = {
                        "Name"              : <user_name_given_on Connect>,
                        "Axis"              : 3
                     }
    '''

    @command(dtype_in=str,dtype_out=str)  
    def GetHOMESearchType(self,userInfoGHST):
        uIGHST=  json.loads(userInfoGHST)
        print(uIGHST)
        return str(self.Controllers[uIGHST["Name"]].get_HOME_search_type(uIGHST["Axis"]))
    
    '''
        userInfoSHST = {
                        "Name"              : <user_name_given_on Connect>,
                        "Axis"              : 3,
                        "HT"                : 1
                     }
    '''

    @command(dtype_in=str,dtype_out=str)  
    def SetHOMESearchType(self,userInfoSHST):
        uISHST=  json.loads(userInfoSHST)
        print(uISHST)
        # self.Controllers[uISHST["Name"]].enter_Config_state(uISHST["Axis"])
        info = str(self.Controllers[uISHST["Name"]].Set_HOME_search_type(uISHST["Axis"],uISHST["HT"]))
        # self.Controllers[uISHST["Name"]].leave_Config_state(uISHST["Axis"])
        return info
    
if __name__ == "__main__":
    SMC100.run_server()



#  Because we are using the 'with' statement context-manager, disposal has been taken care of.
