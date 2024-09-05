"""

This code was a adption of the original work done by Steve Bian that can be found in:
https://github.com/freespace/pySMC100 

The orginal code was writen in python 2.X so I need to update and correct the diferences.

Also with the manifacture User manuel of the SMC100:
https://www.newport.com/mam/celum/celum_assets/resources/SMC100CC_and_SMC100PP_-_User_Manual.pdf?3

It was expanse the amount of comands avaiable to the user.

Author of this version: Pedro Rossa 

"""
import serial
import time

from math import floor

code = 'UTF-8'

# never wait for more than this e.g. during wait_states
MAX_WAIT_TIME_SEC = 120
TIME_BETWEEN_GET_STATUS = 2

# time to wait after sending a command. This number has been arrived at by
# trial and error
COMMAND_WAIT_TIME_SEC = 0.06

# States from page 65 of the manual
STATE_NOT_REFERENCED_FROM_RESET = '0A'
STATE_NOT_REFERENCED_FROM_CONFIGURATION = '0C'
STATE_READY_FROM_HOMING = '32'
STATE_READY_FROM_MOVING = '33'

STATE_CONFIGURATION = '14'

STATE_DISABLE_FROM_READY = '3C'
STATE_DISABLE_FROM_MOVING = '3D'
STATE_DISABLE_FROM_JOGGING = '3E'
HOMING_RS232C = '1E'

print('SMC')

class SMC100ReadTimeOutException(Exception):
  def __init__(self):
    super(SMC100ReadTimeOutException, self).__init__('Read timed out')

class SMC100WaitTimedOutException(Exception):
  def __init__(self):
    super(SMC100WaitTimedOutException, self).__init__('Wait timed out')

class SMC100DisabledStateException(Exception):
  def __init__(self, state):
    super(SMC100DisabledStateException, self).__init__('Disabled state encountered: '+state)

class SMC100RS232CorruptionException(Exception):
  def __init__(self, c):
    super(SMC100RS232CorruptionException, self).__init__('RS232 corruption detected: %s'%(hex(ord(c))))

class SMC100InvalidResponseException(Exception):
  def __init__(self, cmd, resp):
    s = 'Invalid response to %s: %s'%(cmd, resp)
    super(SMC100InvalidResponseException, self).__init__(s)

""""
    TODO:
    Change the class to just establish the communication and pass the address to the function.
    Because this initial was built for only 1 connected controller, but we are going to use 3 different controllers.
    with only one serial port. 

"""
class SMC100(object):
  """
  Class to interface with Newport's SMC100 controller.

  The SMC100 accepts commands in the form of:

    <ID><command><arguments><CR><LF>

  Reply, if any, will be in the form

    <ID><command><result><CR><LF>

  There is minimal support for manually setting stage parameter as Newport's
  ESP stages can supply the SMC100 with the correct configuration parameters.

  Some effort is made to take up backlash, but this should not be trusted too
  much.

  The move commands must be used with care, because they make assumptions
  about the units which is dependent on the STAGE. I only have TRB25CC, which
  has native units of mm. A more general implementation will move the move
  methods into a stage class.
  """

  _port = None

  _silent = True

  _sleepfunc = time.sleep

#___________________ Default function need for initiate and kill the classe ___________________ 

  def __init__(self, port, backlash_compensation=True, silent=True, sleepfunc=None):
    """
    If backlash_compensation is False, no backlash compensation will be done.

    If silent is False, then additional output will be emitted to aid in
    debugging.

    If sleepfunc is not None, then it will be used instead of time.sleep. It
    will be given the number of seconds (float) to sleep for, and is provided
    for ease integration with single threaded GUIs.

    Note that this method only connects to the controller, it otherwise makes
    no attempt to home or configure the controller for the attached stage. This
    delibrate to minimise realworld side effects.

    If the controller has previously been configured, it will suffice to simply
    call home() to take the controller out of not referenced mode. For a brand
    new controller, call reset_and_configure().
    """

    super(SMC100, self).__init__()

    assert port is not None

    if sleepfunc is not None:
      self._sleepfunc = sleepfunc

    self._silent = silent

    self._last_sendcmd_time = 0

    print(f'Connecting to SMC100 on {port}')

    self._port = serial.Serial(
        port = port,
        baudrate = 57600,
        bytesize = 8,
        stopbits = 1,
        parity = 'N',
        xonxoff = True,
        timeout = 0.050)

  def close(self):
    if self._port:
      self._port.close()
      self._port = None

  def __del__(self):
    self.close()

#______________________________________________________________________________________________ 




#_______________________ Mandatory functions need to send and _________________________________
#_______________________ read the message from the Controller __________________________________ 

  def sendcmd(self, ID, command, argument=None, wait_time=COMMAND_WAIT_TIME_SEC, expect_response=False, retry=False):
    """
    Send the specified command along with the argument, if any. The response
    is checked to ensure it has the correct prefix, and is returned WITHOUT
    the prefix.

    It is important that for GET commands, e.g. 1ID?, the ? is specified as an
    ARGUMENT, not as part of the command. Doing so will result in assertion
    failure.

    If expect_response is True, a response is expected from the controller
    which will be verified and returned without the prefix.

    If expect_response is True, and retry is True or an integer, then when the
    response does not pass verification, the command will be sent again for
    retry number of times, or until success if retry is True.

    The retry option MUST BE USED CAREFULLY. It should ONLY be used read-only
    commands, because otherwise REPEATED MOTION MIGHT RESULT. In fact some
    commands are EXPLICITLY REJECTED to prevent this, such as relative move.
    """
    assert command[-1] != '?'

    if self._port is None:
      return

    if argument is None:
      argument = ''

    prefix = str(ID) + command
    tosend = prefix + str(argument)
    print(tosend)

    # prevent certain commands from being retried automatically
    no_retry_commands = ['PR', 'OR', 'OT']
    if command in no_retry_commands:
      retry = False

    while self._port is not None:
      if expect_response:
        self._port.reset_input_buffer()

      self._port.reset_output_buffer()

      self._port.write(tosend.encode()+b'\r\n')

      self._port.flush()

      if not self._silent:
        self._emit('sent', tosend)

      if expect_response:
        try:
          response = self._readline()
          if response.startswith(prefix):
            return response[len(prefix):]
          else:
            raise SMC100InvalidResponseException(command, response)
        except Exception as ex:
          if not retry or retry <=0:
            raise ex
          else:
            if type(retry) == int:
              retry -= 1
            continue
      else:
        # we only need to delay when we are not waiting for a response
        now = time.time()
        dt = now - self._last_sendcmd_time
        dt = wait_time - dt
        if dt > 0:
          self._sleepfunc(dt)
        
        self._last_sendcmd_time = now
        return None

  def _readline(self):
    """
    Returns a line, that is reads until \r\n.

    """
    done = False
    line = str()
    Terminator = b'\r\n'

    """
    Old version (python 2.X) Serial now was the function read_until 
    where we can set msg terminators or the len of the msg

    https://pyserial.readthedocs.io/en/latest/pyserial_api.html#serial.Serial.read_until

    """
    #print 'reading line',
    # while not done:
    #   c = self._port.read()
    #   # ignore \r since it is part of the line terminator
    #   if len(c) == 0:
    #     raise SMC100ReadTimeOutException()
    #   elif c == '\r':
    #     continue
    #   elif c == '\n':
    #     done = True
    #   elif ord(c) > 32 and ord(c) < 127:
    #     line += c
    #   else:
    #     raise SMC100RS232CorruptionException(c)
    test = self._port.read_until(Terminator)
    print("Before decode: ")
    print(test)
    line = test.decode(code)[:-2]
    self._emit('read', line)

    return line

  def _emit(self, *args):
    if len(args) == 1:
      prefix = ''
      message = args[0]
    else:
      prefix = ' ' + args[0]
      message = args[1]

    if not self._silent:
      print(f'[SMC100 {prefix}] {message}')

#______________________________________________________________________________________________ 






#_______________________ Function to operate the Controller SMC00  ____________________________
#____________________________ and the motors connected to it __________________________________


  def reset_and_configure(self,ID):
    """
    Configures the controller by resetting it and then asking it to load
    stage parameters from an ESP compatible stage. This is then followed
    by a homing action.
    """
    self.sendcmd(ID,'RS')
    self.sendcmd(ID,'RS')

    self._sleepfunc(3)

    self.wait_states(ID,STATE_NOT_REFERENCED_FROM_RESET, ignore_disabled_states=True)

    stage = self.sendcmd(ID,'ID', '?', True)
    print(f'Found stage {stage}')

    # enter config mode
    self.sendcmd(ID,'PW', 1)

    self.wait_states(ID,STATE_CONFIGURATION)

    # load stage parameters
    self.sendcmd(ID,'ZX', 1)

    # enable stage ID check
    self.sendcmd(ID,'ZX', 2)

    # exit configuration mode
    self.sendcmd(ID,'PW', 0)

    # wait for us to get back into NOT REFERENCED state
    self.wait_states(ID,STATE_NOT_REFERENCED_FROM_CONFIGURATION)

  def home(self, ID, waitStop=True):
    """
    Homes the controller. If waitStop is True, then this method returns when
    homing is complete.

    Note that because calling home when the stage is already homed has no
    effect, and homing is generally expected to place the stage at the
    origin, an absolute move to 0 um is executed after homing. This ensures
    that the stage is at origin after calling this method.

    Calling this method is necessary to take the controller out of not referenced
    state after a restart.
    """
    
    self.sendcmd(ID,'OR')
    if waitStop:
      # wait for the controller to be ready
      st = self.wait_states(ID,[STATE_READY_FROM_HOMING, STATE_READY_FROM_MOVING,HOMING_RS232C])
      if st == STATE_READY_FROM_MOVING:
        self.move_absolute_um(ID,0, waitStop=True)
    else:
      self.move_absolute_um(ID,0, waitStop=False)


  def stop(self,ID):
    self.sendcmd(ID,'ST')


  def what_is_stattus(self,state):
    if state == '0A':
      print('  state: NOT REFERENCED from reset')
    elif state == '0B':
      print('  state: NOT REFERENCED from HOMING')
    elif state == '0C':
      print('  state: NOT REFERENCED from CONFIGURATION')
    elif state == '0D':
      print('  state: NOT REFERENCED from DISABLE')
    elif state == '0E':
      print('  state: NOT REFERENCED from READY')
    elif state == '0F':
      print('  state: NOT REFERENCED from MOVING')
    elif state == '10':
      print('  state: NOT REFERENCED ESP stage error')
    elif state == '11':
      print('  state: NOT REFERENCED from JOGGING')
    elif state == '14':
      print('  state: CONFIGURATION')
    elif state == '1E':
      print('  state: HOMING commanded from RS-232-C')
    elif state == '1F':
      print('  state: HOMING commanded by SMC-RC')
    elif state == '28':
      print('  state: MOVING')
    elif state == '32':
      print('  state: READY from HOMING')
    elif state == '33':
      print('  state: READY from MOVING')
    elif state == '34':
      print('  state: READY from DISABLE')
    elif state == '35':
      print('  state: READY from JOGGING')
    elif state == '3C':
      print('  state: DISABLE from READY')
    elif state == '3D':
      print('  state: DISABLE from MOVING')
    elif state == '3E':
      print('  state: DISABLE from JOGGING')
    elif state == '46':
      print('  state: JOGGING from READY')
    elif state == '47':
      print('  state: JOGGING from DISABLE')



  def get_status(self, ID,wait_time=COMMAND_WAIT_TIME_SEC, silent=False):
    """
    Executes TS? and returns the the error code as integer and state as string
    as specified on pages 64 - 65 of the manual.
    """

    resp = self.sendcmd(ID,'TS', '?',wait_time, expect_response=True, retry=10)
    errors = int(resp[0:4], 16)
    state = resp[4:]

    print(len(state))
    assert len(state) == 2
    
    if not silent:
      print('status:', end = " " )
      self.what_is_stattus(state)
    return errors, state

  def get_position_mm(self,ID):
    dist_mm = float(self.sendcmd(ID,'TP', '?', expect_response=True, retry=10))
    return dist_mm
  
  def get_HOME_search_type(self,ID):
    info = self.sendcmd(ID,'HT', '?', expect_response=True, retry=10)
    return info
  
  def set_HOME_search_type(self,ID,info_user):
    if info_user in range(0,5):
      info = self.sendcmd(ID,'HT',str(info_user), expect_response=False, retry=10)
    else:
      info = "Error: Read the manuak for the command HT"
      print(" Read the manuak for the command HT")
    return info

  def get_position_um(self):
    return int(self.get_position_mm()*1000)

  def move_relative_mm(self, ID, dist_mm, waitStop=True):
    """
    Moves the stage relatively to the current position by the given distance given in mm

    If waitStop is True then this method returns when the move is completed.
    """
    self.sendcmd(ID,'PR', dist_mm)
    if waitStop:
      # If we were previously homed, then something like PR0 will have no
      # effect and we end up waiting forever for ready from moving because
      # we never left ready from homing. This is why STATE_READY_FROM_HOMING
      # is included.
      self.wait_states(ID,(STATE_READY_FROM_MOVING, STATE_READY_FROM_HOMING))


  def move_relative_um(self, ID, dist_um, **kwargs):
    """
    Moves the stage relatively to the current position by the given distance given in um. The
    given distance is first converted to an integer.

    If waitStop is True then this method returns when the move is completed.
    """
    dist_mm = int(dist_um)/1000
    self.move_relative_mm(ID,dist_mm, **kwargs)

  def move_absolute_mm(self, ID, position_mm, waitStop=True):
    """
    Moves the stage to the given absolute position given in mm.

    If waitStop is True then this method returns when the move is completed.
    """
    self.sendcmd(ID,'PA', position_mm)
    if waitStop:
      # If we were previously homed, then something like PR0 will have no
      # effect and we end up waiting forever for ready from moving because
      # we never left ready from homing. This is why STATE_READY_FROM_HOMING
      # is included.
      self.wait_states(ID,(STATE_READY_FROM_MOVING, STATE_READY_FROM_HOMING))

  def move_absolute_um(self, ID, position_um, **kwargs):
    """
    Moves the stage to the given absolute position given in um. Note that the
    position specified will be floor'd first before conversion to mm.

    If waitStop is True then this method returns when the move is completed.
    """
    pos_mm = floor(position_um)/1000
    return self.move_absolute_mm(ID, pos_mm, **kwargs)
  


# I think this is going to be redone 

  def wait_states(self, ID, targetstates,wait_time=COMMAND_WAIT_TIME_SEC, ignore_disabled_states=False):
    """
    Waits for the controller to enter one of the the specified target state.
    Controller state is determined via the TS command.

    If ignore_disabled_states is True, disable states are ignored. The normal
    behaviour when encountering a disabled state when not looking for one is
    for an exception to be raised.

    Note that this method will ignore read timeouts and keep trying until the
    controller responds.  Because of this it can be used to determine when the
    controller is ready again after a command like PW0 which can take up to 10
    seconds to execute.

    If any disable state is encountered, the method will raise an error,
    UNLESS you were waiting for that state. This is because if we wait for
    READY_FROM_MOVING, and the stage gets stuck we transition into
    DISABLE_FROM_MOVING and then STAY THERE FOREVER.

    The state encountered is returned.
    """
    starttime = time.time()
    done = False
    self._emit('waiting for states %s'%(str(targetstates)))
    while not done:
      waittime = time.time() - starttime
      if waittime > MAX_WAIT_TIME_SEC:
        raise SMC100WaitTimedOutException()

      try:
        state = self.get_status(ID,wait_time)[1]
        if state in targetstates:
          self._emit('in state %s'%(state))
          return state
        elif not ignore_disabled_states:
          disabledstates = [
              STATE_DISABLE_FROM_READY,
              STATE_DISABLE_FROM_JOGGING,
              STATE_DISABLE_FROM_MOVING]
          if state in disabledstates:
            raise SMC100DisabledStateException(state)

      except SMC100ReadTimeOutException:
        self._emit('Read timed out, retrying in 1 second')
        self._sleepfunc(1)
        continue
      time.sleep(TIME_BETWEEN_GET_STATUS)
      
#______________________________________________________________________________________________ 







#_______________________________ Function TO BE TESTED _________________________________
      

  def get_controller_address(self,ID):
    resp = self.sendcmd(ID,'SA', '?', expect_response=True, retry=10)
    print(resp)
    return
  


  def set_controller_address(self, ID,new_addr=1):
    if new_addr == 1:
        print("The controller is by default address set as 1\n\r")
    else:
        resp = self.sendcmd(ID,'SA', str(new_addr))
        print(resp)
    return 
  


  def get_acceleration(self,ID):
    resp = self.sendcmd(ID,'AC', '?', expect_response=True, retry=10)
    print(resp)
    return
  
  def set_acceleration(self,ID,new_acc):
    resp = self.sendcmd(ID,'AC', str(new_acc))
    print(resp)
    return 
  

  def get_stepper_motor_configuration(self,ID):
    resp = self.sendcmd(ID,'FR', '?', expect_response=True, retry=10)
    print(resp)
    return
  
  def set_stepper_motor_configuration(self,ID,new_addr=1):
    if new_addr == 1:
        print("The controller is by default address set as 1\n\r")
    else:
        resp = self.sendcmd(ID,'FR', str(new_addr))
        print(resp)
    return 
  '''
    This function give a the time the controller calculated it will take to move to the position that was given
    There is not equivalent function for the absolute movement. 
  '''
  def get_motion_time_for_relative_move(self,ID,move):
    resp = self.sendcmd(ID,'PT', str(move), expect_response=True, retry=10)
    print(resp)
    return resp
  
#TE
  def get_last_command_error(self,ID):
    resp = self.sendcmd(ID,'TE',"", expect_response=True, retry=10)
    print(resp)
    return
  
  
  def enter_Config_state(self,ID):   
    resp = self.sendcmd(ID,'PW',"1")
    print(self.get_status(ID))
    return
  def leave_Config_state(self,ID):
    resp = self.sendcmd(ID,'PW',"0")
    print(resp)
    return  


  def get_backlash_compensati(self,ID):
    resp = self.sendcmd(ID,'BA',"?", expect_response=True, retry=10)
    print(resp)
    return
  def set_backlash_compensati(self,ID,Backlash):
    resp = self.sendcmd(ID,'BA',str(Backlash))
    print(resp)
    return
  
  def get_negative_software_limit(self,ID):
    resp = self.sendcmd(ID,'SL',"?", expect_response=True, retry=10)
    print(resp)
    return
  
  def set_negative_software_limit(self,ID,Backlash):
    resp = self.sendcmd(ID,'SL',str(Backlash))
    print(resp)
    return
  
  
  def get_positive_software_limit(self,ID):
    resp = self.sendcmd(ID,'SR',"?", expect_response=True, retry=10)
    print(resp)
    return
  
  def set_positive_software_limit(self,ID,Backlash):
    resp = self.sendcmd(ID,'SR',str(Backlash))
    print(resp)
    return
  
'''
This CMD is not for PP

'''
  # def get_encoder_increment_value(self,ID):
  #   resp = self.sendcmd(ID,'SU',"?", expect_response=True, retry=10)
  #   print(resp)
  #   return
  # def set_encoder_increment_value(self,ID,valeu):
  #   resp = self.sendcmd(ID,'SU',str(valeu))
  #   print(resp)
  #   return

'''
This CMD is not for PP ^

'''

#______________________________________________________________________________________________ 

def test_general(ID):
  print('test_general')
  smc100 = SMC100('COM10', silent=False)
  print(smc100.get_position_mm(1))

  smc100.home(ID)

  # make sure there are no errors
  assert smc100.get_status(ID)[0] == 0

  smc100.move_relative_um(ID,5*1000)
  smc100.move_relative_mm(ID,5)

  #assert smc100.get_status()[0] == 0

  pos = smc100.get_position_mm(ID)

  assert abs(pos-10)<0.001

  smc100.move_relative_mm(ID,-pos)

  #assert smc100.get_status()[0] == 0

  del smc100

def test_AC():
  smc100 = SMC100('COM10', silent=False)
  print (smc100.get_position_mm())
  print ("Acceleration")
  smc100.get_acceleration()
  # assert smc100.get_status()[0] == 0
  smc100.set_acceleration(500)
  smc100.get_acceleration()
  # assert smc100.get_status()[0] == 0
  del smc100


"""
_____________________________________________________________
            This was only to set the RS485 Addr
        READ SMC100 MANUAL BEFORE USING THIS FUNCTION
_____________________________________________________________
"""

# def test_SA():
#     # Change this parameter to the desired communication address (all controllers must have different address)
#     RS485_Controll_ADR = 4
#     print('test_general')
#     smc100 = SMC100('COM10', silent=False)
#     #smc100.enter_Config_state()
#     smc100.set_controller_address(RS485_Controll_ADR)
#     # time.sleep(10)
#     assert smc100.get_status()[0] == 0
#     smc100.get_last_command_error()
#     smc100.get_controller_address()
#     smc100.leave_Config_state()
#     time.sleep(10)
    
#     assert smc100.get_status()[0] == 0
"""
_____________________________________________________________
"""  

'''
        This function is one time use 
'''
def config_BA(Com,BA_array=[]):
  smc100 = SMC100(Com, silent=False)
  if BA_array != []:
    for i in range(1,len(BA_array)+1): 
      smc100.enter_Config_state(i)
      smc100.set_backlash_compensati(i,BA_array[i-1])
      smc100.leave_Config_state(i)
      time.sleep(5)
      smc100.get_backlash_compensati(i)
  else:
    return "ERROR"
"""
_____________________________________________________________
"""  


def tets_home(smc100,ID):
  smc100.home(ID, waitStop=True)

def init_connection(Com,number_of_controller):
  smc100 = SMC100(Com, silent=False)
  for i in range(1,number_of_controller+1):
     
    smc100.get_status(i)
    # if i is not 3:
    smc100.home(i, waitStop=False)
    # # # # time.sleep(1)
    # # # # smc100.stop(i)
    if i == 3:
      smc100.stop(i)
    # # else:
    #   smc100.reset_and_configure(3)
    time.sleep(0.4)
  return smc100

# if __name__ == "__main__":
#     # smc100 = SMC100('COM10', silent=False)
#     smc100 = init_connection('COM5',3)
#     smc100.leave_Config_state(3)
  
#     time.sleep(3)
#     l = smc100.get_HOME_search_type(1)
#     l = smc100.get_HOME_search_type(2)
#     l = smc100.get_HOME_search_type(3)

#     # smc100.enter_Config_state(1)
#     # smc100.set_HOME_search_type(1,4)
    
#     # smc100.enter_Config_state(2)
#     # smc100.set_HOME_search_type(2,4)
    
#     # smc100.enter_Config_state(3)
#     # smc100.set_HOME_search_type(3,4)
#     # smc100.get_negative_software_limit(2)
#     # smc100.get_positive_software_limit(2)
#     # config_BA("COM10",["0.00197","0.00185","0.00202"])
#     # smc100.get_status(1)
#     # t = smc100.get_motion_time_for_relative_move(1,5)
#     # smc100.move_relative_mm(1,16,True)
#     # time.sleep(float(t))
#     # t = smc100.get_motion_time_for_relative_move(2,3)
#     # smc100.move_relative_mm(2,3,True)
#     del smc100
    