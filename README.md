# Newport SMC100 Motor Controller - Tango Device Server

This repository contains the driver for controlling a  Newport SMC100 Controller with the Tango Control. After cloning this repository with the following command

```
git clone https://github.com/Golp-Voxel/Tango_SMC100.git
```

It is necessary to create the `tango-env` using the following command:

```
python -m venv tango-env
```

After activating it you can install all the models to run this tool by using the command:

```
pip install -r Requirements.txt
```

To complete the installation, it is necessary to copy the `SMC100.bat` template and change the paths to the installation folder. And the command to run the `...\tango-env\Scripts\activate` script. 

This repository contains the Library (`SMC100_Lib.py`) that enables communication with the SMC100 controller. The `Code_Exemple/Exemple_SMC100.ipynb` contains a example of how to inicialize the controllers and how to move the motors to a relative position.

## Available commands

After installing the Tango Device server, that can detect and connect to a Newport SMC100 Motor Controller, being able to move the motors connect to the it.

- [ConnectCamera](#connectcamera)
- [GetPosition](#getposition)
- [MoveAbsolute_mm](#moveabsolute_mm)
- [MoveRelative_mm](#moverelative_mm)
- [GetMotionTimeForAbsoluteMove](#getmotiontimeforabsolutemove)
- [GetMotionTimeForRelativeMove](#getmotiontimeforrelativemove)
- [Stop](#stop)
- [GetStatus](#getstatus)
- [Home](#home)
- [Reset](#reset)
- [GetHOMESearchType](#gethomesearchtype)
- [SetHOMESearchType](#sethomesearchtype)

Several SMC100 controllers can be daisy-chained on the same serial port. The `"Axis"` field used by most commands is the address of the controller in the chain (1, 2, 3, ...).

### ConnectCamera

Connects to a chain of SMC100 controllers on the given COM port and stores it under the given name.

```python
ConnectCamera(userInfoController)
```

```
userInfoController =  {
                            "Name"                   : <user choice>,
                            "COM"                    : 0,
                            "Number_of_controllers"  : 3
                        }
```

Where `"COM"` is the number of the serial port and `"Number_of_controllers"` is the number of SMC100 units in the chain. It returns a message saying if the connection was successful or not.

### GetPosition

Returns the current position of the given axis in millimeters (float).

```python
GetPosition(userInfoP)
```

```
userInfoP = {
                "Name" : <user_name_given_on_Connect>,
                "Axis" : 3
            }
```

### MoveAbsolute_mm

Moves the given axis to an absolute position in millimeters.

```python
MoveAbsolute_mm(userInfoMA)
```

```
userInfoMA = {
                "Name"           : <user_name_given_on_Connect>,
                "Axis"           : 3,
                "Position"       : 3,
                "Wait_to_finish" : true
             }
```

If `"Wait_to_finish"` is `true` the command only returns after the motion is complete.

### MoveRelative_mm

Moves the given axis by a relative distance in millimeters (same JSON format as [MoveAbsolute_mm](#moveabsolute_mm), where `"Position"` is the relative displacement).

```python
MoveRelative_mm(userInfoMR)
```

### GetMotionTimeForAbsoluteMove

Returns the time (in seconds, float) that the motion to the given absolute position would take from the current position.

```python
GetMotionTimeForAbsoluteMove(userInfoMTA)
```

```
userInfoMTA = {
                "Name"     : <user_name_given_on_Connect>,
                "Axis"     : 3,
                "Position" : 3
              }
```

### GetMotionTimeForRelativeMove

Returns the time (in seconds, float) that a relative move of the given distance would take (same JSON format as [GetMotionTimeForAbsoluteMove](#getmotiontimeforabsolutemove), where `"Position"` is the relative displacement).

```python
GetMotionTimeForRelativeMove(userInfoMTR)
```

### Stop

Stops the motion of the given axes. `"Axis"` is a list of the axes to stop.

```python
Stop(userInfoStop)
```

```
userInfoStop = {
                "Name" : <user_name_given_on_Connect>,
                "Axis" : [1, 2, 3]
               }
```

### GetStatus

Returns the state of the given axis as a human readable string.

```python
GetStatus(userInfoS)
```

```
userInfoS = {
                "Name" : <user_name_given_on_Connect>,
                "Axis" : 3
            }
```

The possible states (decoded from the SMC100 status codes) are:

| Code | State |
|------|-------|
| 0A-11 | NOT REFERENCED (from reset, HOMING, CONFIGURATION, DISABLE, READY, MOVING, ESP stage error, JOGGING) |
| 14 | CONFIGURATION |
| 1E / 1F | HOMING (commanded from RS-232-C / by SMC-RC) |
| 28 | MOVING |
| 32-35 | READY (from HOMING, MOVING, DISABLE, JOGGING) |
| 3C-3E | DISABLE (from READY, MOVING, JOGGING) |
| 46 / 47 | JOGGING (from READY, DISABLE) |

### Home

Executes the home search of the given axis. The motor must be homed (referenced) after a reset before absolute moves can be done.

```python
Home(userInfoH)
```

```
userInfoH = {
                "Name"           : <user_name_given_on_Connect>,
                "Axis"           : 3,
                "Wait_to_finish" : true
             }
```

### Reset

Resets and configures the given axis (same JSON format as [Home](#home), without `"Wait_to_finish"`).

```python
Reset(userInfoH)
```

### GetHOMESearchType

Returns the HOME search type configured on the given axis.

```python
GetHOMESearchType(userInfoGHST)
```

```
userInfoGHST = {
                "Name" : <user_name_given_on_Connect>,
                "Axis" : 3
               }
```

### SetHOMESearchType

Sets the HOME search type of the given axis. See the SMC100 manual (command `HT`) for the meaning of each value.

```python
SetHOMESearchType(userInfoSHST)
```

```
userInfoSHST = {
                "Name" : <user_name_given_on_Connect>,
                "Axis" : 3,
                "HT"   : 1
               }
```

## Example of Tango Client code

```python
import tango
import json

SMC100_Motor = tango.DeviceProxy(<SMC100_Tango_location_on_the_database>)
print(SMC100_Motor.state())
# Homing and long moves can take a while, increase the timeout if needed
SMC100_Motor.set_timeout_millis(30000)

# Connect to a chain of 3 controllers on COM4 and name it "M1"
JSON_Controller = {"Name": "M1",
                   "COM": 4,
                   "Number_of_controllers": 3}
print(SMC100_Motor.ConnectCamera(json.dumps(JSON_Controller)))

# Home the axis 1 and wait for it to finish
print(SMC100_Motor.Home(json.dumps({"Name": "M1", "Axis": 1, "Wait_to_finish": True})))

# Check the state of the axis 1
print(SMC100_Motor.GetStatus(json.dumps({"Name": "M1", "Axis": 1})))

# Move the axis 1 to 5 mm and read back the position
SMC100_Motor.MoveAbsolute_mm(json.dumps({"Name": "M1", "Axis": 1,
                                         "Position": 5, "Wait_to_finish": True}))
print(SMC100_Motor.GetPosition(json.dumps({"Name": "M1", "Axis": 1})))
```

# References

- [Newport SMC100 User Manual](https://www.newport.com/mam/celum/celum_assets/resources/SMC100CC_and_SMC100PP_-_User_Manual.pdf)
