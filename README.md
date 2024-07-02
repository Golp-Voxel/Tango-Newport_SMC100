# Newport SMC100 Motor Controller - Tango Device Server

This repository contains the driver for controlling a Standa 8SMC5-USB Controller with the Tango Control. After cloning this repository with the following command

```
git clone https://github.com/Golp-Voxel/Tango-Newport_SMC100.git
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

This repository contains the Library that enables communication with the SMC100 controller. The `Code_Exemple/Exemple_SMC100.ipynb` contains a example of how to inicialize the controllers and how to move the motors to a relative position.

## Available commands

After installing the Tango Device server, that can detect and connect to a Newport SMC100 Motor Controller, being able to move the motors connect to the it.

- [ConnectCamera](#ConnectCamera)

### ConnectCamera

 
'''
userInfoController =  {
                            "Name"                  : <user_name_given_on Connect>,
                            "COM"                   : 0,
                            "Number_of_controllers" : 3
                        }
'''

```python
ConnectCamera(userInfoController)
```
