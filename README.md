# Xiaomi-CyberGear

Debugging and inverse kinematics program for Xiaomi CyberGear robotic arm. Thanks to @Tony607 and @freezeLUO
for the open-source code.

## Key features

### Low-level tool files

`pcan-cybergear.py`: Multi-mode motor controller based on CAN bus, supporting four modes: position/speed/current/motion control;

`tools.py`: Python files stored in ESP32 provide various operations for controlling motors via CAN bus (including setting operating modes, positions, speeds, currents, and other parameters to achieve position control, start/stop, and communication).

### Debugging motor files

`pcan-robotic-1axis.py`: Debugging program for a single micro motor;

`pcan-robotic-multiaxes.py`: Debugging program for four micro motors simultaneously.

> [!NOTE]
>
> When controlling multiple motors simultaneously, you need to use Xiaomi's official CyberGear debugger to set different CAN IDs for the motors.

### Inverse kinematics calculation and control

`inverseKinematics.py`: 4-degree-of-freedom robotic arm inverse kinematics: Given the corresponding coordinates (X, Y, Z) and pitch angle, calculate the rotation angle of each joint; only solves for the data corresponding to the given set of coordinates and pitch angle; returns False if no solution is found;

`armMoveIK.py`: Imports inverseKinematics, solves within the pitch angle range, and returns either the endpoint of the interval or reference data within the interval; returns False if no solution is found.

### Web control

This code is used for controlling motors from a web page, accessible from both PCs and mobile devices. You can simply run `main.py` of this project and then open http://127.0.0.1:5001 or http://10.10.60.133:5001 to use the control panel.

### 4-axis teach

`pcan-robotic-4axis-teach.ipynb`: This code is a Jupyter Notebook script used to implement teaching and playback functions for the joints of a quadcopter robot.

> [!NOTE]
>
> Since this code needs to be executed in steps (initialization, gradual recording of the robot arm's pose, and replay of the robot arm's pose), please configure the Jupyter Notebook environment with Jupyter MicroPython Kernel to run it.

Open the cmd terminal and enter the command line to execute the installation:

```bash
pip install jupyter notebook
pip install jupyter_micropython_kernel
python -m jupyter_micropython_kernel.install
```

You can run the following command to verify that the installation was successful:

```bash
jupyter kernelspec list
```

Open the cmd terminal and enter the command line to open Jupyter Notebook:

```bash
jupyter notebook
```

Then open `pcan-robotic-4axis-teach.ipynb` in your browser. Note that the `notebook` folder should be parallel to the `cybergear` folder.

## Hardware modules involved

Xiaomi CyberGear

PCAN kvaser leaf Socketcan canable (AP311_USBCAN)

Type-C ESP32 V1.0.0 Rev1 Development Board

SN65HVD230 CAN Module

## How to use

### PCAN

Install python requirements:

```bash
pip install -r requirements.txt
```

> [!NOTE]
>
> The code used for debugging in this project is implemented by default using the PCAN module. If you are using the ESP32 module, please note that you will need to use a CAN conversion module (such as SN65HVD230) and pay attention to the hardware connections.

### ESP32

If you use ESP32 instead of PCAN module to control (for example, to implement the power-on self-start function), install esptool first: https://github.com/espressif/esptool

or:

```bash
pip install esptool
```

Erase the existing content of ESP32:

```bash
python esptool.py --port COM5 erase_flash
```

Flash the MicroPython-compatible firmware to the ESP32 (note: change the COM port name to the corresponding ESP32 serial port number):

```bash
python esptool.py -p COM5 -b 460800 --before default_reset --after hard_reset --chip esp32  write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 bootloader.bin 0x8000 partition-table.bin 0x10000 micropython.bin
```

> [!NOTE]
>
> Your work directory needs to contain all the `*.bin` firmware files.

Using Thonny IDE to upload `tools.py` to the ESP32:

Open Thonny → Tools → Options → Interpreter; Select MicroPython (ESP32) and the correct port.

Paste the code from `tools.py` into the editor window. Click File → Save as → Select MicroPython device → Enter the filename tools.py.

> [!NOTE]
>
> If you need to enable automatic startup, save it as `main.py`.









