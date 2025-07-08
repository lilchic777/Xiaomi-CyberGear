# Xiaomi-CyberGear

Debugging and inverse kinematics program for Xiaomi CyberGear robotic arm. Thanks to @Tony607 and @freezeLUO
for the open-source code.

## Purpose of each Python file

pcan-cybergear.py: Multi-mode motor controller based on CAN bus, supporting four modes: position/speed/current/motion control;<br/>

pcan-robotic-1axis.py: Debugging program for a single micro motor;<br/>

pcan-robotic-2axis.py: Debugging program for two micro motors simultaneously, which can be extended to control multiple axes simultaneously arm;<br/>

> [!NOTE]
>
> When controlling multiple motors simultaneously, you need to use Xiaomi's official CyberGear debugger to set different CAN IDs for the motors.

inverseKinematics.py: 4-degree-of-freedom robotic arm inverse kinematics: Given the corresponding coordinates (X, Y, Z) and pitch angle, calculate the rotation angle of each joint; only solves for the data corresponding to the given set of coordinates and pitch angle; returns False if no solution is found;<br/>

armMoveIK.py: Imports inverseKinematics, solves within the pitch angle range, and returns either the endpoint of the interval or reference data within the interval; returns False if no solution is found.

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











