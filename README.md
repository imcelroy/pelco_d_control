# PELCO_D control Package

## Protocol:
The protocol class is found at [src/pelco_d.py](src/pelco_d.py). The class only contains pre-made methods for `set_preset_85` (WIPER ON) and `set_preset_86` (WIPER OFF) for the VideoTec case (according to the documentation here: [MAXIMUS MHX, MAXIMUS MHXT](https://www.videotec.com/dep/ekr/Manuali/MAXIMUS-MHX_MAXIMUS-MHXT_Manual.pdf#MNVCMHX_1706_EN.indd%3A.16516%3A139))

The class was constructed from this tutorial here: https://www.commfront.com/pages/pelco-d-protocol-tutorial

## Architecture

The package has a simple node ([scripts/pelco_d_control_node.py](scripts/pelco_d_control_node.py)), that has one subscriber to a std_msgs/Bool topic. This topic is used to control the ON/OFF of the wiper.

There is a serial communication class to handle the serial communications here: [src/serial_comm_interface.py](src/serial_comm_interface.py). This file also contains a mocked serial communication class to test the protocol, which is controlled via the `mocked` launch parameter.

## Build:
Run `catkin_make` in workspace.

## Launch:
```
roslaunch pelco_d_control pelco_d_control_node.py
```
