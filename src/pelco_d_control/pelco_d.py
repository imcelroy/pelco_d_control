#
# @file - pelco_d.py
# @author - Ian MCELROY, Forssea Robotics (ian@forssea-robotics.fr)
# @brief - Pelco D protocol.
# @version - 0.1
# @date - 2021-10-28
#
# Copyright (c) 2021 Forssea Robotics. All rights reserved.
#

#! /usr/bin/env python
import serial
import sys
from functools import reduce


class PelcoDProtocol():

    def __init__(self, device_address):
        self.device_address = device_address

        # constants
        self.sync_byte = '\xFF'

        # preset 85 (WIPER ON)
        self.preset_85 = '\x55'
        # preset 86 (WIPER OFF)
        self.preset_86 = '\x56'

    def construct_cmd(self, byte3, byte4, byte5, byte6):
        '''
            construct command in Pelco-D format:
            | Byte 1    | Byte 2            | Byte 3        | Byte 4        | Byte 5    | Byte 6    | Byte 7
            | Sync      | Camera Address    | Command 1     | Command 2     | Data 1    | Data 2    | Checksum
        '''
        payload = "{}{}{}{}{}".format(self.device_address, byte3, byte4, byte5, byte6)
        checksum = self.checksum256(payload)
        cmd_string = "{}{}{}".format(
            self.sync_byte, payload, checksum)

        return bytearray(cmd_string, 'UTF-8')

    def checksum256(self, cmd):
        '''
            Calculate checksum from payload cmd
        '''
        checksum = reduce(lambda x, y: x + y, map(ord, cmd)) % 256
        # hex(checksum): in the form 0x00
        result = hex(checksum)
        # hex(checksum)[2:]: in the form 00
        # result = hex(checksum)[2:]
        return result

    def construct_set_preset_85(self):
        '''
            set preset 85 for device
            (should be Wiper ON)
        '''
        return self.construct_cmd('\x00', '\x03', '\x00', self.preset_85)

    def construct_set_preset_86(self):
        '''
            set preset 86 for device
            (should be Wiper OFF)
        '''
        return self.construct_cmd('\x00', '\x03', '\x00', self.preset_86)
