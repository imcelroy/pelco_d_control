#
# @file - serial_comm_interface.py
# @author - Ian MCELROY, Forssea Robotics (ian@forssea-robotics.fr)
# @brief - Simple interface class to work with serial comms.
# @version - 0.1
# @date - 2021-10-28
#
# Copyright (c) 2021 Forssea Robotics. All rights reserved.
#

#! /usr/bin/env python
import serial


class MockSerial():

    def __init__(self, port, mocked, baudrate=9600, parity=serial.PARITY_NONE,
                 stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS):
        self.port = port
        self.mocked = mocked
        self.baudrate = baudrate
        self.parity = parity
        self.stopbits = stopbits
        self.bytesize = bytesize

    def open(self):
        print("opened mock port: {}".format(self.port))

    def write(self, cmd):
        print("writing ({}) mock port: {}".format(cmd, self.port))

    def close(self):
        print("closed mock port: {}".format(self.port))


class SerialCommInterface():

    def __init__(self, port, mocked, baudrate=9600, parity=serial.PARITY_NONE,
                 stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS):
        self.port = port
        self.mocked = mocked
        self.baudrate = baudrate
        self.parity = parity
        self.stopbits = stopbits
        self.bytesize = bytesize

        self.connection = None
        # setup connection
        if not self.mocked:
            self.connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                parity=self.parity,
                stopbits=self.stopbits,
                bytesize=self.bytesize)
        else:
            self.connection = MockSerial(port, mocked, baudrate=9600, parity=serial.PARITY_NONE,
                                         stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

    def write(self, cmd):
        self.connection.write(cmd)

    def open(self):
        self.connection.open()

    def close(self):
        self.connection.close()
