#
# @file - pelco_d_control_node.py
# @author - Ian MCELROY, Forssea Robotics (ian@forssea-robotics.fr)
# @brief - Simple node for controlling pelco-d device
# @version - 0.1
# @date - 2021-10-27
#
# Copyright (c) 2021 Forssea Robotics. All rights reserved.
#


#! /usr/bin/env python
import sys
import serial
import rospy

from std_msgs.msg import Bool

from pelco_d_control.serial_comm_interface import MockSerial, SerialCommInterface
from pelco_d_control.pelco_d import PelcoDProtocol


class PelcoDControlNode():
    """
        Node class responsible for controlling a pelco-d device.
    """

    def __init__(self):
        rospy.init_node('pelco_d_control')
        self.device_address = rospy.get_param('~device_address', '\x01')
        self.port = rospy.get_param('~port', '/dev/RS485_0')
        self.baudrate = rospy.get_param('~baudrate', 9600)
        self.command_topic = rospy.get_param('~topic', 'pelco_d_cmd')
        self.mocked = rospy.get_param('~mocked', True)
        self.subscriber = rospy.Subscriber(self.command_topic, Bool, self.command_cb, queue_size=1)

        self.protocol = PelcoDProtocol(self.device_address)
        print(self.mocked)
        self.serial = SerialCommInterface(self.port, self.mocked, baudrate=self.baudrate)

        # try to open connection
        try:
            self.serial.open()
        except serial.SerialException as err:
            rospy.logerr("Pelcon-d node had error opening connection: {}". format(err))

    def command_cb(self, msg):
        if msg.data:
            cmd = self.protocol.construct_set_preset_85()
            rospy.loginfo("Pelcon-d node opening wiper, sending command: {}". format(cmd))
            self.serial.write(cmd)
        else:
            cmd = self.protocol.construct_set_preset_86()
            rospy.loginfo("Pelcon-d node closing wiper, sending command: {}". format(cmd))
            self.serial.write(cmd)

    def work(self):
        """
            Work method for pelco-d control node.
        """
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == '__main__':
    node = PelcoDControlNode()
    node.work()
