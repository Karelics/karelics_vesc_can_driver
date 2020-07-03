#!/usr/bin/env python3
from abc import ABCMeta
from enum import IntEnum
import struct
from typing import List

import rospy
from can_msgs.msg import Frame


class CanIds(IntEnum):
    CAN_PACKET_SET_DUTY = 0
    CAN_PACKET_SET_CURRENT = 1
    CAN_PACKET_SET_CURRENT_BRAKE = 2
    CAN_PACKET_SET_RPM = 3
    CAN_PACKET_SET_POS = 4
    CAN_PACKET_FILL_RX_BUFFER = 5
    CAN_PACKET_FILL_RX_BUFFER_LONG = 6
    CAN_PACKET_PROCESS_RX_BUFFER = 7
    CAN_PACKET_PROCESS_SHORT_BUFFER = 8
    CAN_PACKET_STATUS = 9
    CAN_PACKET_SET_CURRENT_REL = 10
    CAN_PACKET_SET_CURRENT_BRAKE_REL = 11
    CAN_PACKET_SET_CURRENT_HANDBRAKE = 12
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL = 13
    CAN_PACKET_STATUS_2 = 14
    CAN_PACKET_STATUS_3 = 15
    CAN_PACKET_STATUS_4 = 16
    CAN_PACKET_PING = 17
    CAN_PACKET_PONG = 18
    CAN_PACKET_DETECT_APPLY_ALL_FOC = 19
    CAN_PACKET_DETECT_APPLY_ALL_FOC_RES = 20
    CAN_PACKET_CONF_CURRENT_LIMITS = 21
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS = 22
    CAN_PACKET_CONF_CURRENT_LIMITS_IN = 23
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN = 24
    CAN_PACKET_CONF_FOC_ERPMS = 25
    CAN_PACKET_CONF_STORE_FOC_ERPMS = 26
    CAN_PACKET_STATUS_5 = 27
    CAN_PACKET_POLL_TS5700N8501_STATUS = 28
    CAN_PACKET_CONF_BATTERY_CUT = 29
    CAN_PACKET_CONF_STORE_BATTERY_CUT = 30
    CAN_PACKET_SHUTDOWN = 31


class VescState:

    def __init__(self, vesc_id):
        self.vesc_id = vesc_id

        # Status message
        self.erpm = 0
        self.duty_cycle = 0
        self.current = 0
        # Status message 2
        self.amp_hours = 0
        self.amp_hours_charged = 0
        # Status message 3
        self.watt_hours = 0
        self.watt_hours_charged = 0
        # Status message 4
        self.temp_fet = 0
        self.temp_motor = 0
        self.current_in = 0
        self.pid_pos_now = 0
        # Status message 5
        self.tacho_value = 0
        self.v_in = 0

    def __str__(self):
        string = "VESC[%i]: \n" % self.vesc_id
        string += "erpm: %i duty: %f  current: %i  \n" % ( self.erpm, self.duty_cycle, self.current)
        string += "amp_hours: %i amp_hours_charged: %f \n" % (self.amp_hours, self.amp_hours_charged)
        string += "watt_hours: %i watt_hours_charged: %f \n" % (self.watt_hours, self.watt_hours_charged)
        string += "temp_fet: %f temp_motor: %f current_in: %f pid_pos_now: %f  \n" % (self.temp_fet, self.temp_motor, self.current_in, self.pid_pos_now)
        string += "tacho_value: %f v_in: %f \n" % (self.tacho_value, self.v_in)
        return string


class CanMsg(metaclass=ABCMeta):

    def __init__(self, msg_id):
        self.msg_id = msg_id
        self.buffer = bytes()
        self.pointer = 0
        self.on_update_func = None

    def pop_int32(self):
        data = struct.unpack(">i", self.buffer[self.pointer:self.pointer+4])[0]
        self.pointer += 4
        return data

    def pop_int16(self):
        data = struct.unpack(">h", self.buffer[self.pointer:self.pointer+2])[0]
        self.pointer += 2
        return data

    def set_data(self, data: bytes):
        self.pointer = 0
        self.buffer = data

    def process_msg(self, data):
        pass


class VescStatusMsg(CanMsg):

    def __init__(self):
        super(VescStatusMsg, self).__init__(msg_id=CanIds.CAN_PACKET_STATUS)
        self.erpm = 0
        self.current = 0
        self.duty_cycle = 0

    def process_msg(self, data):
        self.set_data(data)
        self.erpm = self.pop_int32()
        self.current = self.pop_int16() / 10.0
        self.duty_cycle = self.pop_int16() / 1000.0
        return self

    def update_vesc_state(self, vesc: VescState):
        vesc.erpm = self.erpm
        vesc.current = self.current
        vesc.duty_cycle = self.duty_cycle


class VescStatus2Msg(CanMsg):

    def __init__(self):

        super(VescStatus2Msg, self).__init__(msg_id=CanIds.CAN_PACKET_STATUS_2)
        self.amp_hours = 0
        self.amp_hours_charged = 0

    def update_vesc_state(self, vesc: VescState):
        vesc.amp_hours = self.amp_hours
        vesc.amp_hours_charged = self.amp_hours_charged

    def process_msg(self, data):
        self.set_data(data)
        self.amp_hours = self.pop_int32()/1e4
        self.amp_hours_charged = self.pop_int32()/1e4
        return self


class VescStatus3Msg(CanMsg):

    def __init__(self):

        super(VescStatus3Msg, self).__init__(msg_id=CanIds.CAN_PACKET_STATUS_3)
        self.watt_hours = 0
        self.watt_hours_charged = 0

    def update_vesc_state(self, vesc: VescState):
        vesc.watt_hours = self.watt_hours
        vesc.watt_hours_charged = self.watt_hours_charged

    def process_msg(self, data):
        self.set_data(data)
        self.watt_hours = self.pop_int32()/1e4
        self.watt_hours_charged = self.pop_int32()/1e4
        return self


class VescStatus4Msg(CanMsg):

    def __init__(self):

        super(VescStatus4Msg, self).__init__(msg_id=CanIds.CAN_PACKET_STATUS_4)
        self.temp_fet = 0
        self.temp_motor = 0
        self.current_in = 0
        self.pid_pos_now = 0

    def update_vesc_state(self, vesc: VescState):
        vesc.temp_fet = self.temp_fet
        vesc.temp_motor = self.temp_motor
        vesc.current_in = self.current_in
        vesc.pid_pos_now = self.pid_pos_now

    def process_msg(self, data):
        self.set_data(data)
        self.temp_fet = self.pop_int16() / 10.0
        self.temp_motor = self.pop_int16() / 10.0
        self.current_in = self.pop_int16() / 10.0
        self.pid_pos_now = self.pop_int16() / 50.0
        return self


class VescStatus5Msg(CanMsg):

    def __init__(self):

        super(VescStatus5Msg, self).__init__(msg_id=CanIds.CAN_PACKET_STATUS_5)
        self.tacho_value = 0
        self.v_in = 0

    def update_vesc_state(self, vesc: VescState):
        vesc.tacho_value = self.tacho_value
        vesc.v_in = self.v_in

    def process_msg(self, data):
        self.set_data(data)
        self.tacho_value = self.pop_int32()
        self.v_in = self.pop_int16() / 1e1
        return self


class CanMessageHandler:

    def __init__(self):
        self.known_messages = []
        self.known_messages_ids = []

    def register_message(self, msg: CanMsg):
        self.known_messages_ids.append(msg.msg_id)
        self.known_messages.append(msg)


    def process_message(self, msg_id: int, data: bytes):

        if msg_id in self.known_messages_ids:
            return self.known_messages[self.known_messages_ids.index(msg_id)].process_msg(data)
        else:
            raise NameError("Unhandled message id: %i" % msg_id)


class VescCanDriver:

    known_vesc_ids = []
    known_vescs = []  # type: List[VescState]

    def __init__(self):

        # Subscribe to can topics
        rospy.Subscriber("/received_messages", Frame, callback=self.can_cb)

        # Subscribe to cmd topics
        self.current_sub_ = nh.subscribe("commands/motor/current", 10, & VescDriver::currentCallback, this);
        self.brake_sub_ = nh.subscribe("commands/motor/brake", 10, & VescDriver::brakeCallback, this);
        self.speed_sub_ = nh.subscribe("commands/motor/speed", 10, & VescDriver::speedCallback, this);
        self.position_sub_ = nh.subscribe("commands/motor/position", 10, & VescDriver::positionCallback, this);
        self.servo_sub_ = nh.subscribe("commands/servo/position", 10, & VescDriver::servoCallback, this);

        # Make publisher to send can messages
        self.send_can_msg = rospy.Publisher("sent_messages", Frame, queue_size=1)

        # Initialize CAN message handler and add message types
        self.can_msg_handler = CanMessageHandler()

        self.vesc_status_msg = VescStatusMsg()
        self.vesc_status2_msg = VescStatus2Msg()
        self.vesc_status3_msg = VescStatus3Msg()
        self.vesc_status4_msg = VescStatus4Msg()
        self.vesc_status5_msg = VescStatus5Msg()

        self.can_msg_handler.register_message(self.vesc_status_msg)
        self.can_msg_handler.register_message(self.vesc_status2_msg)
        self.can_msg_handler.register_message(self.vesc_status3_msg)
        self.can_msg_handler.register_message(self.vesc_status4_msg)
        self.can_msg_handler.register_message(self.vesc_status5_msg)

    def can_cb(self, msg: Frame):
        """
        Build after comm_can.c
        :param msg:
        :return:
        """
        controller_id = msg.id & 0xFF
        can_msg_id = (msg.id >> 8)

        if controller_id not in self.known_vesc_ids:
            self.known_vesc_ids.append(controller_id)
            self.known_vescs.append(VescState(controller_id))

        controller_idx = self.known_vesc_ids.index(controller_id)
        current_vesc = self.known_vescs[controller_idx]

        try:
            msg = self.can_msg_handler.process_message(msg_id=can_msg_id, data=msg.data)
            msg.update_vesc_state(current_vesc)

        except NameError as e:
            rospy.loginfo(str(e))

        # Debug output
        for vesc in self.known_vescs:
            print(vesc)



if __name__ == '__main__':
    rospy.init_node("vesc_can_driver")
    rospy.loginfo("Starting vesc can driver")
    vesc_can_driver = VescCanDriver()
    rospy.spin()
