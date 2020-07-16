import struct
from abc import ABCMeta
from enum import IntEnum

from can_msgs.msg import Frame

from karelics_vesc_can_driver.vesc import Vesc


class CanIds(IntEnum):
    """
    Can id's from: 
    https://github.com/vedderb/vesc_tool/blob/87565a40802707bcdd1432f4f470b7a4364a9a82/datatypes.h
    """
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


class CanMsg(metaclass=ABCMeta):
    """
    Interface defining the common fuctions for encoding and decoding can messages from the VESC.
    
    This class should not be used directly but should always be inherited from.
    
    """

    def __init__(self, msg_id):
        self.msg_id = msg_id
        self.in_buffer = bytes()
        self.out_buffer = bytearray()
        self.pointer = 0
        self.on_update_func = None

    """
    Decoding
    """
    def pop_int32(self):
        data = struct.unpack(">i", self.in_buffer[self.pointer:self.pointer + 4])[0]
        self.pointer += 4
        return data

    def pop_int16(self):
        data = struct.unpack(">h", self.in_buffer[self.pointer:self.pointer + 2])[0]
        self.pointer += 2
        return data
    
    """
    Encoding
    """    
    def encode_int32(self, value):
        for b in struct.pack(">i", value):
            self.out_buffer.append(b)

    def encode_uint32(self, value):
        for b in struct.pack(">I", value):
            self.out_buffer.append(b)

    def encode_float32(self, value):
        for b in struct.pack(">f", value):
            self.out_buffer.append(b)
            
    def set_data(self, data: bytes):
        self.pointer = 0
        self.in_buffer = data

    def process_msg(self, data):
        """
        This function should be implemented in the derived class.
        """
        pass

    def start_msg(self):
        self.out_buffer = bytearray()

    @staticmethod
    def decode_frame_id(frame: Frame):
        vesc_id = frame.id & 0xFF
        can_msg_id = (frame.id >> 8)
        return [vesc_id, can_msg_id]

    def get_encoded_msg_id(self, controller_id):

        return controller_id | (self.msg_id << 8)

    def get_can_msg(self, vesc_id):

        can_msg = Frame()
        can_msg.is_extended = True
        can_msg.id = self.get_encoded_msg_id(vesc_id)
        can_msg.data = self.get_encoded_msg()
        can_msg.dlc = len(self.out_buffer)

        return can_msg

    def get_encoded_msg(self):
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

    def update_vesc_state(self, vesc: Vesc):
        vesc.erpm = self.erpm
        vesc.current = self.current
        vesc.duty_cycle = self.duty_cycle


class VescStatus2Msg(CanMsg):

    def __init__(self):

        super(VescStatus2Msg, self).__init__(msg_id=CanIds.CAN_PACKET_STATUS_2)
        self.amp_hours = 0
        self.amp_hours_charged = 0

    def update_vesc_state(self, vesc: Vesc):
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

    def update_vesc_state(self, vesc: Vesc):
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

    def update_vesc_state(self, vesc: Vesc):
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

    def update_vesc_state(self, vesc: Vesc):
        vesc.tacho_value = self.tacho_value
        vesc.v_in = self.v_in

    def process_msg(self, data):
        self.set_data(data)
        self.tacho_value = self.pop_int32()
        self.v_in = self.pop_int16() / 1e1
        return self


class VesSetDuty(CanMsg):

    def __init__(self, dutycycle=0):
        super(VesSetDuty, self).__init__(msg_id=CanIds.CAN_PACKET_SET_DUTY)
        self.duty_cycle = dutycycle

    def get_encoded_msg(self):
        self.start_msg()
        self.encode_int32(int(self.duty_cycle*100000))
        return self.out_buffer


class VesSetCurrent(CanMsg):

    def __init__(self, current=0):
        super(VesSetCurrent, self).__init__(msg_id=CanIds.CAN_PACKET_SET_CURRENT)
        self.current = current

    def get_encoded_msg(self):
        self.start_msg()
        self.encode_int32(int(self.current*1000))
        return self.out_buffer


class VesSetBrakeCurrent(CanMsg):

    def __init__(self, current=0):
        super(VesSetBrakeCurrent, self).__init__(msg_id=CanIds.CAN_PACKET_SET_CURRENT_BRAKE)
        self.current = current

    def get_encoded_msg(self):
        self.start_msg()
        self.encode_int32(int(self.current*1000))
        return self.out_buffer


class VesSetRPM(CanMsg):

    def __init__(self, rpm):
        super(VesSetRPM, self).__init__(msg_id=CanIds.CAN_PACKET_SET_RPM)
        self.rpm = rpm

    def get_encoded_msg(self):
        self.start_msg()
        self.encode_int32(int(self.rpm))
        return self.out_buffer


class VesSetPos(CanMsg):

    def __init__(self, pos=0):
        super(VesSetPos, self).__init__(msg_id=CanIds.CAN_PACKET_SET_POS)
        self.pos = pos

    def get_encoded_msg(self):
        self.start_msg()
        self.encode_int32(int(self.pos))
        return self.out_buffer


class VesSetCurrentRel(CanMsg):
    """
    Set current relative to the minimum and maximum current limits.
    range [-1.0 1.0]
    """

    def __init__(self, current=0):
        super(VesSetCurrentRel, self).__init__(msg_id=CanIds.CAN_PACKET_SET_CURRENT_REL)
        self.current = current

    def get_encoded_msg(self):
        self.start_msg()
        self.encode_float32(int(self.current))
        return self.out_buffer
