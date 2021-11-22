import struct
from abc import ABCMeta
from enum import IntEnum

from can_msgs.msg import Frame


class ComPacketID(IntEnum):
    COMM_FW_VERSION = 0
    COMM_JUMP_TO_BOOTLOADER = 1
    COMM_ERASE_NEW_APP = 2
    COMM_WRITE_NEW_APP_DATA = 3
    COMM_GET_VALUES = 4
    COMM_SET_DUTY = 5
    COMM_SET_CURRENT = 6
    COMM_SET_CURRENT_BRAKE = 7
    COMM_SET_RPM = 8
    COMM_SET_POS = 9
    COMM_SET_HANDBRAKE = 10
    COMM_SET_DETECT = 11
    COMM_SET_SERVO_POS = 12
    COMM_SET_MCCONF = 13
    COMM_GET_MCCONF = 14
    COMM_GET_MCCONF_DEFAULT = 15
    COMM_SET_APPCONF = 16
    COMM_GET_APPCONF = 17
    COMM_GET_APPCONF_DEFAULT = 18
    COMM_SAMPLE_PRINT = 19
    COMM_TERMINAL_CMD = 20
    COMM_PRINT = 21
    COMM_ROTOR_POSITION = 22
    COMM_EXPERIMENT_SAMPLE = 23
    COMM_DETECT_MOTOR_PARAM = 24
    COMM_DETECT_MOTOR_R_L = 25
    COMM_DETECT_MOTOR_FLUX_LINKAGE = 26
    COMM_DETECT_ENCODER = 27
    COMM_DETECT_HALL_FOC = 28
    COMM_REBOOT = 29
    COMM_ALIVE = 30
    COMM_GET_DECODED_PPM = 31
    COMM_GET_DECODED_ADC = 32
    COMM_GET_DECODED_CHUK = 33
    COMM_FORWARD_CAN = 34
    COMM_SET_CHUCK_DATA = 35
    COMM_CUSTOM_APP_DATA = 36
    COMM_NRF_START_PAIRING = 37
    COMM_GPD_SET_FSW = 38
    COMM_GPD_BUFFER_NOTIFY = 39
    COMM_GPD_BUFFER_SIZE_LEFT = 40
    COMM_GPD_FILL_BUFFER = 41
    COMM_GPD_OUTPUT_SAMPLE = 42
    COMM_GPD_SET_MODE = 43
    COMM_GPD_FILL_BUFFER_INT8 = 44
    COMM_GPD_FILL_BUFFER_INT16 = 45
    COMM_GPD_SET_BUFFER_INT_SCALE = 46
    COMM_GET_VALUES_SETUP = 47
    COMM_SET_MCCONF_TEMP = 48
    COMM_SET_MCCONF_TEMP_SETUP = 49
    COMM_GET_VALUES_SELECTIVE = 50
    COMM_GET_VALUES_SETUP_SELECTIVE = 51
    COMM_EXT_NRF_PRESENT = 52
    COMM_EXT_NRF_ESB_SET_CH_ADDR = 53
    COMM_EXT_NRF_ESB_SEND_DATA = 54
    COMM_EXT_NRF_ESB_RX_DATA = 55
    COMM_EXT_NRF_SET_ENABLED = 56
    COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP = 57
    COMM_DETECT_APPLY_ALL_FOC = 58
    COMM_JUMP_TO_BOOTLOADER_ALL_CAN = 59
    COMM_ERASE_NEW_APP_ALL_CAN = 60
    COMM_WRITE_NEW_APP_DATA_ALL_CAN = 61
    COMM_PING_CAN = 62
    COMM_APP_DISABLE_OUTPUT = 63
    COMM_TERMINAL_CMD_SYNC = 64
    COMM_GET_IMU_DATA = 65
    COMM_BM_CONNECT = 66
    COMM_BM_ERASE_FLASH_ALL = 67
    COMM_BM_WRITE_FLASH = 68
    COMM_BM_REBOOT = 69
    COMM_BM_DISCONNECT = 70
    COMM_BM_MAP_PINS_DEFAULT = 71
    COMM_BM_MAP_PINS_NRF5X = 72
    COMM_ERASE_BOOTLOADER = 73
    COMM_ERASE_BOOTLOADER_ALL_CAN = 74
    COMM_PLOT_INIT = 75
    COMM_PLOT_DATA = 76
    COMM_PLOT_ADD_GRAPH = 77
    COMM_PLOT_SET_GRAPH = 78
    COMM_GET_DECODED_BALANCE = 79
    COMM_BM_MEM_READ = 80
    COMM_WRITE_NEW_APP_DATA_LZO = 81
    COMM_WRITE_NEW_APP_DATA_ALL_CAN_LZO = 82
    COMM_BM_WRITE_FLASH_LZO = 83
    COMM_SET_CURRENT_REL = 84
    COMM_CAN_FWD_FRAME = 85
    COMM_SET_BATTERY_CUT = 86
    COMM_SET_BLE_NAME = 87
    COMM_SET_BLE_PIN = 88
    COMM_SET_CAN_MODE = 89
    COMM_GET_IMU_CALIBRATION = 90


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
    Interface defining the common functions for encoding and decoding can messages from the VESC.

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

    def pop_uint16(self):
        data = struct.unpack(">H", self.in_buffer[self.pointer:self.pointer + 2])[0]
        self.pointer += 2
        return data

    def pop_float32(self):
        data = struct.unpack(">f", self.in_buffer[self.pointer:self.pointer + 4])[0]
        self.pointer += 4
        return data

    """
    Encoding
    """

    def encode_int32(self, value):
        for b in struct.pack(">i", value):
            self.out_buffer.append(b)

    def encode_int64(self, value):
        for b in struct.pack(">q", value):
            self.out_buffer.append(b)

    def encode_uint32(self, value):
        for b in struct.pack(">I", value):
            self.out_buffer.append(b)

    def encode_int16(self, value):
        for b in struct.pack(">h", value):
            self.out_buffer.append(b)

    def encode_uint16(self, value):
        for b in struct.pack(">H", value):
            self.out_buffer.append(b)

    def encode_int8(self, value):
        for b in struct.pack(">b", value):
            self.out_buffer.append(b)

    def encode_uint8(self, value):
        for b in struct.pack(">B", value):
            self.out_buffer.append(b)

    def encode_float32(self, value):
        for b in struct.pack(">f", value):
            self.out_buffer.append(b)

    def encode_float64(self, value):
        for b in struct.pack(">d", value):
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
        self.get_encoded_msg()
        can_msg.dlc = len(self.out_buffer)
        if len(self.out_buffer) < 8:
            # apply 0 padding to the out buffer if the encoded data is less that 8 bytes
            no_of_bytes_to_fill = 8 - len(self.out_buffer)
            if no_of_bytes_to_fill == 1:
                self.encode_int8(0)
            elif no_of_bytes_to_fill == 2:
                self.encode_int16(0)
            elif no_of_bytes_to_fill == 3:
                self.encode_int16(0)
                self.encode_int8(0)
            elif no_of_bytes_to_fill == 4:
                self.encode_int32(0)
            elif no_of_bytes_to_fill == 5:
                self.encode_int32(0)
                self.encode_int8(0)
            elif no_of_bytes_to_fill == 6:
                self.encode_int32(0)
                self.encode_int16(0)
            elif no_of_bytes_to_fill == 7:
                self.encode_int32(0)
                self.encode_int16(0)
                self.encode_int8(0)
            elif no_of_bytes_to_fill == 8:  # edge case should never happen but just to be sure
                self.encode_float64(float(0))

        can_msg.data = self.out_buffer

        return can_msg

    def get_encoded_msg(self):
        pass

    def update_vesc_state(self, vesc):
        pass


class ComMsg(CanMsg):
    def __init__(self, msg_id: ComPacketID):
        super(ComMsg, self).__init__(msg_id)


# Incoming messages
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

    def update_vesc_state(self, vesc):
        vesc.erpm = self.erpm
        vesc.current = self.current
        vesc.duty_cycle = self.duty_cycle


class VescStatus2Msg(CanMsg):

    def __init__(self):
        super(VescStatus2Msg, self).__init__(msg_id=CanIds.CAN_PACKET_STATUS_2)
        self.amp_hours = 0
        self.amp_hours_charged = 0

    def update_vesc_state(self, vesc):
        vesc.amp_hours = self.amp_hours
        vesc.amp_hours_charged = self.amp_hours_charged

    def process_msg(self, data):
        self.set_data(data)
        self.amp_hours = self.pop_int32() / 1e4
        self.amp_hours_charged = self.pop_int32() / 1e4
        return self


class VescStatus3Msg(CanMsg):

    def __init__(self):
        super(VescStatus3Msg, self).__init__(msg_id=CanIds.CAN_PACKET_STATUS_3)
        self.watt_hours = 0
        self.watt_hours_charged = 0

    def update_vesc_state(self, vesc):
        vesc.watt_hours = self.watt_hours
        vesc.watt_hours_charged = self.watt_hours_charged

    def process_msg(self, data):
        self.set_data(data)
        self.watt_hours = self.pop_int32() / 1e4
        self.watt_hours_charged = self.pop_int32() / 1e4
        return self


class VescStatus4Msg(CanMsg):

    def __init__(self):
        super(VescStatus4Msg, self).__init__(msg_id=CanIds.CAN_PACKET_STATUS_4)
        self.temp_fet = 0
        self.temp_motor = 0
        self.current_in = 0
        self.pid_pos_now = 0

    def update_vesc_state(self, vesc):
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

    def update_vesc_state(self, vesc):
        vesc.tacho_value = self.tacho_value
        vesc.v_in = self.v_in

    def process_msg(self, data):
        self.set_data(data)
        self.tacho_value = self.pop_int32()
        self.v_in = self.pop_int16() / 1e1
        return self


class VescIMUData(ComMsg):

    def __init__(self):
        super(VescIMUData, self).__init__(msg_id=ComPacketID.COMM_GET_IMU_DATA)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.accX = 0
        self.accY = 0
        self.accZ = 0
        self.gyroX = 0
        self.gyroY = 0
        self.gyroZ = 0
        self.magX = 0
        self.magY = 0
        self.magZ = 0
        self.q0 = 0
        self.q1 = 0
        self.q2 = 0
        self.q3 = 0

    def process_msg(self, data):
        self.set_data(data)
        mask = self.pop_uint16()
        # Only unpack if the data is set
        if mask & (1 << 0):
            self.roll = self.pop_float32()
        if mask & (1 << 1):
            self.pitch = self.pop_float32()
        if mask & (1 << 2):
            self.yaw = self.pop_float32()
        if mask & (1 << 3):
            self.accX = self.pop_float32()
        if mask & (1 << 4):
            self.accY = self.pop_float32()
        if mask & (1 << 5):
            self.accZ = self.pop_float32()
        if mask & (1 << 6):
            self.gyroX = self.pop_float32()
        if mask & (1 << 7):
            self.gyroY = self.pop_float32()
        if mask & (1 << 8):
            self.gyroZ = self.pop_float32()
        if mask & (1 << 9):
            self.magX = self.pop_float32()
        if mask & (1 << 10):
            self.magY = self.pop_float32()
        if mask & (1 << 11):
            self.magZ = self.pop_float32()
        if mask & (1 << 12):
            self.q0 = self.pop_float32()
        if mask & (1 << 13):
            self.q1 = self.pop_float32()
        if mask & (1 << 14):
            self.q2 = self.pop_float32()
        if mask & (1 << 15):
            self.q3 = self.pop_float32()

        return self

    def update_vesc_state(self, vesc):
        # This is a multi package message so tell the vesc that it is not waiting
        vesc.data_recieved()
        # Update state
        vesc.roll = self.roll
        vesc.pitch = self.pitch
        vesc.yaw = self.yaw
        vesc.accX = self.accX
        vesc.accY = self.accY
        vesc.accZ = self.accZ
        vesc.gyroX = self.gyroX
        vesc.gyroY = self.gyroY
        vesc.gyroZ = self.gyroZ
        vesc.magX = self.magX
        vesc.magY = self.magY
        vesc.magZ = self.magZ
        vesc.q0 = self.q0
        vesc.q1 = self.q1
        vesc.q2 = self.q2
        vesc.q3 = self.q3


# Out going messages
class VescSetDuty(CanMsg):

    def __init__(self, dutycycle=0):
        super(VescSetDuty, self).__init__(msg_id=CanIds.CAN_PACKET_SET_DUTY)
        self.duty_cycle = dutycycle

    def get_encoded_msg(self):
        self.start_msg()
        self.encode_int32(int(self.duty_cycle * 1e5))
        return self.out_buffer


class VescSetCurrent(CanMsg):

    def __init__(self, current=0):
        super(VescSetCurrent, self).__init__(msg_id=CanIds.CAN_PACKET_SET_CURRENT)
        self.current = current

    def get_encoded_msg(self):
        self.start_msg()
        self.encode_int32(int(self.current * 1e3))
        return self.out_buffer


class VescSetBrakeCurrent(CanMsg):

    def __init__(self, current=0):
        super(VescSetBrakeCurrent, self).__init__(msg_id=CanIds.CAN_PACKET_SET_CURRENT_BRAKE)
        self.current = current

    def get_encoded_msg(self):
        self.start_msg()
        self.encode_int32(int(self.current * 1e3))
        return self.out_buffer


class VescSetHandbrakeCurrent(CanMsg):

    def __init__(self, current=0):
        super(VescSetHandbrakeCurrent, self).__init__(msg_id=CanIds.CAN_PACKET_SET_CURRENT_HANDBRAKE)
        self.current = current

    def get_encoded_msg(self):
        self.start_msg()
        self.encode_int32(int(self.current * 1e3))
        return self.out_buffer


class VescSetRPM(CanMsg):

    def __init__(self, rpm=0):
        super(VescSetRPM, self).__init__(msg_id=CanIds.CAN_PACKET_SET_RPM)
        self.rpm = rpm

    def get_encoded_msg(self):
        self.start_msg()
        self.encode_int32(int(self.rpm))
        return self.out_buffer


class VescSetPos(CanMsg):

    def __init__(self, pos=0):
        super(VescSetPos, self).__init__(msg_id=CanIds.CAN_PACKET_SET_POS)
        self.pos = pos

    def get_encoded_msg(self):
        self.start_msg()
        self.encode_int32(int(self.pos * 1e6))
        return self.out_buffer


class VescSetCurrentRel(CanMsg):
    """
    Set current relative to the minimum and maximum current limits.
    range [-1.0 1.0]
    """

    def __init__(self, current=0):
        super(VescSetCurrentRel, self).__init__(msg_id=CanIds.CAN_PACKET_SET_CURRENT_REL)
        self.current = current

    def get_encoded_msg(self):
        self.start_msg()
        self.encode_int32(int(self.current * 1e5))
        return self.out_buffer


class VescGetImuData(CanMsg):

    def __init__(self):
        super(VescGetImuData, self).__init__(msg_id=CanIds.CAN_PACKET_PROCESS_SHORT_BUFFER)

    def get_encoded_msg(self):
        self.start_msg()
        self.encode_uint8(0xFE)  # VescTool ID
        self.encode_uint8(0x0)  # Not Used
        self.encode_uint8(ComPacketID.COMM_GET_IMU_DATA)
        self.encode_uint16(0xFFFF)
        return self.out_buffer
