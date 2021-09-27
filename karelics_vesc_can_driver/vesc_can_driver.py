#!/usr/bin/env python3

from typing import List, Union
import sys

import rclpy
from rclpy.node import Node

from can_msgs.msg import Frame

from karelics_vesc_can_driver.vesc_messages import *
from karelics_vesc_can_driver.vesc import *
from karelics_vesc_can_driver.max_current_monitor import MonitorMaxCurrent


class CanMessageHandler:

    _append_msg_id_short = -1
    _append_msg_id_long = -1
    _process_buffer_msg_id = -1

    def __init__(self):
        self.known_messages = []
        self.known_messages_ids = []
        self._input_buffer = bytearray()

    def set_append_msg_ids(self, msg_id_short: CanIds, msg_id_long: CanIds):
        self._append_msg_id_short = msg_id_short
        self._append_msg_id_long = msg_id_long

    def set_process_buffer_msg_id(self, msg_id: CanIds):
        self._process_buffer_msg_id = msg_id

    def register_message(self, msg: CanMsg):
        self.known_messages_ids.append(msg.msg_id)
        self.known_messages.append(msg)

    def process_message(self, msg_id: int, data: bytes) -> Union[None, CanMsg]:

        # todo: add check to see if the buffer position is strictly increasing
        if msg_id == self._append_msg_id_short:
            # buffer position is a short so remove first bite
            self._input_buffer.extend(data[1:])
            return None

        if msg_id == self._append_msg_id_long:
            # Buffer position is a long so remove the first two bytes
            self._input_buffer.extend(data[2:])
            return None

        # todo: add check to match length and crc
        if msg_id == self._process_buffer_msg_id:
            # payload[0] = char(254); // vesc tool node ID
            # payload[1] = char(0); // process
            # payload[2] = char(len >> 8);
            # payload[3] = char(len & 0xFF);
            # payload[4] = char(crc >> 8);
            # payload[5] = char(crc & 0xFF);
            msg_id = self._input_buffer[0]
            data = self._input_buffer[1:]
            self._input_buffer = bytearray()

        # print(" ".join("x%02x" % i for i in data))

        if msg_id in self.known_messages_ids:
            return self.known_messages[self.known_messages_ids.index(msg_id)].process_msg(data)
        else:
            raise NameError("Unhandled message id: %i" % msg_id)


class VescCanDriver(Node):

    _active_vesc_id = None

    vesc_tool_id: int = 254
    known_vesc_ids: List[int] = []
    known_vescs: List[Vesc] = []

    def __init__(self):
        super().__init__("vesc_can_driver")

        self.declare_parameter('motor_poles')
        self.motor_poles = int(self.get_parameter('motor_poles').value)

        self.declare_parameter('gear_ratio')
        self.gear_ratio = float(self.get_parameter('gear_ratio').value)

        self.declare_parameter('continuous_current_limit')
        self.cont_current_lim = int(self.get_parameter('continuous_current_limit').value)

        self.get_logger().info('Starting vesc can driver')

        # Subscribe to can topics
        self.create_subscription(Frame, "/from_can_bus", self.can_cb, qos_profile=10)

        # Make publisher to send can messages
        self.send_can_msg_pub = self.create_publisher(Frame, "/to_can_bus", qos_profile=1)

        # Initialize CAN message handler and add message types
        self.can_msg_handler = CanMessageHandler()
        self.can_msg_handler.set_append_msg_ids(CanIds.CAN_PACKET_FILL_RX_BUFFER, CanIds.CAN_PACKET_FILL_RX_BUFFER_LONG)
        self.can_msg_handler.set_process_buffer_msg_id(CanIds.CAN_PACKET_PROCESS_RX_BUFFER)

        # incoming message types from the VESC
        self.vesc_status_msg = VescStatusMsg()
        self.vesc_status2_msg = VescStatus2Msg()
        self.vesc_status3_msg = VescStatus3Msg()
        self.vesc_status4_msg = VescStatus4Msg()
        self.vesc_status5_msg = VescStatus5Msg()
        self.vesc_imu_msg = VescIMUData()
        self.can_msg_handler.register_message(self.vesc_status_msg)
        self.can_msg_handler.register_message(self.vesc_status2_msg)
        self.can_msg_handler.register_message(self.vesc_status3_msg)
        self.can_msg_handler.register_message(self.vesc_status4_msg)
        self.can_msg_handler.register_message(self.vesc_status5_msg)
        self.can_msg_handler.register_message(self.vesc_imu_msg)

        # outgoing message types to the VESC
        self.vesc_set_duty_msg = VescSetDuty()
        self.vesc_set_current_msg = VescSetCurrent()
        self.vesc_set_brake_current_msg = VescSetBrakeCurrent()
        self.vesc_set_rpm_msg = VescSetRPM()
        self.vesc_set_pos_msg = VescSetPos()
        self.vesc_set_current_rel_msg = VescSetCurrentRel()
        self.vesc_set_hb_msg = VescSetHandbrakeCurrent()
        self.vesc_get_imu_data_msg = VescGetImuData()
        self.can_msg_handler.register_message(self.vesc_set_duty_msg)
        self.can_msg_handler.register_message(self.vesc_set_current_msg)
        self.can_msg_handler.register_message(self.vesc_set_brake_current_msg)
        self.can_msg_handler.register_message(self.vesc_set_rpm_msg)
        self.can_msg_handler.register_message(self.vesc_set_pos_msg)
        self.can_msg_handler.register_message(self.vesc_set_current_rel_msg)
        self.can_msg_handler.register_message(self.vesc_set_hb_msg)
        self.can_msg_handler.register_message(self.vesc_get_imu_data_msg)

        # Set Current monitor to ensure battery health
        self.current_monitor = MonitorMaxCurrent(node=self, cont_current_lim=self.cont_current_lim)

    def aquire_vesc_tool_id_lock(self, vesc_id):
        if self._active_vesc_id and self._active_vesc_id != vesc_id:
            return False
        else:
            self._active_vesc_id = vesc_id
            return True

    def release_vesc_tool_id_lock(self):
        self._active_vesc_id = None

    def can_cb(self, msg: Frame):
        """
        Callback for the /from_can_bus topic published by the ros2_socketcan package.
        :param msg: The Can message
        :return: none
        In this callback the message ID is decoded into a vesc_id and can_msg_id.
        The can message is then decoded and the data is fed into Vesc object holding the
        state of that specific Vesc controller.
        """
        [vesc_id, can_msg_id] = CanMsg.decode_frame_id(msg)

        # Check if we already know the vesc this message comes from, if not add it to the known vesc
        if vesc_id != self.vesc_tool_id:
            if vesc_id not in self.known_vesc_ids:
                self.known_vesc_ids.append(vesc_id)
                self.known_vescs.append(Vesc(node=self,
                                             vesc_id=vesc_id,
                                             send_function=self.send_can_msg_pub.publish,
                                             lock_function=self.aquire_vesc_tool_id_lock,
                                             release_function=self.release_vesc_tool_id_lock,
                                             motor_poles=self.motor_poles,
                                             gear_ratio=self.gear_ratio,
                                             current_monitor=self.current_monitor))
        else:
            if self._active_vesc_id:
                vesc_id = self._active_vesc_id
            else:
                return

        # Get the index of the current vesc
        controller_idx = self.known_vesc_ids.index(vesc_id)
        current_vesc = self.known_vescs[controller_idx]

        # Decode message and update corresponding vesc object
        try:
            can_msg = self.can_msg_handler.process_message(msg_id=can_msg_id, data=msg.data)
            if can_msg:
                can_msg.update_vesc_state(current_vesc)
        except NameError as e:
            self.get_logger().info(str(e))

        # Publish the current status of the current vesc in to ros world
        current_vesc.tick()

        self.current_monitor.tick(self.known_vescs)


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    karelics_vesc_can_driver_node = VescCanDriver()
    rclpy.spin(karelics_vesc_can_driver_node)
    karelics_vesc_can_driver_node.destroy_node()
    rclpy.shutdown()
