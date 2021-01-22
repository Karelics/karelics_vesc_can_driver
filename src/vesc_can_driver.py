#!/usr/bin/env python3

from typing import List, Union
from collections import deque
from threading import Thread
import numpy as np

import rospy

from can_msgs.msg import Frame
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Float32

from karelics_vesc_can_driver.vesc_messages import *
from karelics_vesc_can_driver.vesc import *


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


class VescCanDriver:

    _active_vesc_id = None

    vesc_tool_id = 254
    known_vesc_ids = []
    known_vescs = []  # type: List[Vesc]

    def __init__(self, motor_poles, gear_ratio, cont_current_lim):
        self.motor_poles = motor_poles
        self.gear_ratio = gear_ratio

        # Subscribe to can topics
        rospy.Subscriber("/received_messages", Frame, callback=self.can_cb)

        # Make publisher to send can messages
        self.send_can_msg_pub = rospy.Publisher("sent_messages", Frame, queue_size=1)

        # Initialize CAN message handler and add message types
        self.can_msg_handler = CanMessageHandler()
        self.can_msg_handler.set_append_msg_ids(CanIds.CAN_PACKET_FILL_RX_BUFFER, CanIds.CAN_PACKET_FILL_RX_BUFFER_LONG)
        self.can_msg_handler.set_process_buffer_msg_id(CanIds.CAN_PACKET_PROCESS_RX_BUFFER)

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

        # Set Current monitor to ensure battery health
        self.current_monitor = MonitorOvercurrent(cont_current_lim)

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
        Callback for the /recieved_msg topic published by the roscan_bridge.
        :param msg: The Can message
        :return: none

        In this callback the message ID is decoded into a vesc_id and can_msg_id.
        The can message is then decoded and the data is fed into Vesc object holding the
        state of that specific Vesc controller.

        """
        [vesc_id, can_msg_id] = CanMsg.decode_frame_id(msg)

        # if vesc_id != 1:
            # print(vesc_id)
            # print(msg)
            # return

        # Check if we already know the vesc this message comes from, if not add it to the known vesc

        if vesc_id != self.vesc_tool_id:
            if vesc_id not in self.known_vesc_ids:
                self.known_vesc_ids.append(vesc_id)
                self.known_vescs.append(Vesc(vesc_id=vesc_id,
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
            rospy.loginfo(str(e))

        # Publish the current status of the vescs in to ros world
        # TODO should we process tick only for the current_vesc?
        for vesc in self.known_vescs:
            vesc.tick()

        self.current_monitor.tick(self.known_vescs)


class TimeDeque:
    """ Implements a deque, but instead of giving maxlen to it, you can give it max_seconds.
        Only the data from the past max_seconds is stored. """

    def __init__(self, max_seconds=10):
        self.time_deque = deque()
        self.max_seconds = max_seconds
        self.pub = rospy.Publisher("/status/vescs/median_current", Float32, queue_size=1)

    def _remove_old_data(self):
        now = rospy.get_time()
        while self.time_deque:
            t = self.time_deque[0][0]
            if now - t > self.max_seconds:
                self.time_deque.popleft()
            else:
                break

    def append(self, value):
        self._remove_old_data()
        self.time_deque.append([rospy.get_time(), value])

    def get_median(self):
        self._remove_old_data()
        total_currents = []
        for _, v in self.time_deque:
            total_currents.append(v)
        med = self.median(total_currents) #np.median(self.time_deque, axis=0)[1]
        msg_data = Float32()
        msg_data.data = med
        self.pub.publish(med)
        return med

    @staticmethod
    def median(lst):
        n = len(lst)
        s = sorted(lst)
        return (sum(s[n // 2 - 1:n // 2 + 1]) / 2.0, s[n // 2])[n % 2] if n else None

# TODO MonitorBatteryMaxDischarge
class MonitorOvercurrent:
    def __init__(self, cont_current_lim):
        self.cont_current_lim = cont_current_lim
        self.total_current_pub = rospy.Publisher("vescs/total_current", Float32, queue_size=1)
        self.time_deque = TimeDeque()
        #self.prev_oc_time = None
        #self.prev_total_curr = 0

        self.allowed_overcurrent_time = 2  # Time in seconds for how long the overcurrent can be applied
        self.time_deque = TimeDeque(max_seconds=self.allowed_overcurrent_time*2)
        #self.allowed_peaks = 2 * self.allowed_overcurrent_time  # How many peaks do we allow withing allowed_overcurrent_time
        self.overcurrent_cooldown = 5  # in seconds

        self.fatal_current = False

    def _handle_safety_limit(self, total_current):
        self.time_deque.append(total_current)
        median = self.time_deque.get_median()  # Checking if we motors have drawn maximum current over 2 second total within 4 seconds time window
        if median >= self.cont_current_lim and not self.fatal_current:
            self._set_overcurrent()


    # def _handle_safety_limit(self, total_current):
    #     now = rospy.get_time()
    #     num_peaks = self.get_peaks_from_time_window()
    #
    #     # First time exceeding the overcurrent limit
    #     if total_current >= self.cont_current_lim > self.prev_total_curr:
    #         self.peaks.append(now)
    #
    #     # Exceeded the overcurrent limit on last tick as well
    #     elif total_current >= self.cont_current_lim:
    #         peak_length = now - self.peaks[-1]
    #         if peak_length > self.allowed_overcurrent_time:
    #             rospy.logdebug("Motor current has been over {} for {} seconds".format(self.cont_current_lim,
    #                                                                                   self.allowed_overcurrent_time))
    #             self._set_overcurrent()
    #
    #     elif num_peaks > self.allowed_peaks:
    #         self._set_overcurrent()
    #         rospy.logdebug("Motor current has peaked over {}A for {} times within {} seconds"
    #                        "".format(self.cont_current_lim, num_peaks, self.allowed_overcurrent_time))
    #
    #     # TODO Check max peaks
    #     self.prev_total_curr = total_current

    def _set_overcurrent(self):
        rospy.logerr("Motors are applying too high total current. Cutting off.")
        self.fatal_current = True
        thread = Thread(target=self._set_motor_cooldown, args=())
        thread.daemon = True
        thread.start()

    def _set_motor_cooldown(self):
        rospy.logerr("Starting {} seconds cooldown.".format(self.overcurrent_cooldown))
        rospy.sleep(self.overcurrent_cooldown)
        rospy.loginfo("Motors are now ready for operation.")
        self.fatal_current = False

    # def get_peaks_from_time_window(self):
    #     num_peaks = 0
    #     now = rospy.get_time()
    #     for peak_time in reversed(self.peaks):
    #         if now - peak_time < self.allowed_overcurrent_time:
    #             num_peaks += 1
    #         else:
    #             break
    #     return num_peaks

    def tick(self, vescs):
        total_current = 0
        for vesc in vescs:
            total_current += vesc.current
        current_msg = Float32()
        current_msg.data = total_current
        self.total_current_pub.publish(current_msg)
        self._handle_safety_limit(total_current)


    def is_safe(self):
        """ Returns True if it's not safe to rotate motors without exceeding battery current limits """
        return not self.fatal_current


if __name__ == '__main__':
    rospy.init_node("vesc_can_driver", log_level=rospy.DEBUG)
    rospy.loginfo("Starting vesc can driver")

    motor_poles = rospy.get_param("~motor_poles")
    gear_ratio = rospy.get_param("~gear_ratio")
    cont_current_lim = rospy.get_param("~continuous_current_limit")
    vesc_can_driver = VescCanDriver(motor_poles, gear_ratio, cont_current_lim)
    rospy.spin()
