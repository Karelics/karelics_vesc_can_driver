#!/usr/bin/env python3

from typing import List

import rospy

from can_msgs.msg import Frame

from karelics_vesc_can_driver.vesc_messages import *
from karelics_vesc_can_driver.vesc import *


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
    known_vescs = []  # type: List[Vesc]

    def __init__(self):

        # Subscribe to can topics
        rospy.Subscriber("/received_messages", Frame, callback=self.can_cb)

        # Make publisher to send can messages
        self.send_can_msg_pub = rospy.Publisher("sent_messages", Frame, queue_size=1)

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
        Callback for the /recieved_msg topic published by the roscan_bridge.
        :param msg: The Can message
        :return: none

        In this callback the message ID is decoded into a vesc_id and can_msg_id.
        The can message is then decoded and the data is fed into Vesc object holding the
        state of that specific Vesc controller.

        """
        [vesc_id, can_msg_id] = CanMsg.decode_frame_id(msg)

        # Check if we already know the vesc this message comes from, if not add it to the known vesc
        if vesc_id not in self.known_vesc_ids:
            self.known_vesc_ids.append(vesc_id)
            self.known_vescs.append(Vesc(vesc_id=vesc_id,
                                         send_function=self.send_can_msg_pub.publish))

        # Get the index of the current vesc
        controller_idx = self.known_vesc_ids.index(vesc_id)
        current_vesc = self.known_vescs[controller_idx]

        # Decode message and update corresponding vesc object
        try:
            msg = self.can_msg_handler.process_message(msg_id=can_msg_id, data=msg.data)
            msg.update_vesc_state(current_vesc)

        except NameError as e:
            rospy.loginfo(str(e))

        # Publish the current status of the vescs in to ros world
        for vesc in self.known_vescs:
            vesc.publish_status()


if __name__ == '__main__':
    rospy.init_node("vesc_can_driver")
    rospy.loginfo("Starting vesc can driver")
    vesc_can_driver = VescCanDriver()
    rospy.spin()
