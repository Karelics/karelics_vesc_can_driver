#!/usr/bin/env python3
from functools import partial
import re
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from karelics_vesc_can_driver.msg import VescStatus


class BatteryStatus(Node):
    def __init__(self):
        super().__init__("battery_status")

        self.battery_pub = self.create_publisher(BatteryState, "/battery", qos_profile=1)
        self.battery_pub_timer = self.create_timer(1.0, self.publish_battery_percentage)

        self.vesc_status_subscribers = dict()
        self.vesc_voltages = dict()

        self.current_battery_voltage = None

        # voltage, percentage. All the other voltage values between these key-values
        # are linearly calculated
        self.key_voltages = [[54.10, 1.0],
                             [48, 0.6],
                             [45.8, 0.2],
                             [44.2, 0.05],
                             [41.8, 0.02],
                             [40, 0]]

    def vesc_status_cb(self, data, vesc):
        # TODO
        return

    def get_vesc_status_topics(self):
        topics_and_types = self.get_publisher_names_and_types_by_node('karelics_vesc_can_driver', '/', no_demangle=False)
        vesc_status_topics = []
        string_topic_format = re.compile(r'/vesc_(\d+)/status')  # RegEx template to find all existing vesc status topics

        for topic_tuple in topics_and_types:
            if string_topic_format.match(topic_tuple[0]):
                vesc_status_topics.append(topic_tuple[0])

        return vesc_status_topics

    @staticmethod
    def vesc_from_topic(topic):
        return list(filter(None, topic.split(sep='/')))[0]

    def get_vesc_status_subscribers(self, topics_list):
        # no new subscribers to add
        if len(self.vesc_status_subscribers) == len(topics_list):
            return

        # create all the subs
        if len(self.vesc_status_subscribers) == 0:
            for topic in topics_list:
                vesc_from_current_topic = self.vesc_from_topic(topic)
                status_sub = self.create_subscription(VescStatus, topic, partial(self.vesc_status_cb,
                                                                                 vesc=vesc_from_current_topic),
                                                      qos_profile=1)
                self.vesc_status_subscribers[vesc_from_current_topic] = status_sub

        # we have new vescs, create new subs
        if len(self.vesc_status_subscribers) < len(topics_list):
            # TODO
            return

        # we have less vescs, remove/destroy old subs
        if len(self.vesc_status_subscribers) > len(topics_list):
            # TODO
            return

    def publish_battery_percentage(self):
        # get vesc status topics. If there are new ones, register subs to them and get the data
        vesc_status_topics = self.get_vesc_status_topics()
        self.get_vesc_status_subscribers(vesc_status_topics)

        print(self.vesc_status_subscribers)

        battery_state = BatteryState()
        battery_state.voltage = 40.0  # temporary battery voltage placeholder
        battery_state.percentage = self.get_battery_percentage(40.0)
        self.battery_pub.publish(battery_state)

    def get_battery_percentage(self, curr_voltage):
        last_index = len(self.key_voltages)
        for i in range(last_index):
            upper_volt = self.key_voltages[i][0]
            upper_percentage = self.key_voltages[i][1]

            if i == last_index-1:
                return float(upper_percentage)

            lower_volt = self.key_voltages[i+1][0]
            lower_percentage = self.key_voltages[i+1][1]
            if i == 0 and curr_voltage > upper_volt:
                return float(upper_percentage)
            elif upper_volt > curr_voltage > lower_volt:
                percentage = float((curr_voltage - lower_volt) / (upper_volt - lower_volt))
                scaled_percentage = float(lower_percentage + (upper_percentage - lower_percentage) * percentage)
                return scaled_percentage


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    battery_status_node = BatteryStatus()
    rclpy.spin(battery_status_node)
    battery_status_node.destroy_node()
    rclpy.shutdown()
