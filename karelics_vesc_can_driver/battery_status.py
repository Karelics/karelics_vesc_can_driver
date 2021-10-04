#!/usr/bin/env python3
from functools import partial
import re
import sys

import rclpy
from rclpy.node import Node, NodeNameNonExistentError
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
        self.vesc_voltages[vesc] = float(data.v_in)

    def get_mean_battery_voltage(self):
        if len(self.vesc_voltages) != 0:
            return float(sum(list(self.vesc_voltages.values())) / len(self.vesc_voltages))
        else:
            return 0.0

    def get_vesc_status_topics(self):
        vesc_status_topics = []

        try:
            topics_and_types = self.get_publisher_names_and_types_by_node('karelics_vesc_can_driver',
                                                                          '/', no_demangle=False)
        except (NodeNameNonExistentError, RuntimeError):
            return vesc_status_topics

        string_topic_format = re.compile(r'/vesc_(\d+)/status')  # RegEx template to find all vesc status topics

        for topic_tuple in topics_and_types:
            if string_topic_format.match(topic_tuple[0]):
                vesc_status_topics.append(topic_tuple[0])

        return vesc_status_topics

    @staticmethod
    def vesc_from_topic(topic):
        return list(filter(None, topic.split(sep='/')))[0]

    def get_vescs_from_status_topics(self, topics_list):
        active_vescs = []

        for topic in topics_list:
            active_vescs.append(self.vesc_from_topic(topic))

        return active_vescs

    def create_new_status_sub(self, topic, vesc):
        status_sub = self.create_subscription(VescStatus, topic, partial(self.vesc_status_cb,
                                                                         vesc=vesc),
                                              qos_profile=1)
        self.vesc_status_subscribers[vesc] = status_sub

    def create_new_vesc_status_subs(self, topics_list):
        for topic in topics_list:
            vesc_from_topic = self.vesc_from_topic(topic)
            if vesc_from_topic not in self.vesc_status_subscribers.keys():
                self.create_new_status_sub(topic, vesc_from_topic)

    def destroy_stale_vesc_status_subs(self, active_vescs):
        vescs_to_destroy = []
        for vesc in list(self.vesc_status_subscribers.keys()):
            if vesc not in active_vescs:
                vescs_to_destroy.append(vesc)

        print("vescs for destroying: ", vescs_to_destroy)

        for vesc in vescs_to_destroy:
            self.destroy_subscription(self.vesc_status_subscribers[vesc])
            del self.vesc_status_subscribers[vesc]
            if self.vesc_voltages.get(vesc):
                del self.vesc_voltages[vesc]

    def publish_battery_percentage(self):
        active_vesc_status_topics = self.get_vesc_status_topics()
        active_vescs = self.get_vescs_from_status_topics(active_vesc_status_topics)

        self.create_new_vesc_status_subs(active_vesc_status_topics)

        print("existing subs after adding new ones: ", list(self.vesc_status_subscribers.keys()))
        print("found vescs: ", active_vescs)

        self.destroy_stale_vesc_status_subs(active_vescs)

        print("subs after destroying: ", list(self.vesc_status_subscribers.keys()))
        print()

        if len(active_vescs) == 0:
            return

        mean_battery_voltage = self.get_mean_battery_voltage()

        battery_state = BatteryState()
        battery_state.voltage = mean_battery_voltage
        battery_state.percentage = self.get_battery_percentage(mean_battery_voltage)
        self.battery_pub.publish(battery_state)

    def get_battery_percentage(self, curr_voltage):
        last_index = len(self.key_voltages)
        for i in range(last_index):
            upper_volt = self.key_voltages[i][0]
            upper_percentage = self.key_voltages[i][1]

            if i == last_index - 1:
                return float(upper_percentage)

            lower_volt = self.key_voltages[i + 1][0]
            lower_percentage = self.key_voltages[i + 1][1]
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
