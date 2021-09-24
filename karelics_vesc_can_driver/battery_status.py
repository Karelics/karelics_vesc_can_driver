#!/usr/bin/env python3

from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class BatteryStatus(Node):
    def __init__(self, vesc_can_driver_node: Node):

        self.battery_pub = self.create_publisher(BatteryState, "/battery", qos_profile=1)

        # self.battery_pub_timer = self.create_timer(1.0, self.publish_battery_percentage)

        self.vesc_can_driver_node = vesc_can_driver_node

        self.vesc_status_subs = []
        self.vesc_voltages = []

        self.current_battery_voltage = None

        # voltage, percentage. All the other voltage values between these key-values
        # are linearly calculated
        self.key_voltages = [[54.10, 1.0],
                             [48, 0.6],
                             [45.8, 0.2],
                             [44.2, 0.05],
                             [41.8, 0.02],
                             [40, 0]]

    def get_vesc_status_subs(self):
        topics = self.vesc_can_driver_node.get_topic_names_and_types()
        print()
        print(topics)
        print()

    def publish_battery_percentage(self, voltage):
        # get vesc status topics. If there are new ones, register subs to them and get the data
        self.get_vesc_status_subs()

        battery_state = BatteryState()
        battery_state.voltage = voltage
        battery_state.percentage = self.get_battery_percentage(voltage)
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
