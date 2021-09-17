#!/usr/bin/env python3

from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class BatteryStatus:
    def __init__(self, node: Node):
        """
        node: the parent node that is instantiating the BatteryStatus class
        """

        self.parent_node = node

        self.battery_pub = self.parent_node.create_publisher(BatteryState, "/battery", qos_profile=1)

        # voltage, percentage. All the other voltage values between these key-values
        # are linearly calculated
        self.key_voltages = [[54.10, 1.0],
                             [48, 0.6],
                             [45.8, 0.2],
                             [44.2, 0.05],
                             [41.8, 0.02],
                             [40, 0]]

    def publish(self, voltage):
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
                return upper_percentage

            lower_volt = self.key_voltages[i+1][0]
            lower_percentage = self.key_voltages[i+1][1]
            if i == 0 and curr_voltage > upper_volt:
                return upper_percentage
            elif upper_volt > curr_voltage > lower_volt:
                percentage = (curr_voltage - lower_volt) / (upper_volt - lower_volt)
                scaled_percentage = lower_percentage + (upper_percentage - lower_percentage) * percentage
                return scaled_percentage
