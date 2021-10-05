#!/usr/bin/env python3
from functools import partial
import re
import sys
from typing import List

import rclpy
from rclpy.node import Node, NodeNameNonExistentError
from sensor_msgs.msg import BatteryState

from karelics_vesc_can_driver.msg import VescStatus


class BatteryStatus(Node):
    """
    BatteryStatus node handling the publishing of the battery status according to the statuses of all the active vescs
    in the system. The node dynamically subscribes to the status topics of each of the active vescs, gets the battery
    voltages from them, computes the mean battery voltage and then publishes the overall battery status for the robot
    under the /battery topic
    """

    # voltage, percentage.
    # All the other voltage values between these key-values
    # are linearly calculated
    key_voltages = ((54.10, 1.0),
                    (48, 0.6),
                    (45.8, 0.2),
                    (44.2, 0.05),
                    (41.8, 0.02),
                    (40, 0))

    def __init__(self):
        super().__init__("battery_status")

        self.battery_pub = self.create_publisher(BatteryState, "/battery", qos_profile=1)
        self.battery_pub_timer = self.create_timer(1.0, self.publish_battery_percentage)

        self.vesc_status_subscribers = dict()
        self.vesc_voltages = dict()

        self.current_battery_voltage = None

    def vesc_status_cb(self, data: VescStatus, vesc: str):
        """
        The Callback function for the vesc status subscribers

        :param VescStatus data: the data incoming from the vesc status topic
        :param str vesc: the name of the vesc status we are subscribing to
        """

        self.vesc_voltages[vesc] = float(data.v_in)

    def get_mean_battery_voltage(self) -> float:
        """
        Computes the mean battery voltage from the values read from all the active vescs

        :return float mean_battery_voltage: the mean of the battery voltages read from the status topics of active vescs
        """

        mean_battery_voltage = 0.0

        if len(self.vesc_voltages) != 0:
            mean_battery_voltage = float(sum(list(self.vesc_voltages.values())) / len(self.vesc_voltages))

        return mean_battery_voltage

    def get_vesc_status_topics(self) -> List[str]:
        """
        Returns a list containing the status topics of all active vescs

        :return List[str] vesc_status_topics: all the detected active vesc status topics in the system
        """

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
    def vesc_from_topic(topic: str) -> str:
        """
        Returns the vesc name from the status topic published by the vesc

        :param str topic: the vesc status topic
        :return: str: the name of the vesc decoded from the vesc status topic
        """

        return list(filter(None, topic.split(sep='/')))[0]

    def get_vescs_from_status_topics(self, topics_list: List[str]) -> List[str]:
        """
        Returns a list of vesc names from status topics

        :param List[str] topics_list: list of status topics
        :return List[str] active_vescs: list with the names of active vescs decoded from the status topics
        """

        active_vescs = []

        for topic in topics_list:
            active_vescs.append(self.vesc_from_topic(topic))

        return active_vescs

    def create_new_status_sub(self, topic: str, vesc: str):
        """
        Creates a new subscription to the topic published by the specified vesc

        :param str topic: status topic for which we want to create a new subscription
        :param str vesc: name of the vesc publishing the status topic
        """

        status_sub = self.create_subscription(VescStatus, topic, partial(self.vesc_status_cb,
                                                                         vesc=vesc),
                                              qos_profile=1)
        self.vesc_status_subscribers[vesc] = status_sub

    def create_new_vesc_status_subs(self, topics_list: List[str]):
        """
        Creates new vesc status topic subs for all the newly detected active topics

        :param List[str] topics_list: list of active vesc status topics
        """

        for topic in topics_list:
            vesc_from_topic = self.vesc_from_topic(topic)
            if vesc_from_topic not in self.vesc_status_subscribers.keys():
                self.create_new_status_sub(topic, vesc_from_topic)

    def destroy_stale_vesc_status_subs(self, active_vescs: List[str]):
        """
        Goes through the subs we have already registered before to status topics and checks it against the list of
        active vescs. If we detect that we have a sub to the status of a vesc that is no longer active in the system,
        that subscription gets destroyed and removed from our dictionary of active subscriptions

        :param List[str] active_vescs: list of names of the currently active vescs in the system
        """

        vescs_to_destroy = []
        for vesc in list(self.vesc_status_subscribers.keys()):
            if vesc not in active_vescs:
                vescs_to_destroy.append(vesc)
        for vesc in vescs_to_destroy:
            self.destroy_subscription(self.vesc_status_subscribers[vesc])
            del self.vesc_status_subscribers[vesc]
            if self.vesc_voltages.get(vesc):
                del self.vesc_voltages[vesc]

    def publish_battery_percentage(self):
        """
        Call back function for the timer responsible of publishing the battery status. On each timer callback (1.0
        seconds at the moment) we get the status topics of all the vescs active in the system, we then create new
        subscriptions for the newly discovered topics (published by vescs that were not active before), destroy and
        delete stale subscriptions and then we get the mean battery voltage from the voltages reported by all the
        active vescs and then publish it to the /battery topic. If we have no active vescs this function returns and
        no data is then published on to the /battery topic.

        :return None: only in the scenario when we have no active vescs detected in the system.
        """
        active_vesc_status_topics = self.get_vesc_status_topics()
        active_vescs = self.get_vescs_from_status_topics(active_vesc_status_topics)

        self.create_new_vesc_status_subs(active_vesc_status_topics)

        self.destroy_stale_vesc_status_subs(active_vescs)

        if len(active_vescs) == 0:
            return

        mean_battery_voltage = self.get_mean_battery_voltage()

        battery_state = BatteryState()
        battery_state.voltage = mean_battery_voltage
        battery_state.percentage = self.get_battery_percentage(mean_battery_voltage)
        self.battery_pub.publish(battery_state)

    @staticmethod
    def get_battery_percentage(curr_voltage: float):
        """
        Computes the battery percentage according to the voltage and the key static voltage limits
        specified in BatteryStatus.key_voltages

        :param float curr_voltage: voltage value
        :return float percentage: percentage from voltage
        """

        last_index = len(BatteryStatus.key_voltages)
        for i in range(last_index):
            upper_volt = BatteryStatus.key_voltages[i][0]
            upper_percentage = BatteryStatus.key_voltages[i][1]

            if i == last_index - 1:
                return float(upper_percentage)

            lower_volt = BatteryStatus.key_voltages[i + 1][0]
            lower_percentage = BatteryStatus.key_voltages[i + 1][1]
            if i == 0 and curr_voltage > upper_volt:
                return float(upper_percentage)
            elif upper_volt > curr_voltage > lower_volt:
                percentage = (curr_voltage - lower_volt) / (upper_volt - lower_volt)
                scaled_percentage = lower_percentage + (upper_percentage - lower_percentage) * percentage
                return float(scaled_percentage)


if __name__ == '__main__':
    rclpy.init(args=sys.argv)

    battery_status_node = BatteryStatus()

    rclpy.spin(battery_status_node)

    battery_status_node.destroy_node()
    rclpy.shutdown()
