#    This file contains the base class for battery_status node, which provides
#    basic functions to handle battery status, which are not depend on the
#    implementation. This done in order to make unit tests independent of ROS,
#    messages, etc.
#
#    Copyright (C) 2022  Karelics Oy
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.

import typing


class BatteryStatusBase:
    # voltage, percentage.
    # All the other voltage values between these key-values
    # are linearly calculated
    key_voltages = ((54.10, 1.0), (48, 0.6), (45.8, 0.2), (44.2, 0.05), (41.8, 0.02), (40, 0))

    @staticmethod
    def get_battery_percentage(curr_voltage: float):
        """
        Computes the battery percentage according to the voltage and the key static voltage limits
        specified in BatteryStatus.key_voltages

        :param float curr_voltage: voltage value
        :return float percentage: percentage from voltage
        """

        last_index = len(BatteryStatusBase.key_voltages)
        for i in range(last_index):
            upper_volt = BatteryStatusBase.key_voltages[i][0]
            upper_percentage = BatteryStatusBase.key_voltages[i][1]

            if i == last_index - 1:
                return float(upper_percentage)

            lower_volt = BatteryStatusBase.key_voltages[i + 1][0]
            lower_percentage = BatteryStatusBase.key_voltages[i + 1][1]
            if i == 0 and curr_voltage >= upper_volt:
                return float(upper_percentage)
            elif upper_volt >= curr_voltage >= lower_volt:
                percentage = (curr_voltage - lower_volt) / (upper_volt - lower_volt)
                scaled_percentage = lower_percentage + (upper_percentage - lower_percentage) * percentage
                return float(scaled_percentage)

    @staticmethod
    def get_mean_battery_voltage(vesc_voltages: typing.Dict[str, float]) -> float:
        """
        Computes the median battery voltage from the values read from all the active VESCs

        :return float mean_battery_voltage: the median of the battery voltages read from the status topics of active
        VESCs OR NaN if voltages haven't been received yet (vesc_voltages dictionary is empty).
        """

        median_battery_voltage = float("nan")

        if (v_len := len(vesc_voltages)) != 0:
            v_list = list(vesc_voltages.values())
            v_list.sort()
            i0 = v_len // 2
            i1 = (v_len - 1) // 2

            median_battery_voltage = (v_list[i0] + v_list[i1]) / 2.0

        return median_battery_voltage
