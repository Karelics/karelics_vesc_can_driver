#    This file contains the unit tests for the BatteryStatus ROS node.
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

import math
import unittest
from karelics_vesc_can_driver.battery_status_base import BatteryStatusBase


class TestBatteryStatus(unittest.TestCase):
    def test_battery_percentage(self):
        """Test that percentage is computed correctly:
        - inside each key voltage range
        - on edges of each key voltage range
        Note: this test is based on the knowledge of the implementation,
        so if implementation is changed the test would be broken.
        """
        self.assertEqual(BatteryStatusBase.get_battery_percentage(55), 1.0)
        for i in range(1, len(BatteryStatusBase.key_voltages)):
            top_range_voltage, top_range_percentage = BatteryStatusBase.key_voltages[i - 1]
            bottom_range_voltage, bottom_range_percentage = BatteryStatusBase.key_voltages[i]

            middle_voltage = (top_range_voltage + bottom_range_voltage) / 2.0
            middle_percentage = (top_range_percentage + bottom_range_percentage) / 2.0

            self.assertAlmostEqual(BatteryStatusBase.get_battery_percentage(top_range_voltage), top_range_percentage, 1)
            self.assertAlmostEqual(BatteryStatusBase.get_battery_percentage(middle_voltage), middle_percentage, 1)
            self.assertAlmostEqual(
                BatteryStatusBase.get_battery_percentage(bottom_range_voltage), bottom_range_percentage, 1
            )

        self.assertEqual(BatteryStatusBase.get_battery_percentage(39), 0.0)

    def test_battery_voltage(self):
        """Tests, that battery voltage is correct, even if some vesc is not exist, or provided incorrect value"""

        # Check correct voltages
        vesc_voltages = {"1": 48.0, "2": 47.5, "3": 49.0, "4": 48.5}
        comp_voltage = BatteryStatusBase.get_mean_battery_voltage(vesc_voltages)
        self.assertTrue(47.5 <= comp_voltage <= 49.0, f"with {comp_voltage=} and {vesc_voltages=}")

        vesc_voltages = {"1": 48.0, "2": 47.5, "3": 49.0}
        comp_voltage = BatteryStatusBase.get_mean_battery_voltage(vesc_voltages)
        self.assertTrue(47.5 <= comp_voltage <= 49.0, f"with {comp_voltage=} and {vesc_voltages=}")

        vesc_voltages = {"1": 48.0, "2": 47.5}
        comp_voltage = BatteryStatusBase.get_mean_battery_voltage(vesc_voltages)
        self.assertTrue(47.5 <= comp_voltage <= 48.0, f"with {comp_voltage=} and {vesc_voltages=}")

        vesc_voltages = {"1": 48.0}
        comp_voltage = BatteryStatusBase.get_mean_battery_voltage(vesc_voltages)
        self.assertEqual(comp_voltage, 48.0, f"with {comp_voltage=} and {vesc_voltages=}")

        # Check incorrect (low) voltages
        vesc_voltages = {"1": 8.0, "2": 47.5, "3": 49.0, "4": 48.5}
        comp_voltage = BatteryStatusBase.get_mean_battery_voltage(vesc_voltages)
        self.assertTrue(47.5 <= comp_voltage <= 49.0, f"with {comp_voltage=} and {vesc_voltages=}")

        vesc_voltages = {"1": 98.0, "2": 47.5, "3": 49.0}
        comp_voltage = BatteryStatusBase.get_mean_battery_voltage(vesc_voltages)
        self.assertTrue(47.5 <= comp_voltage <= 49.0, f"with {comp_voltage=} and {vesc_voltages=}")

        # Check incorrect (high) voltages
        vesc_voltages = {"1": 98.0, "2": 47.5, "3": 49.0, "4": 48.5}
        comp_voltage = BatteryStatusBase.get_mean_battery_voltage(vesc_voltages)
        self.assertTrue(47.5 <= comp_voltage <= 49.0, f"with {comp_voltage=} and {vesc_voltages=}")

        vesc_voltages = {"1": 98.0, "2": 47.5, "3": 49.0}
        comp_voltage = BatteryStatusBase.get_mean_battery_voltage(vesc_voltages)
        self.assertTrue(47.5 <= comp_voltage <= 49.0, f"with {comp_voltage=} and {vesc_voltages=}")

        # Check the case when dictionary of voltages is empty (not data has been received yet)
        vesc_voltages = {}
        comp_voltage = BatteryStatusBase.get_mean_battery_voltage(vesc_voltages)
        self.assertTrue(math.isnan(comp_voltage))


if __name__ == "__main__":
    unittest.main()
