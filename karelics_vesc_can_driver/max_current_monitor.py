from collections import deque

from rclpy.node import Node

from karelics_vesc_can_driver.utils import get_current_time_seconds
from std_msgs.msg import Float32


class MonitorMaxCurrent:
    """
    Monitors the total motor current to ensure that we are not discharging battery over
    the battery's continuous maximum discharge limit. Implementation works so that
    during the time-window of max_draw_time*2 we check how many times have have
    crossed cont_current_lim. If more than half of the times, the motor is set on cool-down.
    This is required to monitor max discharge correctly also during fast spikes.
    TODO: Currently only set_erpm and set_rpm functions use this in vesc.py.
    """

    def __init__(self, node: Node, cont_current_lim, max_draw_time=2.0):
        """
        node: the parent node that is instantiating the MonitorMaxCurrent class
        cont_current_lim : float
            maximum continuous discharge current of the battery in amps (A)
        max_draw_time: float
            maximum time (in seconds) we can draw more than the maximum current.
        """
        self.parent_node = node
        self.cont_current_lim = cont_current_lim
        self.max_draw_time = max_draw_time  # Time in seconds for how long the over-current can be applied

        self.total_current_pub = self.parent_node.create_publisher(Float32, "/vescs/total_current", qos_profile=1)

        self.time_deque = TimeDeque(self.parent_node, max_seconds=self.max_draw_time * 2)
        self.cooldown_time = 5  # in seconds

        self.motor_cooldown_timer = None

        self.fatal_current = False  # Set to true if we reached fatal limit and motor is on cool-down

    def _handle_safety_limit(self, total_current):
        self.time_deque.append(total_current)
        median = self.time_deque.get_median()  # Checking if the motors have drawn maximum current over 2 second total
                                               # within 4 seconds time window
        if median >= self.cont_current_lim and not self.fatal_current:
            self._set_overcurrent()

    def _set_overcurrent(self):
        self.parent_node.get_logger().error("Motors are applying too high total current. Cutting off.")
        self.fatal_current = True

        # TODO: this was done in a separate thread before, new approach needs testing
        self.parent_node.get_logger().error(f"Starting {self.cooldown_time} seconds cool-down")

        self._set_motor_cooldown()
        # thread = Thread(target=self._set_motor_cooldown, args=())
        # thread.daemon = True
        # thread.start()

    def _set_motor_cooldown(self):
        self.motor_cooldown_timer = self.parent_node.create_timer(self.cooldown_time, self.cb_motor_cd_timer)

    def cb_motor_cd_timer(self):
        self.parent_node.get_logger().info("Motors are now ready for operation.")
        self.fatal_current = False
        if self.motor_cooldown_timer is not None:
            self.parent_node.destroy_timer(self.motor_cooldown_timer)
            self.motor_cooldown_timer = None

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


class TimeDeque:
    """ Implements a deque, but instead of giving maxlen to it, you can give it max_seconds.
        Only the data from the past max_seconds is stored. """

    def __init__(self, node: Node, max_seconds=10.0):
        self.time_deque = deque()
        self.max_seconds = max_seconds
        self.parent_node = node
        self.pub = self.parent_node.create_publisher(Float32, "/status/vescs/median_current", qos_profile=1)

    def _remove_old_data(self):
        now = get_current_time_seconds(self.parent_node)
        while self.time_deque:
            t = self.time_deque[0][0]
            if now - t > self.max_seconds:
                self.time_deque.popleft()
            else:
                break

    def append(self, value):
        self._remove_old_data()
        self.time_deque.append([get_current_time_seconds(self.parent_node), value])

    def get_median(self):
        self._remove_old_data()
        total_currents = []
        for _, v in self.time_deque:
            total_currents.append(v)
        med = float(self.median(total_currents))
        msg_data = Float32()
        msg_data.data = med
        self.pub.publish(msg_data)
        return med

    @staticmethod
    def median(lst):
        n = len(lst)
        s = sorted(lst)
        return (sum(s[n // 2 - 1:n // 2 + 1]) / 2.0, s[n // 2])[n % 2] if n else None
