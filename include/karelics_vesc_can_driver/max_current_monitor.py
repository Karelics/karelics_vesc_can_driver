#!/usr/bin/env python3

from collections import deque
from threading import Thread
import rospy

from std_msgs.msg import Float32


class MonitorMaxCurrent:
    """ Monitors the total motor current to ensure that we are not discharging battery over
        the battery's continuous maximum discharge limit. Implementation works so that
        during the time-window of max_overcurrent_time*2 we check how many times have have
        crossed cont_current_lim. If more than half of the times, the motor is set on cooldown.
        This is required to monitor max discharge correctly also during fast spikes.
        TODO Currently only set_erpm and set_rpm functions use this in vesc.py. """

    def __init__(self, cont_current_lim, max_draw_time=2):
        """
        cont_current_lim : float
            maximum continuous discharge current of the battery in amps (A)
        max_overcurrent_time: float
            maximum time we can draw more than the maximum current (in seconds).
        """
        self.cont_current_lim = cont_current_lim
        self.max_draw_time = max_draw_time  # Time in seconds for how long the overcurrent can be applied

        self.total_current_pub = rospy.Publisher("vescs/total_current", Float32, queue_size=1)

        self.time_deque = TimeDeque(max_seconds=self.max_draw_time * 2)
        self.cooldown_time = 5  # in seconds

        self.fatal_current = False  # Set to true if on reached fatal limit and motor is on cooldown

    def _handle_safety_limit(self, total_current):
        self.time_deque.append(total_current)
        median = self.time_deque.get_median()  # Checking if the motors have drawn maximum current over 2 second total within 4 seconds time window
        if median >= self.cont_current_lim and not self.fatal_current:
            self._set_overcurrent()

    def _set_overcurrent(self):
        rospy.logerr("Motors are applying too high total current. Cutting off.")
        self.fatal_current = True
        thread = Thread(target=self._set_motor_cooldown, args=())
        thread.daemon = True
        thread.start()

    def _set_motor_cooldown(self):
        rospy.logerr("Starting {} seconds cooldown.".format(self.cooldown_time))
        rospy.sleep(self.cooldown_time)
        rospy.loginfo("Motors are now ready for operation.")
        self.fatal_current = False

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
        med = self.median(total_currents)
        msg_data = Float32()
        msg_data.data = med
        self.pub.publish(med)
        return med

    @staticmethod
    def median(lst):
        n = len(lst)
        s = sorted(lst)
        return (sum(s[n // 2 - 1:n // 2 + 1]) / 2.0, s[n // 2])[n % 2] if n else None
