
import rospy

from std_msgs.msg import Float32, Header

from karelics_vesc_can_driver.msg import VescStatus

from karelics_vesc_can_driver.vesc_messages import *


class Vesc:

    def __init__(self, vesc_id, send_function, motor_poles, gear_ratio):
        self.vesc_id = vesc_id

        self.motor_poles = int(motor_poles)
        self.gear_ratio = float(gear_ratio)

        # Status message
        self.erpm = 0
        self.duty_cycle = 0
        self.current = 0
        # Status message 2
        self.amp_hours = 0
        self.amp_hours_charged = 0
        # Status message 3
        self.watt_hours = 0
        self.watt_hours_charged = 0
        # Status message 4
        self.temp_fet = 0
        self.temp_motor = 0
        self.current_in = 0
        self.pid_pos_now = 0
        # Status message 5
        self.tacho_value = 0
        self.v_in = 0

        # Subscribe to cmd topics
        rospy.Subscriber("vesc_%i/set/current" % self.vesc_id, Float32, self.set_current_cb)
        rospy.Subscriber("vesc_%i/set/brake_current" % self.vesc_id, Float32, self.set_brake_cb)
        rospy.Subscriber("vesc_%i/set/duty_cycle" % self.vesc_id, Float32, self.set_duty_cycle_cb)
        rospy.Subscriber("vesc_%i/set/position" % self.vesc_id, Float32, self.set_position_cb)
        rospy.Subscriber("vesc_%i/set/erpm" % self.vesc_id, Float32, self.set_erpm_cb)
        rospy.Subscriber("vesc_%i/set/rpm" % self.vesc_id, Float32, self.set_rpm_cb)

        # Setup vesc status Publisher
        self.status_pub = rospy.Publisher("vesc_%i/status" % self.vesc_id, VescStatus, queue_size=1)

        self.send_cb = send_function

    def publish_status(self):

        status_msg = VescStatus()
        status_msg.header = Header()
        status_msg.header.stamp = rospy.Time.now()

        status_msg.erpm = int(self.erpm)
        status_msg.rpm = int(self.erpm / (self.motor_poles/2) / self.gear_ratio)
        status_msg.duty_cycle = self.duty_cycle
        status_msg.current = self.current
        status_msg.amp_hours = self.amp_hours
        status_msg.amp_hours_charged = self.amp_hours_charged
        status_msg.watt_hours = self.watt_hours
        status_msg.watt_hours_charged = self.watt_hours_charged
        status_msg.temp_fet = self.temp_fet
        status_msg.temp_motor = self.temp_motor
        status_msg.current_in = self.current_in
        status_msg.pid_pos_now = self.pid_pos_now
        status_msg.tacho_value = self.tacho_value
        status_msg.v_in = self.v_in
        status_msg.rotations = self.tacho_value / self.motor_poles / 3 / self.gear_ratio

        self.status_pub.publish(status_msg)

    def set_send_cb(self, send_function):
        self.send_cb = send_function

    def __str__(self):
        string = "VESC[%i]: \n" % self.vesc_id
        string += "erpm: %i duty: %f  current: %i  \n" % (self.erpm, self.duty_cycle, self.current)
        string += "amp_hours: %i amp_hours_charged: %f \n" % (self.amp_hours, self.amp_hours_charged)
        string += "watt_hours: %i watt_hours_charged: %f \n" % (self.watt_hours, self.watt_hours_charged)
        string += "temp_fet: %f temp_motor: %f current_in: %f " \
                  "pid_pos_now: %f  \n" % (self.temp_fet, self.temp_motor, self.current_in, self.pid_pos_now)
        string += "tacho_value: %f v_in: %f \n" % (self.tacho_value, self.v_in)
        return string

    def set_current_cb(self, msg: Float32):
        current_msg = VesSetCurrent(current=msg.data)
        self.send_cb(current_msg.get_can_msg(self.vesc_id))
        pass

    def set_brake_cb(self, msg: Float32):
        brake_current_msg = VesSetBrakeCurrent(current=msg.data)
        self.send_cb(brake_current_msg.get_can_msg(self.vesc_id))
        pass

    def set_duty_cycle_cb(self, msg: Float32):
        dutycyle_msg = VesSetDuty(dutycycle=msg.data)
        self.send_cb(dutycyle_msg.get_can_msg(self.vesc_id))

    def set_position_cb(self, msg: Float32):
        pos_msg = VesSetPos(pos=msg.data)
        self.send_cb(pos_msg.get_can_msg(self.vesc_id))

    def set_erpm_cb(self, msg: Float32):
        rpm_msg = VescSetRPM(rpm=msg.data)
        self.send_cb(rpm_msg.get_can_msg(self.vesc_id))
        pass

    def set_rpm_cb(self, msg: Float32):
        rpm_msg = VescSetRPM(rpm=msg.data * (self.motor_poles/2) * self.gear_ratio)
        self.send_cb(rpm_msg.get_can_msg(self.vesc_id))
        pass
