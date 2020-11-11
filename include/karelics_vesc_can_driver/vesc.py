
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Header
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

from karelics_vesc_can_driver.msg import VescStatus
from karelics_vesc_can_driver.vesc_messages import *


class Vesc:

    def __init__(self, vesc_id, send_function, lock_function, release_function):
        self._get_imu_data = False
        self._request_send = False

        self.vesc_id = vesc_id
        self.motor_poles = 8

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
        # Imu message
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.accX = 0
        self.accY = 0
        self.accZ = 0
        self.gyroX = 0
        self.gyroY = 0
        self.gyroZ = 0
        self.magX = 0
        self.magY = 0
        self.magZ = 0
        self.q0 = 0
        self.q1 = 0
        self.q2 = 0
        self.q3 = 0

        # Subscribe to cmd topics
        rospy.Subscriber("vesc_%i/set/current" % self.vesc_id, Float32, self.set_current_cb)
        rospy.Subscriber("vesc_%i/set/brake_current" % self.vesc_id, Float32, self.set_brake_cb)
        rospy.Subscriber("vesc_%i/set/duty_cycle" % self.vesc_id, Float32, self.set_duty_cycle_cb)
        rospy.Subscriber("vesc_%i/set/position" % self.vesc_id, Float32, self.set_position_cb)
        rospy.Subscriber("vesc_%i/set/erpm" % self.vesc_id, Float32, self.set_erpm_cb)
        rospy.Subscriber("vesc_%i/set/rpm" % self.vesc_id, Float32, self.set_rpm_cb)

        # Setup vesc status Publisher
        self.status_pub = rospy.Publisher("vesc_%i/status" % self.vesc_id, VescStatus, queue_size=1)
        self.imu_pub = rospy.Publisher("vesc_%i/imu_data" % self.vesc_id, Imu, queue_size=1)

        # Enable IMU
        rospy.Service("vesc_%i/enable_imu" % self.vesc_id, SetBool, self.enable_imu_cb)

        self.send_cb = send_function
        self.lock = lock_function
        self.release = release_function

    def enable_imu_cb(self, msg: SetBoolRequest):

        if self._get_imu_data == msg.data:
            return SetBoolResponse(self._get_imu_data, "State unchanged")

        if not msg.data:
            self._get_imu_data = False
            return SetBoolResponse(self._get_imu_data, "Stopped")

        if self.lock(self.vesc_id):
            self._get_imu_data = msg.data
            return SetBoolResponse(True, "Enabled IMU")
        else:
            return SetBoolResponse(False, "Could not acquire lock other vesc IMU active?")

    def data_recieved(self):
        self._request_send = False

    def request_imu_data(self):

        if not self._request_send:
            self._request_send = True
            msg = VescGetImuData()
            self.send_cb(msg.get_can_msg(self.vesc_id))

    def publish_imu_data(self):

        if not self._request_send:
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.frame_id = "vesc_%i_frame" % self.vesc_id
            imu_msg.header.stamp = rospy.Time.now()

            imu_msg.orientation.x = self.q0
            imu_msg.orientation.y = self.q1
            imu_msg.orientation.z = self.q2
            imu_msg.orientation.w = self.q3

            imu_msg.linear_acceleration.x = self.accX
            imu_msg.linear_acceleration.y = self.accY
            imu_msg.linear_acceleration.z = self.accZ

            imu_msg.angular_velocity.x = self.gyroX
            imu_msg.angular_velocity.y = self.gyroY
            imu_msg.angular_velocity.z = self.gyroZ

            self.imu_pub.publish(imu_msg)

    def tick(self):
        status_msg = VescStatus()
        status_msg.header = Header()
        status_msg.header.stamp = rospy.Time.now()

        status_msg.erpm = int(self.erpm)
        status_msg.rpm = int(self.erpm / self.motor_poles)
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

        self.status_pub.publish(status_msg)

        if self._get_imu_data:
            self.publish_imu_data()
            self.request_imu_data()

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
        string += "== IMU VALUES ==\n"
        string += "vesc.roll %f \n" % self.roll
        string += "vesc.pitch %f \n" % self.pitch
        string += "vesc.yaw %f \n" % self.yaw
        string += "vesc.accX %f \n" % self.accX
        string += "vesc.accY %f \n" % self.accY
        string += "vesc.accZ %f \n" % self.accZ
        string += "vesc.gyroX %f \n" % self.gyroX
        string += "vesc.gyroY %f \n" % self.gyroY
        string += "vesc.gyroZ %f \n" % self.gyroZ
        string += "vesc.magX %f \n" % self.magX
        string += "vesc.magY %f \n" % self.magY
        string += "vesc.magZ %f \n" % self.magZ
        string += "vesc.q0 %f \n" % self.q0
        string += "vesc.q1 %f \n" % self.q1
        string += "vesc.q2 %f \n" % self.q2
        string += "vesc.q3 %f \n" % self.q3

        return string

    def set_current_cb(self, msg: Float32):
        current_msg = VescSetCurrent(current=msg.data)
        self.send_cb(current_msg.get_can_msg(self.vesc_id))
        pass

    def set_brake_cb(self, msg: Float32):
        brake_current_msg = VescSetBrakeCurrent(current=msg.data)
        self.send_cb(brake_current_msg.get_can_msg(self.vesc_id))
        pass

    def set_duty_cycle_cb(self, msg: Float32):
        dutycyle_msg = VescSetDuty(dutycycle=msg.data)
        self.send_cb(dutycyle_msg.get_can_msg(self.vesc_id))

    def set_position_cb(self, msg: Float32):
        pos_msg = VescSetPos(pos=msg.data)
        self.send_cb(pos_msg.get_can_msg(self.vesc_id))

    def set_erpm_cb(self, msg: Float32):
        rpm_msg = VescSetRPM(rpm=msg.data)
        self.send_cb(rpm_msg.get_can_msg(self.vesc_id))
        pass

    def set_rpm_cb(self, msg: Float32):
        rpm_msg = VescSetRPM(rpm=msg.data * self.motor_poles)
        self.send_cb(rpm_msg.get_can_msg(self.vesc_id))
        pass
