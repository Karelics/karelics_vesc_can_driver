from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Header
from std_srvs.srv import SetBool

from rclpy.node import Node

from karelics_vesc_can_driver.msg import VescStatus
from karelics_vesc_can_driver.vesc_messages import *


class Vesc:

    def __init__(self, node: Node, vesc_id, send_function, lock_function, release_function, motor_poles, gear_ratio,
                 current_monitor):

        self.node = node

        self._get_imu_data = False
        self._request_send = False
        self.vesc_id = vesc_id

        self.motor_poles = int(motor_poles)
        self.gear_ratio = float(gear_ratio)
        self.current_monitor = current_monitor

        # Status message
        self.erpm = 0
        self.duty_cycle = 0.0
        self.current = 0.0
        # Status message 2
        self.amp_hours = 0.0
        self.amp_hours_charged = 0.0
        # Status message 3
        self.watt_hours = 0.0
        self.watt_hours_charged = 0.0
        # Status message 4
        self.temp_fet = 0.0
        self.temp_motor = 0.0
        self.current_in = 0.0
        self.pid_pos_now = 0.0
        # Status message 5
        self.tacho_value = 0.0
        self.v_in = 0.0
        # Imu message
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.accX = 0.0
        self.accY = 0.0
        self.accZ = 0.0
        self.gyroX = 0.0
        self.gyroY = 0.0
        self.gyroZ = 0.0
        self.magX = 0.0
        self.magY = 0.0
        self.magZ = 0.0
        self.q0 = 0.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

        # Subscribe to cmd topics
        self.node.create_subscription(Float32, f'vesc_{self.vesc_id}/set/current', self.set_current_cb, qos_profile=1)
        self.node.create_subscription(Float32, f'vesc_{self.vesc_id}/set/brake_current', self.set_brake_cb,
                                      qos_profile=1)
        self.node.create_subscription(Float32, f'vesc_{self.vesc_id}/set/handbrake_current',
                                      self.set_handbrake_current_cb,
                                      qos_profile=1)
        self.node.create_subscription(Float32, f'vesc_{self.vesc_id}/set/duty_cycle', self.set_duty_cycle_cb,
                                      qos_profile=1)
        self.node.create_subscription(Float32, f'vesc_{self.vesc_id}/set/position', self.set_position_cb, qos_profile=1)
        self.node.create_subscription(Float32, f'vesc_{self.vesc_id}/set/erpm', self.set_erpm_cb, qos_profile=1)
        self.node.create_subscription(Float32, f'vesc_{self.vesc_id}/set/rpm', self.set_rpm_cb, qos_profile=1)

        # Setup vesc status Publisher
        self.status_pub = self.node.create_publisher(VescStatus, f'vesc_{self.vesc_id}/status', qos_profile=1)
        self.imu_pub = self.node.create_publisher(Imu, f'vesc_{self.vesc_id}/imu_data', qos_profile=1)

        # Enable IMU
        self.node.create_service(SetBool, f'vesc_{self.vesc_id}/enable_imu', self.enable_imu_cb)

        self.send_cb = send_function
        self.lock = lock_function
        self.release = release_function

    def enable_imu_cb(self, request, _):
        if self._get_imu_data == request.data:
            return SetBool.Response(success=self._get_imu_data, message="State unchanged")

        if not request.data:
            self._get_imu_data = False
            return SetBool.Response(success=self._get_imu_data, message="Stopped")

        if self.lock(self.vesc_id):
            self._get_imu_data = request.data
            return SetBool.Response(success=True, message="Enabled IMU")
        else:
            return SetBool.Response(success=False, message="Could not acquire lock other vesc IMU active?")

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
            imu_msg.header.stamp = self.node.get_clock().now().to_msg()

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
        status_msg.header.stamp = self.node.get_clock().now().to_msg()

        status_msg.erpm = int(self.erpm)
        status_msg.rpm = int(self.erpm / (self.motor_poles / 2) / self.gear_ratio)
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
        status_msg.tacho_value = float(self.tacho_value)
        status_msg.v_in = float(self.v_in)
        status_msg.rotations = float(self.tacho_value / self.motor_poles / 3 / self.gear_ratio)

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
        try:
            can_frame = current_msg.get_can_msg(self.vesc_id)
            self.send_cb(can_frame)
        except AssertionError as e:
            self.node.get_logger().error(str(e))

    def set_brake_cb(self, msg: Float32):
        brake_current_msg = VescSetBrakeCurrent(current=msg.data)
        try:
            can_frame = brake_current_msg.get_can_msg(self.vesc_id)
            self.send_cb(can_frame)
        except AssertionError as e:
            self.node.get_logger().error(str(e))

    def set_handbrake_current_cb(self, msg: Float32):
        handbrake_current_msg = VescSetHandbrakeCurrent(current=msg.data)
        try:
            can_frame = handbrake_current_msg.get_can_msg(self.vesc_id)
            self.send_cb(can_frame)
        except AssertionError as e:
            self.node.get_logger().error(str(e))

    def set_duty_cycle_cb(self, msg: Float32):
        dutycyle_msg = VescSetDuty(dutycycle=msg.data)
        try:
            can_frame = dutycyle_msg.get_can_msg(self.vesc_id)
            self.send_cb(can_frame)
        except AssertionError as e:
            self.node.get_logger().error(str(e))

    def set_position_cb(self, msg: Float32):
        pos_msg = VescSetPos(pos=msg.data)
        try:
            can_frame = pos_msg.get_can_msg(self.vesc_id)
            self.send_cb(can_frame)
        except AssertionError as e:
            self.node.get_logger().error(str(e))

    def set_erpm_cb(self, msg: Float32):
        if not self.current_monitor.is_safe():
            msg.data = 0.0
        rpm_msg = VescSetRPM(rpm=msg.data)
        try:
            can_frame = rpm_msg.get_can_msg(self.vesc_id)
            self.send_cb(can_frame)
        except AssertionError as e:
            self.node.get_logger().error(str(e))

    def set_rpm_cb(self, msg: Float32):
        if not self.current_monitor.is_safe():
            msg.data = 0.0
        rpm_msg = VescSetRPM(rpm=float(msg.data * (self.motor_poles / 2) * self.gear_ratio))
        try:
            can_frame = rpm_msg.get_can_msg(self.vesc_id)
            self.send_cb(can_frame)
        except AssertionError as e:
            self.node.get_logger().error(str(e))
