#!/usr/bin/env python

import array as arr
import roslib
import rospy
from fximu.msg import SensorData
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

GYRO_SENSITIVITY_250DPS = 0.0078125
GYRO_SENSITIVITY_500DPS = 0.015625
GYRO_SENSITIVITY_1000DPS = 0.03125
GYRO_SENSITIVITY_2000DPS = 0.0625

ACCEL_MG_LSB_2G = 0.000244
ACCEL_MG_LSB_4G = 0.000488
ACCEL_MG_LSB_8G = 0.000976

MAG_UT_LSB = 0.1

SENSORS_DPS_TO_RADS = 0.017453293
SENSORS_GRAVITY_EARTH = 9.80665

mag_offsets = [35.27, 63.69, 49.95]
mag_softiron_matrix = [[0.985,-0.016,-0.003],[-0.016,0.983,0.015],[-0.003,0.015,1.034]]
gyro_zero_offsets = [0.0, 0.0, 0.0]

class ImuBridge():

    def __init__(self):

        rospy.init_node('imu_bridge', anonymous=False)

        self.pub_imu = rospy.Publisher('/imu/data_raw', Imu, queue_size=128)
        self.pub_mag = rospy.Publisher('/imu/mag', MagneticField, queue_size=128)

        rospy.wait_for_message('/imu/sensor_data', SensorData)
        rospy.Subscriber('/imu/sensor_data', SensorData, self.pub_sensor_data)

        rospy.loginfo("Publishing imu data on /imu/data_raw and /imu/mag")

    def pub_sensor_data(self, msg):

	# create imu object
        imu = Imu()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = 'base_imu_link'

	# process gyro values
	x = msg.gx * GYRO_SENSITIVITY_250DPS * SENSORS_DPS_TO_RADS
        y = msg.gy * GYRO_SENSITIVITY_250DPS * SENSORS_DPS_TO_RADS
        z = msg.gz * GYRO_SENSITIVITY_250DPS * SENSORS_DPS_TO_RADS

        imu.angular_velocity.x = x + gyro_zero_offsets[0]
        imu.angular_velocity.y = y + gyro_zero_offsets[1]
        imu.angular_velocity.z = z + gyro_zero_offsets[2]

	# process accelerometer values
	imu.linear_acceleration.x = msg.ax * ACCEL_MG_LSB_4G * SENSORS_GRAVITY_EARTH
        imu.linear_acceleration.y = msg.ay * ACCEL_MG_LSB_4G * SENSORS_GRAVITY_EARTH
        imu.linear_acceleration.z = msg.az * ACCEL_MG_LSB_4G * SENSORS_GRAVITY_EARTH

	# process magnetometer values
        x = msg.mx * MAG_UT_LSB
        y = msg.my * MAG_UT_LSB
        z = msg.mz * MAG_UT_LSB

        # apply mag offset compensation (base values in uTesla)
        x = x - mag_offsets[0]
        y = y - mag_offsets[1]
        z = z - mag_offsets[2]

	# create magnetic field object
        mag = MagneticField()
        mag.header.stamp = imu.header.stamp
        mag.header.frame_id = 'base_imu_link'

        # apply mag soft iron error compensation
        mag.magnetic_field.x = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2]
        mag.magnetic_field.y = y * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2]
        mag.magnetic_field.z = z * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2]

        self.pub_imu.publish(imu)
        self.pub_mag.publish(mag);


if __name__ == '__main__':
    try:
        ImuBridge()
        rospy.spin()
    except:
        pass
