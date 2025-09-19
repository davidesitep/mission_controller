#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import tf.transformations as tft


def imu_simulator():
    rospy.init_node("imu_simulator")
    pub = rospy.Publisher("/imu/data", Imu, queue_size=10)
    rate = rospy.Rate(20)  # 20 Hz

    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - t0
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"

        # Angular velocity (rad/s) -> simula piccola rotazione
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.1 * math.sin(0.5 * t)

        # Linear acceleration (m/s^2) -> gravità + rumore sinusoidale
        imu_msg.linear_acceleration.x = 0.1 * math.sin(0.1 * t)
        imu_msg.linear_acceleration.y = 0.1 * math.cos(0.1 * t)
        imu_msg.linear_acceleration.z = 9.81

        # Orientamento (quaternion) -> simula piccola oscillazione in roll
        quat = tft.quaternion_from_euler(0.1 * math.sin(0.2 * t), 0.0, 0.0)
        imu_msg.orientation = Quaternion(*quat)

        pub.publish(imu_msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        imu_simulator()
    except rospy.ROSInterruptException:
        pass
