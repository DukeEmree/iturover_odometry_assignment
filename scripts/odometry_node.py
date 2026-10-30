#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import Imu


class OdometryNode:
    def __init__(self):

        rospy.init_node("odometry_node")

        self.wheel_radius = 0.135  # metre (135 mm)
        self.track_width = 0.890   # metre (890 mm - iki tekerlek arası mesafe)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # radyan cinsinden
        self.yaw_start = None

        self.left_rpm = 0.0
        self.right_rpm = 0.0
        self.last_time = rospy.Time.now()
        self.last_msg_time = rospy.Time.now()

        self.odom_pub_ = rospy.Publisher("/odom", Odometry, queue_size=10)

        self.left_sub_ = rospy.Subscriber(
            "/drive_system_left_motors_feedbacks",
            Float64MultiArray,
            self.callback_left_motors
        )
        
        self.right_sub_ = rospy.Subscriber(
            "/drive_system_right_motors_feedbacks",
            Float64MultiArray,
            self.callback_right_motors
        )
        self.imu_sub_ = rospy.Subscriber("/imu1/data", Imu, self.callback_imu)


        self.timer_ = rospy.Timer(rospy.Duration.from_sec(0.05), self.timer_callback)

        rospy.loginfo("Odometry Node basariyla baslatildi.")


    # tekrar bak anlamadım
    def callback_imu(self, msg: Imu):
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        norm = math.sqrt(sum(k * k for k in q))
        if norm > 0:
            q_norm = [k / norm for k in q]
            roll, pitch, yaw = euler_from_quaternion(q_norm)
            if self.yaw_start is None:
                self.yaw_start = yaw
            # Robotun o anki dönüş açısını doğrudan IMU'dan alıyoruz:
            self.theta = yaw - self.yaw_start



    def callback_left_motors(self, msg: Float64MultiArray):
        self.left_rpm = (msg.data[0] + msg.data[1]) / 2.0
        self.last_msg_time = rospy.Time.now()

    def callback_right_motors(self, msg: Float64MultiArray):
        self.right_rpm = (msg.data[0] + msg.data[1]) / 2.0
        self.last_msg_time = rospy.Time.now()

    def timer_callback(self, event):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        if dt <= 0.0:
            return

        # Eger bag bittiyse veya veri akisi durduysa motorlari durdur:
        if (current_time - self.last_msg_time).to_sec() > 0.5:
            self.left_rpm = 0.0
            self.right_rpm = 0.0

        v_left = self.left_rpm *(2* math.pi/60) * self.wheel_radius
        v_right = self.right_rpm *(2* math.pi/60) * self.wheel_radius


        v = (v_left + v_right)/2
        v_omega = (v_right - v_left) / self.track_width
        

        dt_x =v * math.cos(self.theta) *dt
        dt_y =v * math.sin(self.theta) * dt

        self.x += dt_x
        self.y += dt_y

        q = quaternion_from_euler(0, 0, self.theta)
  
        odom_msg = Odometry()

        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
  
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = v_omega

        self.odom_pub_.publish(odom_msg)





def main():
    node = OdometryNode()
    rospy.spin()


if __name__ == "__main__":
    main()
