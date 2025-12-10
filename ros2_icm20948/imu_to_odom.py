import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math, time

class FakeOdom(Node):
    def __init__(self):
        super().__init__('fake_odom')
        self.publisher = self.create_publisher(Odometry, 'odom', 10)
        self.subscription = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.timer = self.create_timer(0.1, self.publish_odom)  # 10 Hz

        self.x = 0.0
        self.y = 0.0
        self.last_time = time.time()
        self.orientation = None
        self.yaw = 0.0

    def imu_callback(self, msg):
        # Stocke orientation quaternion
        self.orientation = msg.orientation
        # Convertit quaternion en yaw
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def publish_odom(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # Vitesse fictive (m/s) → tu peux mapper ton PWM=50 à ~0.05 m/s
        v = 0.05  
        self.x += v * dt * math.cos(self.yaw)
        self.y += v * dt * math.sin(self.yaw)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        # Orientation réelle de l’IMU
        if self.orientation:
            odom.pose.pose.orientation = self.orientation

        self.publisher.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = FakeOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
