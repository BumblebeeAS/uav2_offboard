import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleOdometry
from sensor_msgs.msg import Imu
from uav2_offboard.utils.qos_profiles import QOS_PROFILE_SUB


class ImuRepubNode(Node):

    def __init__(self):
        super().__init__('imu_repub')

        self.sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.callback,
            QOS_PROFILE_SUB
        )

        self.pub = self.create_publisher(
            Imu,
            '/imu',
            10
        )

    def callback(self, msg: VehicleOdometry):
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "uav2/base_link_frd"

        imu.orientation.w = msg.q[0]
        imu.orientation.x = msg.q[1]
        imu.orientation.y = msg.q[2]
        imu.orientation.z = msg.q[3]

        imu.angular_velocity.x = msg.angular_velocity[0]
        imu.angular_velocity.y = msg.angular_velocity[1]
        imu.angular_velocity.z = msg.angular_velocity[2]

        imu.linear_acceleration.x =  msg.acceleration[0]
        imu.linear_acceleration.y = msg.acceleration[1]
        imu.linear_acceleration.z = msg.acceleration[2]

        self.pub.publish(imu)


def main():
    rclpy.init()
    node = ImuRepubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
