import csv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
import os


class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher')

        # --- load trajectory data from CSV ---
        pkg_share = get_package_share_directory('manipulator_planning')
        csv_path = os.path.join(pkg_share, 'data', 'trajectory.csv')

        self.times = []
        self.theta1 = []
        self.theta2 = []
        self.theta3 = []
        self.d4 = []

        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.times.append(float(row['time']))
                self.theta1.append(float(row['theta1']))
                self.theta2.append(float(row['theta2']))
                self.theta3.append(float(row['theta3']))
                self.d4.append(float(row['d4'])) 

        self.total_points = len(self.times)
        self.get_logger().info(f'Loaded {self.total_points} trajectory points from {csv_path}')

        # --- current index into the trajectory ---
        self.index = 0

        # --- publisher ---
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # --- figure out the publish rate from the data ---
        # time between consecutive points in your trajectory
        if self.total_points > 1:
            dt = self.times[1] - self.times[0]
        else:
            dt = 0.033  # fallback ~30Hz

        # dt might be very small (your trajectory has ~1090 points over 27.5s, so dt ≈ 0.025s)
        self.get_logger().info(f'Publishing at dt = {dt:.4f}s ({1.0/dt:.1f} Hz)')

        self.timer = self.create_timer(dt, self.timer_callback)

        # --- joint names: must match your URDF joint names exactly ---
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

    def timer_callback(self):
        if self.index >= self.total_points:
            self.get_logger().info('Trajectory finished.')
            self.timer.cancel()
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [
            self.theta1[self.index],
            self.theta2[self.index],
            self.theta3[self.index],
            self.d4[self.index],
        ]

        self.publisher.publish(msg)
        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = TrajectoryPublisher()
    rclpy.spin(trajectory_publisher)
    trajectory_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()