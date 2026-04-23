import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from manipulator_planning.interpolation import plan_trajectory

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from manipulator_planning.forward_kinematics import fk

class TrajectoryActionClient(Node):

    def __init__(self):
        super().__init__('trajectory_action_client')

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.marker_pub = self.create_publisher(MarkerArray, '/trajectory_markers', 10) #the second entry is the topic name

    def send_trajectory(self, theta1_wp, theta2_wp, theta3_wp, d4_wp, delta_t):
        self.get_logger().info('Running interpolation...')

        times, th1, th2, th3, d4 = plan_trajectory(
            theta1_wp, theta2_wp, theta3_wp, d4_wp, delta_t
        )

        self.publish_markers(times, th1, th2, th3, d4,
                     theta1_wp, theta2_wp, theta3_wp, d4_wp)

        self.get_logger().info(f'Planned {len(times)} points. Sending goal...')

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        for i in range(len(times)):
            point = JointTrajectoryPoint()
            point.positions = [
                float(th1[i]),
                float(th2[i]),
                float(th3[i]),
                float(d4[i]),
            ]
            sec = int(times[i])
            nanosec = int((times[i] - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            goal_msg.trajectory.points.append(point)

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        #since now the markers are only published once and might arrive before rviz2 is ready,
        #so we keep publishing them
        # store for re-publishing
        self._cached_markers_args = (times, th1, th2, th3, d4,
                              theta1_wp, theta2_wp, theta3_wp, d4_wp)
        self._marker_timer = self.create_timer(1.0, self._republish_markers)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            return
        self.get_logger().info('Goal accepted. Executing trajectory...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        # you can log progress here if you want
        pass

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Trajectory finished. Error code: {result.error_code}')

    #visualizing the trajectory and the waypoints    
    def publish_markers(self, times, th1, th2, th3, d4,
                        theta1_wp, theta2_wp, theta3_wp, d4_wp):
        markers = MarkerArray()

        # --- trajectory path as a line strip ---
        path_marker = Marker()
        path_marker.header.frame_id = 'base_link'  # match your URDF base link name
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = 'trajectory_path'
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.003  # line width in meters
        path_marker.color = ColorRGBA(r=0.2, g=0.7, b=1.0, a=0.8)
        path_marker.pose.orientation.w = 1.0

        # sample every Nth point to keep it lightweight
        step = max(1, len(times) // 200)
        for i in range(0, len(times), step):
            x, y, z = fk(th1[i], th2[i], th3[i], d4[i])
            path_marker.points.append(Point(x=x, y=y, z=z))

        markers.markers.append(path_marker)

        # --- waypoints as spheres ---
        for i in range(len(theta1_wp)):
            wp_marker = Marker()
            wp_marker.header.frame_id = 'base_link'
            wp_marker.header.stamp = self.get_clock().now().to_msg()
            wp_marker.ns = 'waypoints'
            wp_marker.id = i
            wp_marker.type = Marker.SPHERE
            wp_marker.action = Marker.ADD

            x, y, z = fk(theta1_wp[i], theta2_wp[i], theta3_wp[i], d4_wp[i])
            wp_marker.pose.position.x = x
            wp_marker.pose.position.y = y
            wp_marker.pose.position.z = z
            wp_marker.pose.orientation.w = 1.0

            wp_marker.scale.x = 0.015  # sphere diameter in meters
            wp_marker.scale.y = 0.015
            wp_marker.scale.z = 0.015
            wp_marker.color = ColorRGBA(r=1.0, g=0.3, b=0.2, a=1.0)

            markers.markers.append(wp_marker)

        self.marker_pub.publish(markers)
        self.get_logger().info(
            f'Published {len(theta1_wp)} waypoint markers + trajectory path'
        )

    def _republish_markers(self):
        if self._cached_markers_args is not None:
            self.publish_markers(*self._cached_markers_args)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryActionClient()

    # your original 12 waypoints
    import numpy as np
    theta1_wp = [-np.pi/4, np.pi/4, np.pi/8, -np.pi/8,
                 -np.pi/4, np.pi/4, np.pi/8, -np.pi/8,
                 -np.pi/4, np.pi/4, np.pi/8, -np.pi/8]

    theta2_wp = [-np.pi/6, -np.pi/6, -np.arctan(3/4), -np.arctan(3/4),
                 np.arctan(5)-np.pi, np.arctan(5)-np.pi,
                 np.arctan(6)-np.pi, np.arctan(6)-np.pi,
                 -np.pi*5/6, -np.pi*5/6,
                 np.arctan(3/4)-np.pi, np.arctan(3/4)-np.pi]
    theta2_wp = np.array(theta2_wp) + np.pi/2

    theta3_wp = [-np.arccos((21*np.sqrt(3)-35)/5),
                 np.arccos((21*np.sqrt(3)-35)/5),
                 np.arccos(1/30), -np.arccos(1/30),
                 np.arccos(-16/25), -np.arccos(-16/25),
                 2/3*np.pi, -2/3*np.pi,
                 np.arccos((21*np.sqrt(3)-35)/5)-np.pi,
                 -np.arccos((21*np.sqrt(3)-35)/5)+np.pi,
                 -np.arccos(1/30)+np.pi, np.arccos(1/30)-np.pi]
    theta3_wp = np.array(theta3_wp) - np.pi/2


    d4_wp = [0.14, 0.14, 0.15, 0.15,
             0.09, 0.09, 0.08, 0.08,
             0.14, 0.14, 0.15, 0.15]

    node.send_trajectory(theta1_wp, theta2_wp, theta3_wp, d4_wp, delta_t=2.5)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()