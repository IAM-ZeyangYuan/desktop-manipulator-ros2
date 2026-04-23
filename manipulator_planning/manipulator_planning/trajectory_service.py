import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from manipulator_interfaces.srv import PlanTrajectory
from manipulator_planning.interpolation import plan_trajectory


class TrajectoryService(Node):

    def __init__(self):
        super().__init__('trajectory_service')

        # service server
        self.srv = self.create_service(
            PlanTrajectory,
            'plan_trajectory',
            self.plan_callback
        )

        # publisher for joint states
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # joint names — must match your URDF
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        # trajectory playback state
        self.trajectory = None
        self.index = 0
        self.playback_timer = None

        # publish an initial pose so the arm looks correct before any service call
        self.publish_initial_pose()

        self.get_logger().info('Trajectory service ready. Waiting for requests...')

    def plan_callback(self, request, response):

        if self.idle_timer is not None:
            self.idle_timer.cancel()
            self.idle_timer = None

        self.get_logger().info(
            f'Received request with {len(request.theta1)} waypoints, delta_t={request.delta_t}'
        )

        # validate input
        n = len(request.theta1)
        if not (n == len(request.theta2) == len(request.theta3) == len(request.d4)):
            response.success = False
            response.message = 'All waypoint arrays must have the same length.'
            return response

        if n < 2:
            response.success = False
            response.message = 'Need at least 2 waypoints.'
            return response

        # run interpolation
        try:
            times, th1, th2, th3, d4 = plan_trajectory(
                list(request.theta1),
                list(request.theta2),
                list(request.theta3),
                list(request.d4),
                request.delta_t
            )
        except Exception as e:
            response.success = False
            response.message = f'Planning failed: {str(e)}'
            self.get_logger().error(response.message)
            return response

        # store trajectory and start playback
        self.trajectory = (times, th1, th2, th3, d4)
        self.index = 0

        # compute dt from the trajectory
        if len(times) > 1:
            dt = float(times[1] - times[0])
        else:
            dt = 0.033

        # cancel any existing playback
        if self.playback_timer is not None:
            self.playback_timer.cancel()

        self.playback_timer = self.create_timer(dt, self.playback_callback)

        response.success = True
        response.message = f'Trajectory planned with {len(times)} points. Executing...'
        self.get_logger().info(response.message)
        return response

    def playback_callback(self):
        if self.trajectory is None:
            return

        times, th1, th2, th3, d4 = self.trajectory

        if self.index >= len(times):
            self.get_logger().info('Trajectory execution finished.')
            self.playback_timer.cancel()
            self.trajectory = None
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [
            float(th1[self.index]),
            float(th2[self.index]),
            float(th3[self.index]),
            float(d4[self.index]),
        ]

        self.publisher.publish(msg)
        self.index += 1

    def publish_initial_pose(self):
        """Keep publishing a static pose until a trajectory is requested."""
        self.idle_timer = self.create_timer(0.1,self.idle_callback)

    def idle_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [0.0, 0.0, 0.0, 0.05]
        self.publisher.publish(msg)
        self.get_logger().info('Published initial joint state.')



def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()