import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import math
import time
from builtin_interfaces.msg import Duration


class UR5MotionNode(Node):
    def __init__(self):
        super().__init__('ur5_wave_node')

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.joint_names = [
            'ur5_shoulder_pan_joint',
            'ur5_shoulder_lift_joint',
            'ur5_elbow_joint',
            'ur5_wrist_1_joint',
            'ur5_wrist_2_joint',
            'ur5_wrist_3_joint'
        ]

        self.current_joint_positions = {}

        # Example poses
        self.initial_pose = [
            math.radians(48.52),
            math.radians(-169.29),
            math.radians(160.07),
            math.radians(-115.57),
            math.radians(47.77),
            math.radians(-1.41)
        ]

        # self.goal_pose = [
        #     math.radians(48.52),
        #     math.radians(-169.29),
        #     math.radians(160.07),
        #     math.radians(-115.57),
        #     math.radians(127.36),  # different only in wrist_2
        #     math.radians(-1.41)
        # ]
        self.goal_pose = self.initial_pose.copy()
        self.goal_pose[4] = math.radians(127.36)  # waving motion via wrist_2_joint

        self.get_logger().info("Waiting for joint states...")
        self.wait_for_joint_states()

        # General motion plan: just plug in goal states here
        self.move_to_pose(self.initial_pose)
        self.move_to_pose(self.goal_pose)
        self.move_to_pose(self.initial_pose)

        self.get_logger().info("Motion sequence complete.")

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.current_joint_positions[name] = position

    def wait_for_joint_states(self, timeout=5.0):
        start = time.time()
        while not self.has_all_joint_positions():
            if time.time() - start > timeout:
                self.get_logger().warn("Timed out waiting for joint states.")
                break
            rclpy.spin_once(self, timeout_sec=0.1)

    def has_all_joint_positions(self):
        return all(name in self.current_joint_positions for name in self.joint_names)

    def move_to_pose(self, target_positions, threshold=math.radians(1)):
        self.send_trajectory(target_positions)
        self.wait_until_reached(target_positions, threshold)

    def send_trajectory(self, joint_positions):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = [0.0] * len(joint_positions)

        # Current joint positions
        current_positions = [
            self.current_joint_positions.get(name, 0.0) for name in self.joint_names
        ]

        max_delta = max(abs(curr - goal) for curr, goal in zip(current_positions, joint_positions))
        max_joint_speed = math.radians(30)  # conservative 30 deg/s
        duration_sec = max(1.0, max_delta / max_joint_speed)

        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1.0) * 1e9)
        )

        traj_msg.points.append(point)
        self.publisher_.publish(traj_msg)

        self.get_logger().info(f"Sent trajectory with duration {duration_sec:.2f}s")

    def wait_until_reached(self, goal_positions, threshold, timeout=10.0):
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            current_positions = [
                self.current_joint_positions.get(name, 0.0) for name in self.joint_names
            ]
            max_error = max(abs(c - g) for c, g in zip(current_positions, goal_positions))
            if max_error < threshold:
                self.get_logger().info("Reached goal.")
                return
        self.get_logger().warn("Timeout while waiting for robot to reach goal.")


def main(args=None):
    rclpy.init(args=args)
    time.sleep(10) # for testing
    node = UR5MotionNode()
    rclpy.spin_once(node, timeout_sec=0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
