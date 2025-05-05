import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import math
import time
from std_msgs.msg import Bool
from builtin_interfaces.msg import Duration


class UR5MotionNode(Node):
    def __init__(self):
        super().__init__('ur5_wave_node')

        # Publishers and subscribers
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

        self.palm_detected_sub = self.create_subscription(
            Bool,
            'palm_detected',
            self.palm_callback,
            10
        )

        # Joint configuration
        self.joint_names = [
            'ur5_shoulder_pan_joint',
            'ur5_shoulder_lift_joint',
            'ur5_elbow_joint',
            'ur5_wrist_1_joint',
            'ur5_wrist_2_joint',
            'ur5_wrist_3_joint'
        ]

        self.max_joint_speed = math.radians(60)  # Max joint speed in rad/s
        self.current_joint_positions = {}

        # Define initial and goal poses
        self.initial_pose = [
            math.radians(48.52),
            math.radians(-169.29),
            math.radians(160.07),
            math.radians(-115.57),
            math.radians(47.77),
            math.radians(-1.41)
        ]

        self.goal_pose = self.initial_pose.copy()
        self.goal_pose[4] = math.radians(127.36)  # waving via wrist_2

        # Palm detection and wave control
        self.palm_detected = False
        self.wave_state = "initial"  # Tracks current state: "initial" or "goal"
        self.last_palm_detection_time = 0
        self.palm_timeout = 1.0  # seconds without palm = lost detection
        self.last_wave_allowed_time = 0  # When next wave can start

        # Create heartbeat timer for control loop
        self.heartbeat_timer = self.create_timer(0.5, self.heartbeat_callback)

        # Wait for joint state info and move to initial pose
        self.get_logger().info("Waiting for joint states...")
        self.wait_for_joint_states()
        self.move_to_pose(self.initial_pose)
        self.get_logger().info("Ready to detect palms and wave.")

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.current_joint_positions[name] = position

    def palm_callback(self, msg):
        current_time = time.time()
        if msg.data:
            if not self.palm_detected:
                self.get_logger().info("Palm detected! Starting wave.")
            self.palm_detected = True
            self.last_palm_detection_time = current_time

    def heartbeat_callback(self):
        current_time = time.time()

        # Check for palm detection timeout
        if self.palm_detected and (current_time - self.last_palm_detection_time > self.palm_timeout):
            self.get_logger().info("Palm detection timeout. Returning to initial pose.")
            self.palm_detected = False
            if self.wave_state != "initial":
                self.move_to_pose(self.initial_pose)
                self.wave_state = "initial"

        # If it's time for the next wave motion
        if self.palm_detected and current_time >= self.last_wave_allowed_time:
            if self.wave_state == "initial":
                self.get_logger().info("Waving! initial → goal")
                duration = self.move_to_pose(self.goal_pose)
                self.wave_state = "goal"
            else:
                self.get_logger().info("Waving! goal → initial")
                duration = self.move_to_pose(self.initial_pose)
                self.wave_state = "initial"

            # Schedule next motion time based on duration + buffer
            self.last_wave_allowed_time = current_time + duration + 0.5  # buffer to settle

    def wait_for_joint_states(self, timeout=5.0):
        start = time.time()
        while not self.has_all_joint_positions():
            if time.time() - start > timeout:
                self.get_logger().warn("Timed out waiting for joint states.")
                break
            rclpy.spin_once(self, timeout_sec=0.1)

    def has_all_joint_positions(self):
        return all(name in self.current_joint_positions for name in self.joint_names)

    def move_to_pose(self, target_positions):
        current_positions = [
            self.current_joint_positions.get(name, 0.0) for name in self.joint_names
        ]

        deviations = [abs(c - t) for c, t in zip(current_positions, target_positions)]
        max_deviation = max(deviations) if deviations else 0

        if max_deviation < math.radians(5):
            self.get_logger().info("Already at target pose, skipping motion.")
            return 0.0  # No delay needed

        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * len(target_positions)

        # Compute adaptive duration
        max_delta = max(deviations)
        duration_sec = max(1.0, max_delta / self.max_joint_speed)

        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1.0) * 1e9)
        )

        traj_msg.points.append(point)
        self.publisher_.publish(traj_msg)

        self.get_logger().info(f"Sent trajectory with duration {duration_sec:.2f}s")
        return duration_sec  # Return so control loop can wait accordingly


def main(args=None):
    rclpy.init(args=args)
    node = UR5MotionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
