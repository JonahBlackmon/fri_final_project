import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import math
import time
from std_msgs.msg import Bool
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float32


class MimicNode(Node):
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

        self.angle_sub = self.create_subscription(
            Float32,
            'middle_finger_angle',
            self.angle_callback,
            10
        )

        self.latest_middle_finger_angle = None  # Store the latest received angle
        self.last_sent_angle = None
        self.last_move_time = time.time()
        self.angle_change_threshold = math.radians(2)  # Minimum 2° change
        self.move_interval = 0.5  # Minimum 0.5s between motions

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
            math.radians(87.77),
            math.radians(-1.41)
        ]

        self.goal_pose = self.initial_pose.copy()
        # Change to be the subscribed angle
        self.goal_pose[4] = math.radians(127.36)  # waving via wrist_2

        # Wait for joint state info and move to initial pose
        self.get_logger().info("Waiting for joint states...")
        self.wait_for_joint_states()
        self.move_to_pose(self.initial_pose)
        self.get_logger().info("Ready to detect")

        self.last_sent_angle = None  # Add in __init__

    def angle_callback(self, msg):
        new_angle_deg = msg.data
        self.get_logger().info(f"Original angle: {new_angle_deg}°")
        
        # Step 1: First, reflect around 180° as before
        mirrored_angle = 360 - new_angle_deg
        
        # Step 2: Now shift the center from 180° to 80°
        # We need to subtract 100° (the difference between 180° and 80°)
        adjusted_angle = mirrored_angle - 100
        
        # Step 3: Normalize to 0-360 range
        if adjusted_angle < 0:
            adjusted_angle += 360
        elif adjusted_angle >= 360:
            adjusted_angle -= 360
        
        # Convert to radians for the UR5
        adjusted_angle_rad = math.radians(adjusted_angle)
        
        # Log the transformation
        self.get_logger().info(f"Hand angle: {new_angle_deg}° → Robot angle: {adjusted_angle}° (centered at 80°)")
        
        # Check if the angle change is significant and if enough time has passed
        now = time.time()
        time_since_last_move = now - self.last_move_time
        
        angle_diff_ok = (
            self.last_sent_angle is None or 
            abs(adjusted_angle_rad - self.last_sent_angle) >= self.angle_change_threshold
        )
        
        time_ok = time_since_last_move >= self.move_interval
        
        if angle_diff_ok and time_ok:
            # Set the goal pose to the adjusted angle for the wrist
            self.goal_pose[4] = adjusted_angle_rad
            self.latest_middle_finger_angle = adjusted_angle  # Store as degrees
            self.last_sent_angle = adjusted_angle_rad
            self.last_move_time = now
            
            self.get_logger().info(
                f"Moving to new wrist angle: {adjusted_angle_rad:.2f} rad "
                f"(Δangle ≥ {self.angle_change_threshold:.2f} rad, Δt ≥ {self.move_interval:.2f}s)"
            )
            self.move_to_pose(self.goal_pose)
        else:
            reason = []
            if not angle_diff_ok:
                reason.append("angle change too small")
            if not time_ok:
                reason.append(f"only {time_since_last_move:.2f}s since last move")
            self.get_logger().debug(f"Skipping move: {', '.join(reason)}")

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
    node = MimicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()