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

        self.joint_names = [
            'ur5_shoulder_pan_joint',
            'ur5_shoulder_lift_joint',
            'ur5_elbow_joint',
            'ur5_wrist_1_joint',
            'ur5_wrist_2_joint',
            'ur5_wrist_3_joint'
        ]

        self.current_joint_positions = {}

        self.initial_pose = [
            math.radians(48.52),
            math.radians(-169.29),
            math.radians(160.07),
            math.radians(-115.57),
            math.radians(47.77),
            math.radians(-1.41)
        ]

        self.goal_pose = self.initial_pose.copy()
        self.goal_pose[4] = math.radians(127.36)

        # State variables
        self.palm_detected = False
        self.wave_state = "initial"  # States: "initial", "goal"
        self.last_palm_detection_time = 0
        self.last_wave_time = 0
        self.wave_interval = 3.0  # Time between wave motions
        self.palm_timeout = 1.0  # Consider palm lost if no detection for this long
        
        # Heartbeat timer to manage wave state and check for palm timeouts
        self.heartbeat_timer = self.create_timer(0.5, self.heartbeat_callback)
        
        # Make sure we have joint states before starting
        self.get_logger().info("Waiting for joint states...")
        self.wait_for_joint_states()

        # Move to initial position at startup
        self.move_to_pose(self.initial_pose)
        self.get_logger().info("Ready to detect palms and wave.")

    def joint_state_callback(self, msg):
        """Store joint positions from joint state messages"""
        for name, position in zip(msg.name, msg.position):
            self.current_joint_positions[name] = position

    def palm_callback(self, msg):
        """Handle palm detection messages"""
        current_time = time.time()
        
        if msg.data:  # Palm detected
            if not self.palm_detected:
                self.palm_detected = True
                self.get_logger().info("Palm detected! Starting to wave.")
            
            # Update last detection time
            self.last_palm_detection_time = current_time
    
    def heartbeat_callback(self):
        """Main control loop - manages palm detection timeouts and waving"""
        current_time = time.time()
        
        # Check if palm detection has timed out
        if self.palm_detected and (current_time - self.last_palm_detection_time > self.palm_timeout):
            self.palm_detected = False
            self.get_logger().info("Palm detection timeout. Stopping wave.")
            # Return to initial pose when detection is lost
            if self.wave_state != "initial":
                self.move_to_pose(self.initial_pose)
                self.wave_state = "initial"
        
        # If palm is detected and it's time for next wave motion
        if self.palm_detected and (current_time - self.last_wave_time > self.wave_interval):
            if self.wave_state == "initial":
                self.get_logger().info("Waving! Current state: initial -> goal")
                self.move_to_pose(self.goal_pose)
                self.wave_state = "goal"
            else:
                self.get_logger().info("Waving! Current state: goal -> initial")
                self.move_to_pose(self.initial_pose)
                self.wave_state = "initial"
                
            self.last_wave_time = current_time

    def wait_for_joint_states(self, timeout=5.0):
        """Wait until we have received joint states for all joints"""
        start = time.time()
        while not self.has_all_joint_positions():
            if time.time() - start > timeout:
                self.get_logger().warn("Timed out waiting for joint states.")
                break
            rclpy.spin_once(self, timeout_sec=0.1)

    def has_all_joint_positions(self):
        """Check if we have received positions for all joints"""
        return all(name in self.current_joint_positions for name in self.joint_names)

    def move_to_pose(self, target_positions):
        """Send a trajectory to move the robot to the specified joint positions"""
        # Check if we're already at the target pose
        current_positions = [
            self.current_joint_positions.get(name, 0.0) for name in self.joint_names
        ]
        
        # Calculate max deviation from target positions
        deviations = [abs(c - t) for c, t in zip(current_positions, target_positions)]
        max_deviation = max(deviations) if deviations else 0
        
        # If we're already close to the target pose, don't send redundant commands
        if max_deviation < math.radians(5):  # Within 5 degrees
            self.get_logger().info("Already at target pose, skipping movement")
            return
            
        # Create and send trajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * len(target_positions)

        max_delta = max(abs(curr - goal) for curr, goal in zip(current_positions, target_positions))
        max_joint_speed = math.radians(30)  # 30 deg/s
        duration_sec = max(1.0, max_delta / max_joint_speed)

        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1.0) * 1e9)
        )

        traj_msg.points.append(point)
        self.publisher_.publish(traj_msg)

        self.get_logger().info(f"Sent trajectory with duration {duration_sec:.2f}s")


def main(args=None):
    rclpy.init(args=args)
    node = UR5MotionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()