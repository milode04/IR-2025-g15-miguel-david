from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.prev_error = 0  # Store previous error for derivative calculation
        self.integral = 0    # Accumulate error for integral calculation

    def compute(self, error):
        # P: Proportional term (reacts to current error)
        P = self.Kp * error
        
        # I: Integral term, accumulates error over time
        self.integral += error
        I = self.Ki * self.integral
        
        # D: Derivative term, reacts to the rate of change of the error
        D = self.Kd * (error - self.prev_error)
        self.prev_error = error  # Update previous error for next cycle
        
        return P + I + D  # Return total control output

class ProjectFirstNode(Node):
    def __init__(self):
        super().__init__('project_first_node')  # Initialize the ROS2 node with a name
        
        self.left = 0.0   # Variable to store distance to the left
        self.right = 0.0  # Variable to store distance to the right
        self.front = 0.0  # Variable to store distance to the front
        self.ranges = []  # List to store raw lidar data
        
        # CONFIGURATION
        self.target_dist = 0.65  # Distance to front wall to trigger stop
        self.max_sensor_val = 1.5 # "Virtual Wall": max distance considered for PID calculations
        self.turn_speed = 1.0    # Fixed angular velocity for turning
        self.cruise_speed = 0.25 # Linear velocity for moving forward

        # PID TUNING
        # Kp: 0.6 (Responsiveness)
        # Ki: 0.0 (Zero to prevent oscillations/s-shapes)
        # Kd: 0.8 (Damping to smooth out the movement)
        self.pid = PID(Kp=0.6, Ki=0.0, Kd=0.8)

        # LIDAR Subscriber
        self.subscriber_ = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Velocity Publisher
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer (0.1s = 10Hz control loop)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("ProjectFirstNode has been started with STOP & TURN logic")

    def scan_callback(self, msg):
        self.ranges = msg.ranges # Store the full range array
        n = len(msg.ranges)      # Get the number of laser beams

        # Calculate indices for Front, Left, and Right beams
        # NOTE: Assumes standard ROS configuration (0=Front, +pi/2=Left, -pi/2=Right)
        # If your robot uses index 0 for the back, these calculations need adjustment
        front_idx = int((0 - msg.angle_min) / msg.angle_increment)
        left_idx  = int((math.pi/2 - msg.angle_min) / msg.angle_increment)
        right_idx = int((-math.pi/2 - msg.angle_min) / msg.angle_increment)

        # Helper function to process and clean sensor data
        def get_clean_dist(indices):
            valid = []
            for i in range(indices - 5, indices + 5): # Average a window of 10 rays
                # Use modulo n to handle array index wrapping
                dist = msg.ranges[i % n]
                if not math.isinf(dist) and not math.isnan(dist) and dist > 0.0:
                    # CLAMPING: If reading is too far, cap it at 1.5m
                    # This prevents the error from spiking at intersections
                    valid.append(min(dist, self.max_sensor_val))
            
            if not valid:
                return self.max_sensor_val # Return max value if no valid data
            return sum(valid) / len(valid) # Return average of valid rays

        # Helper function specifically for front distance (Real distance, no clamping)
        def get_front_dist(center_idx):
            valid = []
            for i in range(center_idx - 5, center_idx + 5):
                dist = msg.ranges[i % n]
                if not math.isinf(dist) and not math.isnan(dist) and dist > 0.0:
                    valid.append(dist)
            if not valid:
                return 5.0 # Return large distance if clear
            return min(valid) # Return minimum distance for safety

        self.left = get_clean_dist(left_idx)
        self.right = get_clean_dist(right_idx)
        self.front = get_front_dist(front_idx)

    def move_robot(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)   # Set forward speed
        msg.angular.z = float(angular) # Set turning speed
        self.cmd_vel_publisher_.publish(msg)

    def control_loop(self):
        if not self.ranges: # Wait until sensor data is received
            return

        # 1. STATE: FRONT WALL DETECTION (STOP & TURN)
        if self.front < self.target_dist:
            # Stop moving forward
            linear_x = 0.0
            
            # Simple Open Loop logic for turning direction
            # Turn towards the side with more space
            if self.left > self.right:
                # Turn Left
                angular_z = -self.turn_speed
            else:
                # Turn Right
                angular_z = self.turn_speed
                
            self.get_logger().info(f"WALL! Turning... F:{self.front:.2f}")

        # 2. STATE: CRUISE (CORRIDOR CENTERING)
        else:
            linear_x = self.cruise_speed
            
            # Calculate Error: (Left Distance - Right Distance)
            # Logic: If Left (1.0) > Right (0.5), Error is positive (+0.5).
            # We are closer to the right wall, so we need to turn Left.
            # Depending on robot kinematics, Positive Z usually turns Left.
            
            error = self.left - self.right
            correction = self.pid.compute(error)
            
            # IMPORTANT: Clamp the maximum angular velocity from PID
            # Prevents violent shaking or unstable movements
            angular_z = max(min(correction, 1.0), -1.0)
            
            # Debug logging (optional)
            # self.get_logger().info(f"Cruise: Err={error:.2f} Corr={angular_z:.2f}")

        # Send calculated commands to the robot
        self.move_robot(linear_x, angular_z)

def main(args=None):
    rclpy.init(args=args)      # Initialize ROS2 communication
    node = ProjectFirstNode()  # Create node instance
    rclpy.spin(node)           # Keep the node running
    node.destroy_node()        # Cleanup node
    rclpy.shutdown()           # Shutdown ROS2

if __name__ == '__main__':
    main()
