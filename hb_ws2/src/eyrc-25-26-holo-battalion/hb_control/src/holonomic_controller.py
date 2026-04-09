#!/usr/bin/env python3

'''
This Python file runs a ROS 2 node of name holonomic_pid_controller which holds the position of a holonomic robot
and drives it through a series of predefined goals using PID controllers on [x, y, θ].

This node publishes and subscribes to the following topics:

        PUBLICATIONS                               SUBSCRIPTIONS
        /forward_velocity_controller/commands      /bot_pose

Instead of defining separate variables for each PID axis, lists/dictionaries are used.
For example: pid_params['x'], pid_params['y'], pid_params['theta'], etc.

Code modularity and clarity are maintained to make tuning and extension easier.
'''

# ---------------------- Import Required Libraries ----------------------------
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
# import hb_interface messages
import numpy as np
import math


# ---------------------- PID Controller Class --------------------------------
class PID:
    def __init__(self, kp, ki, kd, max_out=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
#-----------------------------PID Compute Steps--------------------------------------------------------------
        # 1. Accumulate the error over time for the Integral term
        # 2. Compute the change in error for the Derivative term
        # 3. Calculate the PID output:
        # 4. Store the current error for use in the next iteration
        # 5. Limit (clip) the output between [-max_out, +max_out] to avoid unsafe velocities
#------------------------------------------------------------------------------------------------------------
        return 
    
    
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


# ---------------------- Main Node Class -------------------------------------
class HolonomicPIDController(Node):
    def __init__(self):
        super().__init__('holonomic_pid_controller')  # initializing ros node

        # ---------------- Robot Parameters ----------------
        # 1. Robot ID(s)
        # 2. Current pose of the robot:
        #    - Updated from the /bot_pose topic in the callback function.
        #    - Stores [x, y, θ] information for the active robot.
        # 3. Goal tracking index
        # 4. Timing information:
        #    - Used to calculate the time difference (dt) between control loop iterations.
        # 5. Threshold for goal completion:
        #    - Defines the acceptable error tolerance for x, y, and θ.
        #    - Example: if error < 5 units → goal considered reached.

        # ---------------- Goal Definitions ----------------

        # List of waypoints [(x, y, yaw_deg)]
        self.goals = [
            (700, 800, 0),
            (700, 1400, 0),
            (1500, 1400, 0),
            (1500, 800, 0),
            (700, 800, 0),
        ]

        #----------------DO NOT CHNAGE----------------------

        # ---------------- PID Parameters ----------------
        self.pid_params = {
            'x': {'kp': 0.0, 'ki': 0.00, 'kd': 0.0, 'max_out': self.max_vel},
            'y': {'kp': 0.0, 'ki': 0.00, 'kd': 0.0, 'max_out': self.max_vel},
            'theta': {'kp': 0.0, 'ki': 0.00, 'kd': 0.0, 'max_out': self.max_vel * 2}
        }

        # Initialize PIDs
        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_theta = PID(**self.pid_params['theta'])

        # ---------------- ROS 2 Publishers & Subscribers ----------------
        
        # Write a subscriber for /bot_pose

        self.publisher = self.create_publisher(
            Float64MultiArray, '/forward_velocity_controller/commands', 10
        )
        
        # ---------------- Timer for Control Loop ----------------
        self.timer = self.create_timer(0.03, self.control_cb)  # ~30ms = 33 Hz

        self.get_logger().info(f'Holonomic PID Controller started. Goals: {self.goals}')


    # ---------------- Subscriber Callback ----------------
    def pose_cb(self, msg):
        """
        Callback function for /bot_pose topic.
        This function is executed each time a message is received.

        Steps:
        1. Iterate through all poses in the incoming message.
        2.  Update self.current_pose with this robot’s pose.
        """

    # ---------------- Control Loop ----------------
    def control_cb(self):

        """
        Control loop callback executed periodically by the ROS 2 timer.

        Main Steps:
        1. Check if the current pose is available; if not, exit.
        2. Compute the time difference (dt) since the last control cycle.
        3. Get the current robot pose (x, y, θ).
        4. If all goals are completed → stop the robot.
        5. Select the current goal (x, y, θ) from the goals list.
        6. Compute errors in x, y, and θ between current pose and goal.
        7. Use PID controllers to calculate required body velocities [vx, vy, ω].
        8. Convert body velocities into individual wheel velocities.
        9. Limit (clip) wheel velocities within safe bounds.
        10. Publish the wheel velocities to the motor controller.
        11. Check if the goal is reached:
              - If yes → update goal index, reset PIDs, and move to the next goal.
        """


        # Time delta
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now

        # Current robot pose

        # If all goals are reached → stop

        # Current target goal

        # Errors

        # PID outputs

        # Convert to wheel velocities (custom equations)

        # Publish wheel velocities

        # Goal check


    # ---------------- Publisher ----------------
    def publish_wheel_velocities(self, wheel_vel):
        # Wheel velocity array (Float64MultiArray)
        # Order: [Left wheel speed, Right wheel speed, Rear wheel speed]
        msg = Float64MultiArray()
        msg.data = np.array(wheel_vel).tolist()
        self.publisher.publish(msg)


# ---------------------- Main Function -------------------------------------
def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
