#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from hb_interfaces.msg import Pose2D, Poses2D
import numpy as np
import math

#PID Controller Class
class PID:
    def __init__(self, kp, ki, kd, max_out=1.0, integral_max=2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral_max = integral_max
        self.integral = 0.0
        self.prev_error = 0.0

#PID Compute Steps
    def compute(self, error, dt):
    #Track how much total error has built up over time
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_max), -self.integral_max)
    #Calculate rate of change of error over time
        derivative = (error - self.prev_error) / dt if dt>0 else 0.0
    #Calculate the PID output
        output = (self.kp*error) + (self.ki*self.integral) + (self.kd*derivative)
    #Save the current error
        self.prev_error = error
    #limits the output
        if output > self.max_out:
            output = self.max_out
        elif output < -self.max_out:
            output = -self.max_out
        return output
    
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


#Main Node Class
class HolonomicPIDController(Node):
    def __init__(self):
        super().__init__('holonomic_pid_controller') 

    #Robot Parameters
        self.bot_id = 9
        self.current_pose = None
        self.goal_index = 0
        self.last_time = self.get_clock().now()
        self.max_vel = 1.0
        self.L = 0.350  #Distance between robot center to wheel center
        R = 0.05  #Wheel Radius
        self.pose_threshold = 0.01
        self.yaw_threshold = 0.087
        self.goal_logged = False 

    #Goal Definitions
        self.goals = [
            (0.820, 0.920, 0.0),
            (0.820, 1.520, 0.0),
            (1.620, 1.520, 0.0),
            (1.620, 0.920, 0.0),
            (0.820, 0.920, 0.0),
        ]
        self.num_goals = len(self.goals)

    #PID Parameters 
        self.pid_params = {
            'x': {'kp': 9.0, 'ki': 0.02,'kd': 0.1, 'max_out': self.max_vel},
            'y': {'kp': 9.0, 'ki': 0.02,'kd': 0.1, 'max_out': self.max_vel},
            'theta': {'kp': 0.09,'ki': 0.001, 'kd': 0.02, 'max_out': 2.0}
        }


        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_theta = PID(**self.pid_params['theta'])

    #Publishers & Subscribers
        self.subscriber = self.create_subscription(Poses2D, "/bot_pose", self.pose_cb, 10)
        self.publisher = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        
    #Timer for Control Loop
        self.timer = self.create_timer(0.03, self.control_cb)  # ~30ms = 33 Hz
        self.get_logger().info(f'Holonomic PID Controller started. Goals: {self.num_goals} waypoints defined.')


#Subscriber Callback
    def pose_cb(self, msg):
        for pose in msg.poses:
            if pose.id == self.bot_id:
                yaw_in_radians = math.radians(pose.w)
                pose.x = pose.x / 1000.0
                pose.y = pose.y / 1000.0
                self.current_pose = [pose.x, pose.y, yaw_in_radians]
                return

#Inverse Kinematics
    def inverse_kinematics(self, vx, vy, omega):
        a1 = math.pi / 6 
        a2 = 5*math.pi / 6
        a3 = 3*math.pi / 2
        L = self.L
        v1 = -vx*math.sin(a1) + vy*math.cos(a1) + L*omega
        v2 = -vx*math.sin(a2) + vy*math.cos(a2) + L*omega
        v3 = -vx*math.sin(a3) + vy*math.cos(a3) + L*omega
        wheel_vel = [v1, v2, v3] 
        return wheel_vel

#Control Loop
    def control_cb(self):
        if self.current_pose is None:
            return
        
    #Timing
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now
        current_x, current_y, current_theta = self.current_pose

    #Goal Check
        if self.goal_index >= self.num_goals:
            self.get_logger().info("All goals completed. Stopping robot.")
            self.publish_wheel_velocities([0.0, 0.0, 0.0])
            return
        
    #Current Goal
        target_x, target_y, target_theta = self.goals[self.goal_index]

        if not self.goal_logged:
            self.get_logger().info(f"Moving to Goal {self.goal_index+1}/{self.num_goals}: X={target_x}, Y={target_y}, Yaw={target_theta} deg")
            self.goal_logged = True 

    #World Frame Error Calculation
        error_x_world = target_x - current_x
        error_y_world = target_y - current_y
        error_theta = target_theta - current_theta

    #Angle Normalization
        error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))

    #Robot Frame Error Transformation
        cos_t = math.cos(current_theta)
        sin_t = math.sin(current_theta)
        error_x_robot = error_x_world*cos_t + error_y_world*sin_t
        error_y_robot = -error_x_world*sin_t + error_y_world*cos_t

        vx = self.pid_x.compute(error_x_robot, dt) #forward/backward velocity
        vy = self.pid_y.compute(error_y_robot, dt) #right/left velocity
        omega = self.pid_theta.compute(error_theta, dt) #rotation around z-axis
        
        wheel_vel = self.inverse_kinematics(vx, vy, omega)
        self.publish_wheel_velocities(wheel_vel)
        
    #Check if target is achieved
        dist_to_goal = math.sqrt(error_x_world**2 + error_y_world**2)

        if dist_to_goal < self.pose_threshold and abs(error_theta) < self.yaw_threshold:
        #Goal Reached and Prepare for Next Goal
            self.get_logger().info(f"Goal {self.goal_index+1}/{self.num_goals} reached. Position: ({current_x:.4f},{current_y:.4f}, {current_theta:.4f}).")
            
        #Move to next goal
            self.goal_index += 1
            
        #Reset PIDs
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_theta.reset()
            
        #Stops the robot immediately
            self.publish_wheel_velocities([0.0, 0.0, 0.0])
            self.goal_logged = False

#Publisher
    def publish_wheel_velocities(self, wheel_vel):
        msg = Float64MultiArray()
        msg.data = np.array(wheel_vel).tolist()
        self.publisher.publish(msg)


#Main Function
def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()