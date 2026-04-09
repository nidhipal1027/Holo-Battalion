#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from eyantrasim_msgs.msg import Pose
import math

#Here we created a ROS2 node called "BattalionController"
class BattalionController(Node):                 
    def __init__(self):
        super().__init__('battalion_controller')

        # --- Service client (for targets) ---
        self.cli = self.create_client(Trigger, '/eyantrasim/get_coordinates')    #HERE the client call the service(/eyantrasim/get_coordinates) where the type is std_srvs/Trigger
        self.req = Trigger.Request()

        # --- Publishers ( to send velocity commands for bots) ---
        self.pub_glacio = self.create_publisher(Twist, '/eyantrasim/glacio/cmd_vel', 10)            #here the topic is Twist messages so that it commands linear and angular velocity
        self.pub_crystal = self.create_publisher(Twist, '/eyantrasim/crystal/cmd_vel', 10)
        self.pub_frostbite = self.create_publisher(Twist, '/eyantrasim/frostbite/cmd_vel', 10)

        # --- Subscribers (so that we can have current poses for bot) ---
        self.sub_glacio = self.create_subscription(Pose, '/eyantrasim/glacio/pose', self.pose_glacio_cb, 10)
        self.sub_crystal = self.create_subscription(Pose, '/eyantrasim/crystal/pose', self.pose_crystal_cb, 10)
        self.sub_frostbite = self.create_subscription(Pose, '/eyantrasim/frostbite/pose', self.pose_frostbite_cb, 10)

        # --- Storage for targets ---
        self.targets_glacio = []
        self.targets_crystal = []
        self.targets_frostbite = []

        # --- Current placeholder ---  
        self.current_pose_glacio = None        #here callbacks save the lastest pose into variables 
        self.current_pose_crystal = None 
        self.current_pose_frostbite = None

        # --- target index ---
        self.target_idx_glacio = 0      #targets index says the current target point aim by bots
        self.target_idx_crystal = 0
        self.target_idx_frostbite = 0

        # --- Controller gains and thresholds ---
        self.Kp_linear = 0.5            
        self.Kp_angular = 1.5        
        self.linear_threshold = 5.0 
        self.angular_threshold = 0.05
        
        # Call the method to get targets 
        self.get_targets()

        # --- Timer for the control loop ---
        self.timer = self.create_timer(0.1, self.control_loop)

    # --- Request targets from service ---
    def get_targets(self):
        self.get_logger().info('Calling /eyantrasim/get_coordinates service to get targets...')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.targets_callback)

    def targets_callback(self, future):
    # Processes the service response and assigns coordinates.
        try:
            response = future.result()
            if response.success:
               # The service response gives tuples in string form, so we need to convert them into real tuples
                coords_str = response.message[1:-1].replace('), (', ');(').replace(' ', '')
                coordinates = coords_str.split(';')
                
                # Convert string tuples to actual tuples of integers
                poses = [tuple(map(int, coord[1:-1].split(','))) for coord in coordinates]
                
                # Assign poses to the bots 
                self.targets_glacio = poses[0:50]
                self.targets_crystal = poses[50:100]
                self.targets_frostbite = poses[100:150]
                
                self.get_logger().info('Successfully received and assigned 150 target coordinates to the bots.')
                self.get_logger().info(f'Glacio\'s assigned targets: {self.targets_glacio}')  # Prints assigned targets of Glacio
                self.get_logger().info(f'Crystal\'s assigned targets: {self.targets_crystal}') # Prints assigned targets of Crystal
                self.get_logger().info(f'Frostbite\'s assigned targets: {self.targets_frostbite}') # Prints assigned targets of frostbite
            else:
                self.get_logger().error(f'Service call failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed with exception: {e}')

    # --- Pose callbacks for each bot ---
    def pose_glacio_cb(self, msg):
        self.current_pose_glacio = msg

    def pose_crystal_cb(self, msg):
        self.current_pose_crystal = msg

    def pose_frostbite_cb(self, msg):
        self.current_pose_frostbite = msg

    # --- P-controller for movement ---
    def compute_velocity(self, current_pose, target_pose):
        """Calculates Twist velocity based on a simple P-controller."""
        vel = Twist()
        
        dx = target_pose[0] - current_pose.x
        dy = target_pose[1] - current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Check if the bot has reached the current target 
        if distance < self.linear_threshold:
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            return vel, True # Target reached

        angle_to_target = math.atan2(dy, dx)
        angle_diff = angle_to_target - current_pose.theta
        
        # Normalize angle_diff to be between -pi and pi
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        # Proportional control for angular velocity
        if abs(angle_diff) > self.angular_threshold:
            vel.angular.z = self.Kp_angular * angle_diff
            vel.linear.x = 0.0  # Stop linear movement while turning
        else:
            vel.angular.z = 0.0
            vel.linear.x = min(self.Kp_linear * distance, 0.5) # Cap linear velocity

        return vel, False # Target not reached

    # --- Main control loop ---      #here every 0.1s control_loop() runs in this way each bot will move from target 0 to target 1 to target 2 to _ _ _ _
    def control_loop(self):
        """Sends velocity commands to all bots based on their current targets."""
        # Glacio's control
        if self.current_pose_glacio and self.targets_glacio and self.target_idx_glacio < len(self.targets_glacio):
            vel, reached = self.compute_velocity(self.current_pose_glacio, self.targets_glacio[self.target_idx_glacio])
            self.pub_glacio.publish(vel)
            if reached:
                self.target_idx_glacio += 1
                self.get_logger().info(f'Glacio reached target {self.target_idx_glacio - 1}. Moving to the next one.')

        # Crystal's control
        if self.current_pose_crystal and self.targets_crystal and self.target_idx_crystal < len(self.targets_crystal):
            vel, reached = self.compute_velocity(self.current_pose_crystal, self.targets_crystal[self.target_idx_crystal])
            self.pub_crystal.publish(vel)
            if reached:
                self.target_idx_crystal += 1
                self.get_logger().info(f'Crystal reached target {self.target_idx_crystal - 1}. Moving to the next one.')

        # Frostbite's control
        if self.current_pose_frostbite and self.targets_frostbite and self.target_idx_frostbite < len(self.targets_frostbite):
            vel, reached = self.compute_velocity(self.current_pose_frostbite, self.targets_frostbite[self.target_idx_frostbite])
            self.pub_frostbite.publish(vel)
            if reached:
                self.target_idx_frostbite += 1
                self.get_logger().info(f'Frostbite reached target {self.target_idx_frostbite - 1}. Moving to the next one.')


def main(args=None):         
    rclpy.init(args=args)       #main() function
    node = BattalionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    #IN THIS CODE: Node start requests targets to them per robot then each robot publish its pose which is stored in callback variables.
    #then timer trigger control_loop() which computs velocities using p controller so its publishes bots.
    #then the bots reaches from one target to the another target until it reach its destination.
    #and then we get the output "THE FLOWER" by three bots(GLACIO,CRYSTAL,AND FROSTBITE