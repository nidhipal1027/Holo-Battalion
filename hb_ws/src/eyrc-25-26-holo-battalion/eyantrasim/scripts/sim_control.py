#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from eyantrasim_msgs.msg import Pose

class BattalionController(Node):
    def __init__(self):
        super().__init__('battalion_controller')

        # --- Service client (for targets) ---
        # self.cli = self.create_client(Trigger, '/eyantrasim/get_coordinates')
        # self.req = Trigger.Request()

        # --- Publishers (for bots) ---
        self.pub_glacio = self.create_publisher(Twist, '/eyantrasim/glacio/cmd_vel', 10)
        # self.pub_crystal = ...
        # self.pub_frostbite = ...

        # --- Subscribers (for bot poses) ---
        self.sub_glacio = self.create_subscription(Pose, '/eyantrasim/glacio/pose', self.pose_glacio_cb, 10)
        # self.sub_crystal = ...
        # self.sub_frostbite = ...

        # --- Storage placeholders ---
        self.targets_glacio = []
        self.targets_crystal = []
        self.targets_frostbite = []

        self.current_pose_glacio = None
        self.current_pose_crystal = None
        self.current_pose_frostbite = None

        # Example: get target coordinates
        # self.get_targets()

        # --- Timer for control loop ---
        self.timer = self.create_timer(0.1, self.control_loop)

    # --- Example method: request targets from service ---
    def get_targets(self):
        # call the ros2 service to get target coordinates
        # future = self.cli.call_async(self.req)
        pass

    # --- Pose callbacks ---
    def pose_glacio_cb(self, msg):
        self.current_pose_glacio = msg

    # --- Example controller ---
    def compute_velocity(self, current_pose, target_pose):
        vel = Twist()
        # implement P/PD controller
        return vel

    # --- Control loop for all bots ---
    def control_loop(self):
        if self.current_pose_glacio and self.targets_glacio:
            vel = self.compute_velocity(self.current_pose_glacio, self.targets_glacio[0])
            self.pub_glacio.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = BattalionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()