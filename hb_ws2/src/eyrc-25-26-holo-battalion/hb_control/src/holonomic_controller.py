#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from hb_interfaces.msg import BotCmd, BotCmdArray, Pose2D, Poses2D 
from linkattacher_msgs.srv import AttachLink, DetachLink
from rclpy.callback_groups import ReentrantCallbackGroup
import json
import math

#Constants
ROBOT_ID = 0
ROBOT_MODEL_NAME = "hb_crystal"
ARM_LINK_NAME = "arm_link_2"
CRATE_MODEL_NAME = "crate_green_43" 
CRATE_LINK_NAME = "box_link_43"

D1_ZONE = [1.020, 1.410, 1.075, 1.355]

TOLERANCE_POS = 0.020     #meters
TOLERANCE_YAW = math.radians(8)  #radians

MAX_LIN_VEL = 3.0         
MAX_ANG_VEL = 1.5         
MAX_INTEGRAL = 1.0        

WHEEL_RADIUS = 0.05       #meters
ROBOT_RADIUS_L = 0.185    #meters
MIN_ROBOT_TO_BOX_DISTANCE = 0.130 #meters

DOCK_POSE = [1.218, 0.205, math.radians(0)] #meters, meters, radians

ALPHA_1 = math.radians(30)  
ALPHA_2 = math.radians(150) 
ALPHA_3 = math.radians(270) 

class PID:
    def __init__(self, kp, ki, kd, max_out, integral_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral_max = integral_max
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        if dt <= 0: return 0.0
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_max), -self.integral_max)
        derivative = (error - self.prev_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        return max(min(output, self.max_out), -self.max_out)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

class MissionState:
    DETECT_CRATE = 0
    NAVIGATE_TO_CRATE = 1
    APPROACH_AND_GRIP = 2
    WAITING_FOR_ATTACHMENT = 3
    LIFT_AND_STABILIZE = 4
    TRANSPORT_CRATE = 5
    PLACE_CRATE = 6
    WAITING_FOR_DETACHMENT = 7
    NAVIGATE_TO_DOCK = 8
    MISSION_COMPLETE = 9

class PickAndPlaceRobot(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        self.cb_group = ReentrantCallbackGroup()
    
        self.pid_x = PID(kp=49, ki=1, kd=12.5, max_out=MAX_LIN_VEL, integral_max=MAX_INTEGRAL)
        self.pid_y = PID(kp=49, ki=1, kd=12.5, max_out=MAX_LIN_VEL, integral_max=MAX_INTEGRAL)
        self.pid_theta = PID(kp=0, ki=0, kd=00, max_out=MAX_ANG_VEL, integral_max=MAX_INTEGRAL)
        
        self.cmd_pub = self.create_publisher(BotCmdArray, '/bot_cmd', 10)
        self.crate_pose_sub = self.create_subscription(
            Poses2D, '/crate_pose', self.crate_pose_callback, 10, callback_group=self.cb_group
        )
        self.bot_pose_sub = self.create_subscription(
            Poses2D, '/bot_pose', self.bot_pose_callback, 10, callback_group=self.cb_group
        )
        self.attach_cli = self.create_client(AttachLink, '/attach_link', callback_group=self.cb_group)
        self.detach_cli = self.create_client(DetachLink, '/detach_link', callback_group=self.cb_group)
        
        self.current_state = MissionState.DETECT_CRATE
        self.crate_position = None
        self.robot_position = DOCK_POSE.copy()
        self.target_position = None
        self.attach_future = None
        self.detach_future = None
        self.crate_attached = False
        
        self.prev_time = self.get_clock().now()
        self.state_entry_time = self.get_clock().now()
        
        self.timer = self.create_timer(0.05, self.mission_loop, callback_group=self.cb_group)
        self.get_logger().info('Pick and Place Node Initialized!')

    def crate_pose_callback(self, msg: Poses2D):
        if msg.poses:
            pose: Pose2D = msg.poses[0]
            self.crate_position = [pose.x / 1000.0, pose.y / 1000.0, math.radians(pose.w)]

    def bot_pose_callback(self, msg: Poses2D):
        if msg.poses:
            pose: Pose2D = msg.poses[0]
            self.robot_position = [pose.x / 1000.0, pose.y / 1000.0, math.radians(pose.w)]

    def send_bot_command(self, m1=0.0, m2=0.0, m3=0.0, base=0.0, elbow=0.0):
        cmd = BotCmd(id=ROBOT_ID, m1=float(m1), m2=float(m2), m3=float(m3), base=float(base), elbow=float(elbow))
        msg = BotCmdArray(cmds=[cmd])
        self.cmd_pub.publish(msg)

    def attach_crate_async(self):
        if not self.attach_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(' Attach service not available.')
            return None
        data_dict = {
            "model1_name": ROBOT_MODEL_NAME,
            "link1_name": ARM_LINK_NAME,
            "model2_name": CRATE_MODEL_NAME,
            "link2_name": CRATE_LINK_NAME
        }
        request = AttachLink.Request(data=json.dumps(data_dict))
        self.get_logger().info(' Requesting crate attachment...')
        return self.attach_cli.call_async(request)

    def detach_crate_async(self):
        if not self.detach_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Detach service not available.')
            return None
        data_dict = {
            "model1_name": ROBOT_MODEL_NAME,
            "link1_name": ARM_LINK_NAME,
            "model2_name": CRATE_MODEL_NAME,
            "link2_name": CRATE_LINK_NAME
        }
        request = DetachLink.Request(data=json.dumps(data_dict))
        self.get_logger().info(' Requesting crate detachment...')
        return self.detach_cli.call_async(request)

    def set_arm_position(self, base_angle, elbow_angle):
        self.send_bot_command(m1=0.0, m2=0.0, m3=0.0, base=base_angle, elbow=elbow_angle)

    def compute_wheel_velocities(self, vx, vy, omega):
        r = WHEEL_RADIUS
        L = ROBOT_RADIUS_L
        
        v1 = -vx * math.sin(ALPHA_1) + vy * math.cos(ALPHA_1) + L * omega
        v2 = -vx * math.sin(ALPHA_2) + vy * math.cos(ALPHA_2) + L * omega
        v3 = -vx * math.sin(ALPHA_3) + vy * math.cos(ALPHA_3) + L * omega
        
        w1 = v1 / r
        w2 = v2 / r
        w3 = v3 / r
        return w1, w2, w3

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def reset_pid(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_theta.reset()
        self.prev_time = self.get_clock().now()

    def navigate_to_target(self, tx, ty, tw, keep_arm_up=True):
        current_x, current_y, current_w = self.robot_position
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time
        
        if dt < 1e-6:
            return False
        
        error_x_world = tx - current_x
        error_y_world = ty - current_y
        distance_error = math.sqrt(error_x_world**2 + error_y_world**2)
        error_theta_world = self.normalize_angle(tw - current_w)
        
        if distance_error < TOLERANCE_POS and abs(error_theta_world) < TOLERANCE_YAW:
            self.get_logger().info(f'Target reached! Distance: {distance_error*1000:.1f}mm, Yaw error: {math.degrees(error_theta_world):.1f}°')
            self.send_bot_command(m1=0.0, m2=0.0, m3=0.0) 
            self.reset_pid()
            return True
        
        cos_w = math.cos(current_w)
        sin_w = math.sin(current_w)
        error_x_robot = error_x_world * cos_w + error_y_world * sin_w
        error_y_robot = -error_x_world * sin_w + error_y_world * cos_w
        
        vx_robot = self.pid_x.compute(error_x_robot, dt)
        vy_robot = self.pid_y.compute(error_y_robot, dt)
        wz_robot = self.pid_theta.compute(error_theta_world, dt)
        
        m1, m2, m3 = self.compute_wheel_velocities(vx_robot, vy_robot, wz_robot)
        
        base_angle = 0.0
        elbow_angle = 0.0
        if keep_arm_up:
            if self.crate_attached:
                base_angle, elbow_angle = 60.0, 60.0 
            else:
                base_angle, elbow_angle = 45.0, 45.0 
        
        self.send_bot_command(m1=m1, m2=m2, m3=m3, base=base_angle, elbow=elbow_angle)
        return False

    def mission_loop(self):
        
        if self.current_state == MissionState.DETECT_CRATE:
            if self.crate_position is not None:
                cx, cy, cyaw = self.crate_position
    
                approach_offset = 0.15 
                approach_x = cx
                approach_y = cy - approach_offset
                approach_yaw = math.radians(0)
                
                self.target_position = [approach_x, approach_y, approach_yaw]
                self.reset_pid()
                self.get_logger().info(f'Navigating to approach position: X:{approach_x*1000:.1f}mm, Y:{approach_y*1000:.1f}mm')
                self.current_state = MissionState.NAVIGATE_TO_CRATE
            else:
                self.send_bot_command(m1=0.0, m2=0.0, m3=0.0, base=0.0, elbow=0.0)

        elif self.current_state == MissionState.NAVIGATE_TO_CRATE:
            tx, ty, tw = self.target_position
            if self.navigate_to_target(tx, ty, tw, keep_arm_up=True):
                self.current_state = MissionState.APPROACH_AND_GRIP

        elif self.current_state == MissionState.APPROACH_AND_GRIP:
            self.send_bot_command(m1=0.0, m2=0.0, m3=0.0, base=91.0, elbow=90.0)
            
            self.attach_future = self.attach_crate_async()
            
            if self.attach_future is not None:
                self.state_entry_time = self.get_clock().now()
                self.current_state = MissionState.WAITING_FOR_ATTACHMENT
            else:
                self.get_logger().error("Attachment setup failed, retrying.")
                self.current_state = MissionState.DETECT_CRATE

        elif self.current_state == MissionState.WAITING_FOR_ATTACHMENT:
            self.send_bot_command(m1=0.0, m2=0.0, m3=0.0, base=92.0, elbow=90.0) 
        
            stabilization_dt = (self.get_clock().now() - self.state_entry_time).nanoseconds / 1e9
            
            if stabilization_dt < 1.0: 
                return 

            if self.attach_future.done():
                result = self.attach_future.result()
                if result and result.success:
                    self.get_logger().info("Crate attached!")
                    self.crate_attached = True
                    self.current_state = MissionState.LIFT_AND_STABILIZE
                    self.state_entry_time = self.get_clock().now()
                else:
                    self.get_logger().error("Attachment failed, retrying.")
                    self.current_state = MissionState.DETECT_CRATE

        elif self.current_state == MissionState.LIFT_AND_STABILIZE:
            self.set_arm_position(base_angle=60.0, elbow_angle=60.0)
            stabilization_dt = (self.get_clock().now() - self.state_entry_time).nanoseconds / 1e9
            
            if stabilization_dt > 2.0: 
                self.get_logger().info("Crate lifted & stabilized. Begin transport.")
                self.reset_pid()
                self.current_state = MissionState.TRANSPORT_CRATE

        elif self.current_state == MissionState.TRANSPORT_CRATE:
            target_x = (D1_ZONE[0] + D1_ZONE[1]) / 2.0
            target_y = (D1_ZONE[2] + D1_ZONE[3]) / 2.0
            target_yaw = math.radians(0) 
            self.target_position = [target_x, target_y, target_yaw]
            
            if self.navigate_to_target(target_x, target_y, target_yaw, keep_arm_up=True):
                self.get_logger().info("Reached D1 zone. Placing crate...")
                self.state_entry_time = self.get_clock().now()
                self.current_state = MissionState.PLACE_CRATE

        elif self.current_state == MissionState.PLACE_CRATE:
            self.set_arm_position(base_angle=92.0, elbow_angle=90.0)
            
            placement_delay = (self.get_clock().now() - self.state_entry_time).nanoseconds / 1e9

            if placement_delay > 1.0: 
                self.set_arm_position(base_angle=0.0, elbow_angle=0.0)
                
                self.detach_future = self.detach_crate_async()
                if self.detach_future is not None:
                    self.current_state = MissionState.WAITING_FOR_DETACHMENT
                else:
                    self.get_logger().error("Detachment failed immediately.")
                    self.current_state = MissionState.NAVIGATE_TO_DOCK

        elif self.current_state == MissionState.WAITING_FOR_DETACHMENT:
            self.set_arm_position(base_angle=90.0, elbow_angle=90.0) 
            
            if self.detach_future.done():
                result = self.detach_future.result()
                if result and result.success:
                    self.get_logger().info("Crate detached successfully!")
                    self.crate_attached = False
                else:
                    self.get_logger().error("Detachment failed.")
                
                self.set_arm_position(base_angle=45.0, elbow_angle=45.0)
                self.reset_pid()
                self.current_state = MissionState.NAVIGATE_TO_DOCK
        
        elif self.current_state == MissionState.NAVIGATE_TO_DOCK:
            tx, ty, tw = DOCK_POSE
            if self.navigate_to_target(tx, ty, tw, keep_arm_up=False): 
                self.current_state = MissionState.MISSION_COMPLETE

        elif self.current_state == MissionState.MISSION_COMPLETE:
            self.get_logger().info("MISSION COMPLETE! ")
            self.send_bot_command(m1=0.0, m2=0.0, m3=0.0, base=45.0, elbow=45.0)
            self.timer.cancel() 

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    robot_node = PickAndPlaceRobot()
    executor.add_node(robot_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        robot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()