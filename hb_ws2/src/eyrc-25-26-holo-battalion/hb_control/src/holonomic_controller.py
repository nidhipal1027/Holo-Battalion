#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from hb_interfaces.msg import BotCmd, BotCmdArray, Poses2D
from std_msgs.msg import String
from linkattacher_msgs.srv import AttachLink, DetachLink
from rclpy.callback_groups import ReentrantCallbackGroup
import math

#constants
BOT_NAMES = ['hb_crystal']        #robot information
BOT_IDS = {'hb_crystal': 0}

DOCK_POSES = {                              #dock position
    'hb_crystal': [1.218, 0.190, 0.0]
}

#zone boundaries
DROP_ZONES = {
    'red':   [1.070, 1.360, 0.895, 1.175],
    'green': [0.705, 0.935, 1.950, 2.085],
    'blue':  [1.500, 1.732, 1.950, 2.085]
}

#zone centers 
ZONE_CENTERS = {
    'red':   [1.215, 1.035], #average of red bounds
    'green': [0.820, 2.017], #average of green bounds
    'blue':  [1.616, 2.017]  #average of blue bounds
}

ZONE_MAP = {0: 'red', 1: 'green', 2: 'blue'}

TOLERANCE_POS = 0.05
TOLERANCE_YAW = math.radians(8)

WHEEL_RADIUS = 0.018
ROBOT_RADIUS_L = 0.08
#robot kinematics (omni wheel)
ALPHA_1 = math.radians(30)
ALPHA_2 = math.radians(150)
ALPHA_3 = math.radians(270) 
#pid controller
class PID:
    def __init__(self, kp, ki, kd, max_out, integral_max, max_accel=2.0, min_out=0.15):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_out = max_out
        self.integral_max = integral_max
        self.max_accel = max_accel
        self.min_out = min_out
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_output = 0.0

    def compute(self, error, dt):
        if dt <= 0: return self.prev_output
        self.integral = max(min(self.integral + error * dt, self.integral_max), -self.integral_max)
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        target_out = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        if abs(error) < 0.005: 
            target_out = 0.0
        elif 0 < abs(target_out) < self.min_out:
            target_out = self.min_out if target_out > 0 else -self.min_out
        diff = target_out - self.prev_output
        max_change = self.max_accel * dt
        clamped_diff = max(min(diff, max_change), -max_change)
        current_out = self.prev_output + clamped_diff
        current_out = max(min(current_out, self.max_out), -self.max_out)
        self.prev_output = current_out
        return current_out

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_output = 0.0
#mission state machine
class MissionState:
    DETECT_CRATE = 0
    NAVIGATE_TO_CRATE = 1
    APPROACH_AND_GRIP = 2
    GRIPPING = 3          
    LIFT_AND_STABILIZE = 4
    TRANSPORT_CRATE = 5
    PLACE_CRATE = 6
    RETURN_TO_DOCK = 7
    MISSION_COMPLETE = 8
#ros2 main node
class SingleBotController(Node):
    def __init__(self):
        super().__init__('crystal_controller')
        self.cb_group = ReentrantCallbackGroup()

        self.cmd_pub = self.create_publisher(BotCmdArray, '/bot_cmd', 10)
        self.crate_pose_sub = self.create_subscription(Poses2D, '/crate_pose', self.crate_pose_callback, 10, callback_group=self.cb_group)
        self.bot_pose_sub = self.create_subscription(Poses2D, '/bot_pose', self.bot_pose_callback, 10, callback_group=self.cb_group)
        self.ir_sub = self.create_subscription(String, '/ir_sensor_0', self.ir_callback, 10, callback_group=self.cb_group)

        self.attach_client = self.create_client(AttachLink, 'attach_crate')
        self.detach_client = self.create_client(DetachLink, 'detach_crate')

        self.crate_physically_detected = False
        self.detected_crates = {}
        self.target_crate_id = None
        self.bot_pose_initialized = False
        self.service_busy = False

        self.bot_data = {
            'hb_crystal': {
                'state': MissionState.DETECT_CRATE,
                'x': PID(1.8, 0.7, 1.5, 2.0, 1.0, max_accel=1.5, min_out=0.12),
                'y': PID(1.8, 0.7, 1.5, 2.0, 1.0, max_accel=1.5, min_out=0.12),
                'theta': PID(0.1, 0.0, 2.0, 1.2, 1.0, max_accel=1.0, min_out=0.08),
                'prev_time': None,
                'state_entry_time': None,
                'position': [0.0, 0.0, 0.0]
            }
        }

        self.timer = self.create_timer(0.05, self.mission_loop, callback_group=self.cb_group)
        self.get_logger().info("Crystal Controller Started")

    def ir_callback(self, msg):
        is_detected = "CRATE_DETECTED" in msg.data
        self.crate_physically_detected = is_detected

        bot = self.bot_data['hb_crystal']
        current_state = bot['state']

        
        if is_detected and (current_state == MissionState.NAVIGATE_TO_CRATE or current_state == MissionState.APPROACH_AND_GRIP):
            self.get_logger().info("⚡ IR INTERRUPT: STOPPING & GRIPPING")
            self.send_bot_command(0.0, 0.0, 0.0, 50.0, 100.0)
            
            if not self.service_busy:
                self.call_trigger_service('attach')
            
            bot['state'] = MissionState.GRIPPING
            bot['state_entry_time'] = self.get_clock().now()
            bot['x'].reset(); bot['y'].reset(); bot['theta'].reset()

    def crate_pose_callback(self, msg):
        for pose in msg.poses:
            self.detected_crates[pose.id] = [pose.x / 1000.0, pose.y / 1000.0, math.radians(pose.w)]
        if self.target_crate_id is None and self.detected_crates:
            self.target_crate_id = min(self.detected_crates.keys())

    def bot_pose_callback(self, msg):
        for pose in msg.poses:
            if pose.id == 0:
                self.bot_data['hb_crystal']['position'] = [pose.x / 1000.0, pose.y / 1000.0, math.radians(pose.w)]
                self.bot_pose_initialized = True

    def mission_loop(self):
        if not self.bot_pose_initialized: return
        
        bot = self.bot_data['hb_crystal']
        curr_time = self.get_clock().now()
        if bot['state_entry_time'] is None: bot['state_entry_time'] = curr_time

        now = self.get_clock().now()
        dt = (now - (bot['prev_time'] or now)).nanoseconds / 1e9
        bot['prev_time'] = now

        state = bot['state']

        if state == MissionState.DETECT_CRATE:
            if self.target_crate_id is not None:
                self.get_logger().info("STATE COMPLETED: DETECT_CRATE")
                bot['state'] = MissionState.NAVIGATE_TO_CRATE
        
        elif state == MissionState.NAVIGATE_TO_CRATE:
            
            target = self.detected_crates.get(self.target_crate_id)
            if target and self.navigate_to_target(bot, target[0], target[1] - 0.25, 0.0, dt):
                self.send_bot_command(0, 0, 0, 120.0, 130.0)
                bot['state'] = MissionState.APPROACH_AND_GRIP
                bot['state_entry_time'] = curr_time

        elif state == MissionState.APPROACH_AND_GRIP:
            target = self.detected_crates.get(self.target_crate_id)
            if target:
                cx, cy, cw = bot['position']
                tx, ty, tw = target
                ex, ey = tx - cx, ty - cy
                ew = self.normalize_angle(tw - cw)
                cosw, sinw = math.cos(cw), math.sin(cw)
                exr = ex * cosw + ey * sinw
                eyr = -ex * sinw + ey * cosw
                
                vx = 0.08 if abs(eyr) < 0.02 else 0.0 
                vy = bot['y'].compute(eyr, dt)
                wz = bot['theta'].compute(ew, dt)
                m1, m2, m3 = self.compute_kinematics(vx, vy, wz)
                self.send_bot_command(m1, m2, m3, 120.0, 130.0)

        elif state == MissionState.GRIPPING:
            
            self.send_bot_command(0.0, 0.0, 0.0, 50.0, 100.0)
            time_in_grip = (curr_time - bot['state_entry_time']).nanoseconds / 1e9
            
            if time_in_grip > 2.0:
                self.get_logger().info("Grip Confirmed. Lifting...")
                bot['state'] = MissionState.LIFT_AND_STABILIZE
                bot['state_entry_time'] = curr_time
                
                bot['x'].reset(); bot['y'].reset(); bot['theta'].reset()

        elif state == MissionState.LIFT_AND_STABILIZE:
            self.send_bot_command(0, 0, 0, 100.0, 100.0)
            if (curr_time - bot['state_entry_time']).nanoseconds / 1e9 > 2.5:
                bot['state'] = MissionState.TRANSPORT_CRATE
                
                bot['x'].reset(); bot['y'].reset(); bot['theta'].reset()

        elif state == MissionState.TRANSPORT_CRATE:
            color = ZONE_MAP.get(self.target_crate_id % 3)
            if color:
                target_center = ZONE_CENTERS[color]
                drop_zone = DROP_ZONES[color]
                cx, cy, _ = bot['position']

        
                if self.is_inside_zone(cx, cy, drop_zone):
                   self.get_logger().info(f"ENTERED {color.upper()} ZONE - STOPPING")
                   self.send_bot_command(0, 0, 0, 100.0, 100.0) # Stop
                   bot['state'] = MissionState.PLACE_CRATE
                   bot['state_entry_time'] = curr_time
                else:
                    
                    
                    self.navigate_to_target(bot, target_center[0], target_center[1], 0.0, dt, arm_base=100.0, arm_elbow=100.0)

            else:
                bot['state'] = MissionState.RETURN_TO_DOCK

        elif state == MissionState.PLACE_CRATE:
            self.send_bot_command(0, 0, 0, 60.0, 100.0) # Lower arm
            time_in_state = (curr_time - bot['state_entry_time']).nanoseconds / 1e9
            
            if time_in_state < 2.0: return 
            
            self.call_trigger_service('detach')
            
            if time_in_state > 3.0:
                self.target_crate_id = None
                bot['state'] = MissionState.RETURN_TO_DOCK
                bot['state_entry_time'] = curr_time
                bot['x'].reset(); bot['y'].reset(); bot['theta'].reset()

        elif state == MissionState.RETURN_TO_DOCK:
            tx, ty, tw = DOCK_POSES['hb_crystal']
            if self.navigate_to_target(bot, tx, ty, tw, dt):
                bot['state'] = MissionState.MISSION_COMPLETE
                self.get_logger().info("Mission accomplished.")

    def navigate_to_target(self, bot, tx, ty, tw, dt, arm_base=120.0, arm_elbow=130.0):
        cx, cy, cw = bot['position']
        ex, ey = tx - cx, ty - cy
        ew = self.normalize_angle(tw - cw)

        if math.hypot(ex, ey) < TOLERANCE_POS and abs(ew) < TOLERANCE_YAW:
            self.send_bot_command(0, 0, 0, arm_base, arm_elbow)
            return True

        cosw, sinw = math.cos(cw), math.sin(cw)
        exr = ex * cosw + ey * sinw
        eyr = -ex * sinw + ey * cosw

        vx = bot['x'].compute(exr, dt)
        vy = bot['y'].compute(eyr, dt)
        wz = bot['theta'].compute(ew, dt)

        m1, m2, m3 = self.compute_kinematics(vx, vy, wz)
        self.send_bot_command(m1, m2, m3, arm_base, arm_elbow)
        return False
    
    def is_inside_zone(self, x, y, drop_zone):
        x_min, x_max, y_min, y_max = drop_zone
        return (x_min <= x <= x_max) and (y_min <= y <= y_max)

    def compute_kinematics(self, vx, vy, wz):
        v1 = (-math.sin(ALPHA_1) * vx + math.cos(ALPHA_1) * vy + ROBOT_RADIUS_L * wz) / WHEEL_RADIUS
        v2 = (-math.sin(ALPHA_2) * vx + math.cos(ALPHA_2) * vy + ROBOT_RADIUS_L * wz) / WHEEL_RADIUS
        v3 = (-math.sin(ALPHA_3) * vx + math.cos(ALPHA_3) * vy + ROBOT_RADIUS_L * wz) / WHEEL_RADIUS
        return v1, v2, v3

    def normalize_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def send_bot_command(self, m1, m2, m3, base, elbow):
        cmd = BotCmd(id=0, m1=float(m1), m2=float(m2), m3=float(m3), base=float(base), elbow=float(elbow))
        self.cmd_pub.publish(BotCmdArray(cmds=[cmd]))

    def call_trigger_service(self, mode):
        if self.service_busy: return
        self.service_busy = True
        if mode == 'attach':
            self.attach_client.call_async(AttachLink.Request()).add_done_callback(self.srv_cb)
        else:
            self.detach_client.call_async(DetachLink.Request()).add_done_callback(self.srv_cb)

    def srv_cb(self, future):
        self.service_busy = False
        self.get_logger().info("Service call completed.")

def main():
    rclpy.init()
    node = SingleBotController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()