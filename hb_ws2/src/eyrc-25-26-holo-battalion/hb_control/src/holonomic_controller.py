#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from hb_interfaces.msg import BotCmd, BotCmdArray, Poses2D
from std_msgs.msg import String
from linkattacher_msgs.srv import AttachLink, DetachLink
from rclpy.callback_groups import ReentrantCallbackGroup
import math

#configurations

BOT_CONFIG = {
    'hb_crystal': {
        'id': 0,  
        'dock': [1.220, 0.205, 0.0],  
        'dock_radius': 0.04,   
        'ir_topic': '/ir_sensor_0',
        'y_offset': 0.07,
        'angles': {'nav': (120.0, 130.0), 'grip': (50.0, 100.0), 'lift': (100.0, 100.0), 'drop': (60.0, 100.0)}
    },
    'hb_frostbite': {
        'id': 2, 
        'dock': [1.587, 0.172, 0.0], 
        'dock_radius': 0.04, 
        'ir_topic': '/ir_sensor_2',
        'y_offset': 0.14,
        'angles': {'nav': (40.0, 60.0), 'grip': (12.0, 40.0), 'lift': (50.0, 40.0), 'drop': (12.0, 40.0)}
    },
    'hb_glacio': {
        'id': 4, 
        'dock': [0.864, 0.204, 0.0],
        'dock_radius': 0.04,   
        'ir_topic': '/ir_sensor_4',
        'y_offset': 0.14,
        'angles': {'nav': (130.0, 120.0), 'grip': (110.0, 95.0), 'lift': (150.0, 90.0), 'drop': (110.0, 80.0)}
    }
}
#drop zone and slots

DROP_ZONES = {
    'red':   [1.070, 1.360, 0.895, 1.175],
    'green': [0.735, 0.905, 1.770, 1.935],
    'blue':  [1.500, 1.732, 1.950, 2.085]
}

ZONE_SLOTS = {
    'red': [   
        [1.2805, 1.0584, 0.0], #slot 0
        [1.2005, 1.0584, 0.0]  #slot 1
    ],
    'green': [
        [0.840, 1.835, 0.0], 
        [0.760, 1.835, 0.0]
    ],
    'blue':  [[1.656, 1.870, 0.0], [1.626, 1.870, 0.0]]
}

ZONE_MAP = {0: 'red', 1: 'green', 2: 'blue'}

#constants

TOLERANCE_POS = 0.01
TOLERANCE_YAW = math.radians(5)
WHEEL_RADIUS = 0.018
ROBOT_RADIUS_L = 0.08
ALPHA_1 = math.radians(30)
ALPHA_2 = math.radians(150)
ALPHA_3 = math.radians(270)
SEARCH_SPIN_SPEED = 0.6 

#swarm/avoidance constants 
AVOID_TRIGGER_DIST = 0.00 
SIDESTEP_SPEED = 0.20      

#pid class

class PID:
    def __init__(self, kp, ki, kd, max_out, integral_max, max_accel=1.5, min_out=0.15):
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

#mission states
class MissionState:
    IDLE = 0            
    NAVIGATE_TO_CRATE = 1
    GRIPPING = 3            
    LIFT_AND_STABILIZE = 4
    TRANSPORT_CRATE = 5
    PLACE_CRATE = 6
    RETURN_TO_DOCK = 7
    MISSION_COMPLETE = 8

#controller node

class MultiBotController(Node):
    def __init__(self):
        super().__init__('fleet_controller')
        self.cb_group = ReentrantCallbackGroup()

        self.cmd_pub = self.create_publisher(BotCmdArray, '/bot_cmd', 10)
        self.crate_pose_sub = self.create_subscription(Poses2D, '/crate_pose', self.crate_pose_callback, 10, callback_group=self.cb_group)
        self.bot_pose_sub = self.create_subscription(Poses2D, '/bot_pose', self.bot_pose_callback, 10, callback_group=self.cb_group)

        self.detected_crates = {}        
        self.assigned_crates = set()    
        
        self.zone_slots_counters = {'red': 0, 'green': 0, 'blue': 0}
        
        self.bots = {}
        for name, config in BOT_CONFIG.items():
            self.setup_bot(name, config)

        self.timer = self.create_timer(0.05, self.fleet_loop, callback_group=self.cb_group)
        self.get_logger().info("Swarm Fleet Controller Started (Universal Dock Stop Enabled)")

    def setup_bot(self, name, config):
        attach_srv_name = f'/{name}/attach_crate' 
        detach_srv_name = f'/{name}/detach_crate'
        
        if name == 'hb_crystal':
            pid_x = PID(1.0, 0, 0.5, 2.0, 1.0, max_accel=1.5, min_out=0.12)
            pid_y = PID(1.0, 0, 0.5, 2.0, 1.0, max_accel=1.5, min_out=0.12)
            pid_w = PID(1, 0.8, 0.6, 1.2, 1.0, max_accel=1.0, min_out=0.00)
        elif name == 'hb_frostbite':
            pid_x = PID(3.5, 0.2, 2.4, 2.0, 1.0, max_accel=1.5, min_out=0.12)
            pid_y = PID(3.5, 0.2, 2.4, 2.0, 1.0, max_accel=1.5, min_out=0.12)
            pid_w = PID(3, 0.8, 3.8, 1.2, 1.0, max_accel=1.0, min_out=0.0)
        elif name == 'hb_glacio':
            pid_x = PID(1.0, 0.0, 0.6, 2.0, 2.0, max_accel=1.5, min_out=0.12)
            pid_y = PID(1.0, 0.0, 0.6, 2.0, 2.0, max_accel=1.5, min_out=0.12)
            pid_w = PID(2.0, 0.8, 0.5, 1.2, 1.0, max_accel=1.0, min_out=0.0) 

        self.bots[name] = {
            'id': config['id'],
            'name': name,
            'state': MissionState.IDLE,
            'target_crate_id': None,
            'last_known_target': None, 
            'assigned_slot_idx': None,
            'backoff_point': None, 
            'current_destination': None, 
            'position': [0.0, 0.0, 0.0],
            'dock_pose': config['dock'],
            'angles': config['angles'],
            'y_offset': config['y_offset'], 
            'pose_initialized': False, 
            'service_busy': False,
            'ir_triggered': False,
            'attach_client': self.create_client(AttachLink, attach_srv_name),
            'detach_client': self.create_client(DetachLink, detach_srv_name),
            'x': pid_x, 'y': pid_y, 'theta': pid_w,
            'prev_time': None, 'state_entry_time': None
        }

        self.create_subscription(String, config['ir_topic'], lambda msg, n=name: self.ir_callback(msg, n), 10, callback_group=self.cb_group)

    def ir_callback(self, msg, bot_name):
        is_detected = "CRATE_DETECTED" in msg.data
        bot = self.bots[bot_name]
        bot['ir_triggered'] = is_detected

        if is_detected and bot['state'] == MissionState.NAVIGATE_TO_CRATE:
            self.get_logger().info(f" {bot_name} IR INTERRUPT")
            nav_b, nav_e = bot['angles']['nav']
            self.send_single_cmd(bot_name, 0.0, 0.0, 0.0, nav_b, nav_e)
            if not bot['service_busy']: self.call_trigger_service(bot_name, 'attach')
            bot['state'] = MissionState.GRIPPING
            bot['state_entry_time'] = self.get_clock().now()
            bot['x'].reset(); bot['y'].reset(); bot['theta'].reset()

    def crate_pose_callback(self, msg):
        self.detected_crates = {}
        for pose in msg.poses:
            self.detected_crates[pose.id] = [pose.x / 1000.0, pose.y / 1000.0, math.radians(pose.w)]

    def bot_pose_callback(self, msg):
        id_map = {cfg['id']: name for name, cfg in BOT_CONFIG.items()}
        for pose in msg.poses:
            if pose.id in id_map:
                bot_name = id_map[pose.id]
                self.bots[bot_name]['position'] = [pose.x / 1000.0, pose.y / 1000.0, math.radians(pose.w)]
                self.bots[bot_name]['pose_initialized'] = True

    def fleet_loop(self):
        self.assign_tasks()
        cmd_array = BotCmdArray()
        cmd_array.cmds = []
        curr_time = self.get_clock().now()

        for bot_name, bot in self.bots.items():
            if not bot['pose_initialized']: continue
            
            if bot['prev_time'] is None: bot['prev_time'] = curr_time
            dt = (curr_time - bot['prev_time']).nanoseconds / 1e9
            bot['prev_time'] = curr_time
            if bot['state_entry_time'] is None: bot['state_entry_time'] = curr_time

            cmd = self.run_bot_state_machine(bot_name, bot, dt, curr_time)
            if cmd: cmd_array.cmds.append(cmd)

        if cmd_array.cmds: self.cmd_pub.publish(cmd_array)

    def assign_tasks(self):
        idle_bots = [name for name, b in self.bots.items() if b['state'] == MissionState.IDLE and b['pose_initialized']]
        available_crates = [cid for cid in self.detected_crates.keys() if cid not in self.assigned_crates]
        if not idle_bots or not available_crates: return

        assignments = [] 
        for bot_name in idle_bots:
            bx, by, _ = self.bots[bot_name]['position']
            for cid in available_crates:
                cx, cy, _ = self.detected_crates[cid]
                dist = math.hypot(cx - bx, cy - by)
                assignments.append((dist, bot_name, cid))
        
        assignments.sort(key=lambda x: x[0])
        assigned_this_tick = set()
        
        for _, bot_name, cid in assignments:
            target_pose = self.detected_crates[cid]
            conflict = False
            for other_bot_name, other_bot in self.bots.items():
                if other_bot['target_crate_id'] is not None and other_bot['last_known_target']:
                    ox, oy, _ = other_bot['last_known_target']
                    tx, ty, _ = target_pose
                    if math.hypot(ox - tx, oy - ty) < 0.6: 
                        conflict = True
                        break

            if not conflict and bot_name not in assigned_this_tick and cid not in self.assigned_crates:
                self.bots[bot_name]['target_crate_id'] = cid
                self.bots[bot_name]['state'] = MissionState.NAVIGATE_TO_CRATE
                self.bots[bot_name]['state_entry_time'] = self.get_clock().now()
                self.bots[bot_name]['last_known_target'] = self.detected_crates[cid] 
                self.assigned_crates.add(cid)
                assigned_this_tick.add(bot_name)
#collision avoidance

    def compute_sidestep(self, bot_name):
        my_bot = self.bots[bot_name]
        mx, my, mw = my_bot['position']
        
        cos_w = math.cos(mw)
        sin_w = math.sin(mw)

        for other_name, other_bot in self.bots.items():
            if bot_name == other_name: continue

            ox, oy, _ = other_bot['position']
            dist = math.hypot(mx - ox, my - oy)

            if dist < AVOID_TRIGGER_DIST:
                dx_global = ox - mx
                dy_global = oy - my
                
                local_x = dx_global * cos_w + dy_global * sin_w
                local_y = -dx_global * sin_w + dy_global * cos_w

                if local_x > 0.05: 
                    if local_y > -0.05:
                        return -SIDESTEP_SPEED
                    else:
                        return SIDESTEP_SPEED
        return 0.0
#state machine
    def run_bot_state_machine(self, name, bot, dt, now):
        state = bot['state']
        nav_b, nav_e = bot['angles']['nav']
        grip_b, grip_e = bot['angles']['grip']
        lift_b, lift_e = bot['angles']['lift']
        drop_b, drop_e = bot['angles']['drop']

        cmd = BotCmd(id=bot['id'], m1=0.0, m2=0.0, m3=0.0, base=nav_b, elbow=nav_e)

        if state == MissionState.IDLE:
             bot['current_destination'] = None 
             if bot['target_crate_id'] is None:
                dx, dy, _ = bot['dock_pose']
                cx, cy, _ = bot['position']
                if math.hypot(dx - cx, dy - cy) > 0.1:
                    bot['state'] = MissionState.RETURN_TO_DOCK
        
        elif state == MissionState.NAVIGATE_TO_CRATE:
            live_target = self.detected_crates.get(bot['target_crate_id'])
            if live_target: bot['last_known_target'] = live_target 
            target = live_target if live_target else bot['last_known_target']

            if target:
                bot['current_destination'] = [target[0], target[1] - bot['y_offset']]
                reached = self.navigate_to_target(bot, target[0], target[1] - bot['y_offset'], 0.0, dt, cmd, nav_b, nav_e)
                if reached:
                    m1, m2, m3 = self.compute_kinematics(0.0, 0.0, SEARCH_SPIN_SPEED)
                    self.fill_cmd(cmd, m1, m2, m3, nav_b, nav_e)
            else:
                 bot['state'] = MissionState.IDLE

        elif state == MissionState.GRIPPING:
            bot['current_destination'] = None 
            time_in_grip = (now - bot['state_entry_time']).nanoseconds / 1e9
            move_duration = 1.5 
            if time_in_grip < move_duration:
                ratio = time_in_grip / move_duration
                curr_b = nav_b + (grip_b - nav_b) * ratio
                curr_e = nav_e + (grip_e - nav_e) * ratio
                self.fill_cmd(cmd, 0.0, 0.0, 0.0, curr_b, curr_e)
            else:
                self.fill_cmd(cmd, 0.0, 0.0, 0.0, grip_b, grip_e)
            
            if time_in_grip > 2.0:
                bot['state'] = MissionState.LIFT_AND_STABILIZE
                bot['state_entry_time'] = now
                bot['x'].reset(); bot['y'].reset(); bot['theta'].reset()

        elif state == MissionState.LIFT_AND_STABILIZE:
            bot['current_destination'] = None
            time_in_lift = (now - bot['state_entry_time']).nanoseconds / 1e9
            lift_duration = 2.0 
            if time_in_lift < lift_duration:
                ratio = time_in_lift / lift_duration
                curr_b = grip_b + (lift_b - grip_b) * ratio
                curr_e = grip_e + (lift_e - grip_e) * ratio
                self.fill_cmd(cmd, 0.0, 0.0, 0.0, curr_b, curr_e)
            else:
                self.fill_cmd(cmd, 0.0, 0.0, 0.0, lift_b, lift_e)

            if time_in_lift > (lift_duration + 0.5):
                color = ZONE_MAP.get(bot['target_crate_id'] % 3)
                if color:
                    bot['state'] = MissionState.TRANSPORT_CRATE
                    bot['x'].reset(); bot['y'].reset(); bot['theta'].reset()
                else:
                    bot['state'] = MissionState.RETURN_TO_DOCK

        elif state == MissionState.TRANSPORT_CRATE:
            color = ZONE_MAP.get(bot['target_crate_id'] % 3)
            if color:
                if bot['assigned_slot_idx'] is None:
                    if name == 'hb_frostbite' and color == 'green':
                        bot['assigned_slot_idx'] = 0
                    else:
                        bot['assigned_slot_idx'] = self.zone_slots_counters[color]
                        self.zone_slots_counters[color] += 1
                
                slot_idx = bot['assigned_slot_idx']
                available_slots = ZONE_SLOTS.get(color, [])
                if slot_idx < len(available_slots):
                    tx, ty, tw = available_slots[slot_idx]
                else:
                    tx, ty, tw = available_slots[-1]

                bot['current_destination'] = [tx, ty]
                cx, cy, _ = bot['position']
                if math.hypot(tx - cx, ty - cy) < 0.05:
                   self.fill_cmd(cmd, 0, 0, 0, lift_b, lift_e)
                   bot['state'] = MissionState.PLACE_CRATE
                   bot['state_entry_time'] = now
                else:
                    self.navigate_to_target(bot, tx, ty, tw, dt, cmd, lift_b, lift_e)
            else:
                bot['state'] = MissionState.RETURN_TO_DOCK

        elif state == MissionState.PLACE_CRATE:
            bot['current_destination'] = None
            self.fill_cmd(cmd, 0, 0, 0, drop_b, drop_e)
            time_in_state = (now - bot['state_entry_time']).nanoseconds / 1e9
            
            if time_in_state > 2.0 and time_in_state < 2.1:
                 if not bot['service_busy']: self.call_trigger_service(name, 'detach')
            
            if time_in_state > 3.0:
                bot['state'] = MissionState.IDLE
                bot['target_crate_id'] = None
                bot['assigned_slot_idx'] = None
                bot['state_entry_time'] = now

        elif state == MissionState.RETURN_TO_DOCK:
            bot['assigned_slot_idx'] = None 
            tx, ty, tw = bot['dock_pose']
            cx, cy, cw = bot['position']

            #this prevents from crashing
            dist_to_dock = math.hypot(tx - cx, ty - cy)
            
            dock_radius = BOT_CONFIG[name].get('dock_radius', 0.08)
            
            if dist_to_dock <= dock_radius:
                self.get_logger().info(f"{name} reached Safe Dock Area ({dist_to_dock:.3f}m). STOPPING.")
                self.fill_cmd(cmd, 0.0, 0.0, 0.0, nav_b, nav_e) # Stop Motors
                bot['target_crate_id'] = None
                bot['state'] = MissionState.IDLE 
                return cmd 
           

            #movement logic
            if name in ['hb_frostbite', 'hb_glacio']:
                if abs(cx - tx) > 0.05:
                    target_y = cy 
                    bot['current_destination'] = [tx, target_y]
                    self.navigate_to_target(bot, tx, target_y, tw, dt, cmd, nav_b, nav_e)

                else:
                    bot['current_destination'] = [tx, ty]
                    reached = self.navigate_to_target(bot, tx, ty, tw, dt, cmd, nav_b, nav_e)
                    if reached:
                        bot['target_crate_id'] = None
                        bot['state'] = MissionState.IDLE
            else:
                bot['current_destination'] = [tx, ty]
                reached = self.navigate_to_target(bot, tx, ty, tw, dt, cmd, nav_b, nav_e)
                if reached:
                    bot['target_crate_id'] = None
                    bot['state'] = MissionState.IDLE

        return cmd
#navigation logic
    def navigate_to_target(self, bot, tx, ty, tw, dt, cmd_obj, arm_base, arm_elbow):
        cx, cy, cw = bot['position']
        ex, ey = tx - cx, ty - cy
        ew = self.normalize_angle(tw - cw)

        if math.hypot(ex, ey) < TOLERANCE_POS and abs(ew) < TOLERANCE_YAW:
            self.fill_cmd(cmd_obj, 0, 0, 0, arm_base, arm_elbow)
            return True

        cosw, sinw = math.cos(cw), math.sin(cw)
        exr = ex * cosw + ey * sinw
        eyr = -ex * sinw + ey * cosw

        vx = bot['x'].compute(exr, dt)
        vy = bot['y'].compute(eyr, dt)
        wz = bot['theta'].compute(ew, dt)

        sidestep_vy = self.compute_sidestep(bot['name'])
        final_vy = vy + sidestep_vy

        m1, m2, m3 = self.compute_kinematics(vx, final_vy, wz)
        self.fill_cmd(cmd_obj, m1, m2, m3, arm_base, arm_elbow)
        return False

    def fill_cmd(self, cmd, m1, m2, m3, base, elbow):
        cmd.m1, cmd.m2, cmd.m3 = float(m1), float(m2), float(m3)
        cmd.base, cmd.elbow = float(base), float(elbow)

    def send_single_cmd(self, bot_name, m1, m2, m3, base, elbow):
        cmd = BotCmd(id=self.bots[bot_name]['id'], m1=float(m1), m2=float(m2), m3=float(m3), base=float(base), elbow=float(elbow))
        self.cmd_pub.publish(BotCmdArray(cmds=[cmd]))

    def call_trigger_service(self, bot_name, mode):
        bot = self.bots[bot_name]
        if bot['service_busy']: return
        bot['service_busy'] = True
        cli = bot['attach_client'] if mode == 'attach' else bot['detach_client']
        req = AttachLink.Request() if mode == 'attach' else DetachLink.Request()
        future = cli.call_async(req)
        future.add_done_callback(lambda f, n=bot_name: self.srv_cb(f, n))

    def srv_cb(self, future, bot_name):
        self.bots[bot_name]['service_busy'] = False
#wheel kinematics
    def compute_kinematics(self, vx, vy, wz):
        v1 = (-math.sin(ALPHA_1) * vx + math.cos(ALPHA_1) * vy + ROBOT_RADIUS_L * wz) / WHEEL_RADIUS
        v2 = (-math.sin(ALPHA_2) * vx + math.cos(ALPHA_2) * vy + ROBOT_RADIUS_L * wz) / WHEEL_RADIUS
        v3 = (-math.sin(ALPHA_3) * vx + math.cos(ALPHA_3) * vy + ROBOT_RADIUS_L * wz) / WHEEL_RADIUS
        return v1, v2, v3

    def normalize_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))
#main function
def main():
    rclpy.init()
    node = MultiBotController()
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