#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from hb_interfaces.msg import BotCmd, BotCmdArray, Poses2D
from std_msgs.msg import String
from linkattacher_msgs.srv import AttachLink, DetachLink
from rclpy.callback_groups import ReentrantCallbackGroup
import math
import time

#Auto-tunning PID
class AutoTunePID:
    def __init__(self, name, kp_start, ki_start, kd_start, lr=0.002, deadband=0.012):
        self.name = name
        self.lr = lr
        self.deadband = deadband
        self.prev_error = 0.0
        self.integral = 0.0
        
        self.kp = kp_start
        self.ki = ki_start
        self.kd = kd_start
            
    #Limits
        self.kp_min, self.kp_max = 0.5, 6.0
        self.kd_min, self.kd_max = 0.1, 4.0
        self.min_power = 0.05 

    def compute(self, target, current, dt):
        error = target - current
        d_error = (error - self.prev_error) / dt if dt > 0 else 0.0
        
        
        if abs(error) < self.deadband:
            self.integral = 0.0
            self.prev_error = 0.0
            return 0.0 
            
        else:
            if abs(error) < 0.05:
                self.integral += error * dt
                self.integral = max(-0.02, min(self.integral, 0.02)) 
            else:
                self.integral = 0.0
                
            output = (self.kp * error) + (self.ki * self.integral) + (self.kd * d_error)
            
            threshold = self.deadband * 2.0
            
            if abs(error) > threshold:
                if error > 0 and 0 <= output < self.min_power:
                    output = self.min_power
                elif error < 0 and 0 >= output > -self.min_power:
                    output = -self.min_power
                
            if abs(error) < 0.05:
                output = max(-0.15, min(output, 0.15))

            self.prev_error = error
            return output

class TrapezoidalProfile:
    def __init__(self, start, end, max_v, max_a):
        self.start = start
        self.end = end
        self.max_a = max_a
        self.dist = end - start
        self.direction = 1.0 if self.dist > 0 else -1.0
        self.total_dist = abs(self.dist)
        
        self.t_accel = max_v / max_a
        self.d_accel = 0.5 * max_a * (self.t_accel**2)
        
        if self.d_accel * 2 > self.total_dist:
            self.t_accel = math.sqrt(self.total_dist / max_a)
            self.d_accel = 0.5 * max_a * (self.t_accel**2)
            self.t_cruise = 0.0
            self.d_cruise = 0.0
            self.current_max_v = self.t_accel * max_a
        else:
            self.d_cruise = self.total_dist - (2 * self.d_accel)
            self.t_cruise = self.d_cruise / max_v
            self.current_max_v = max_v
            
        self.total_time = (self.t_accel * 2) + self.t_cruise
        self.start_time = None

    def calculate(self, t_now):
        if self.start_time is None: self.start_time = t_now
        t = t_now - self.start_time
        if t >= self.total_time: return self.end
        
        if t < self.t_accel: 
            pos = 0.5 * self.max_a * (t**2)
        elif t < (self.t_accel + self.t_cruise): 
            pos = self.d_accel + (self.current_max_v * (t - self.t_accel))
        else: 
            t_dec = t - self.t_accel - self.t_cruise
            pos = (self.d_accel + self.d_cruise) + (self.current_max_v * t_dec) + (0.5 * self.max_a * (t_dec**2))
            
        return self.start + (pos * self.direction)

#configuration
MAX_V, MAX_A = 0.6, 1.0

BOT_CONFIG = {
    'hb_crystal': {
        'id': 0,  
        'dock': [1.220, 0.205, 0.0],  
        'dock_radius': 0.01,   
        'ir_topic': '/ir_sensor_0',
        'y_offset': 0.12,
        'pid_xy': (2.0, 0.0, 0.8),
        'pid_theta': (2.0, 0.8, 0.5),
        'angles': {
            'nav': (100.0, 120.0), 
            'grip': (30.0, 110.0), 
            'lift': (100.0, 120.0), 
            'drop': (30.0, 110.0),
            'pyramid': (100.0, 120.0),
            'post_pyramid_nav': (100.0, 120.0) 
        }
    },
    'hb_frostbite': {
        'id': 2, 
        'dock': [1.587, 0.172, 0.0], 
        'dock_radius': 0.04, 
        'ir_topic': '/ir_sensor_2',
        'y_offset': 0.10,
        'pid_xy': (2.0, 0.0, 0.5),
        'pid_theta': (2.0, 0.8, 0.5),
        'angles': {'nav': (120.0, 50.0), 
            'grip': (35.0, 50.0), 
            'lift': (120.0, 50.0), 
            'drop': (35.0, 50.0),
            'pyramid': (100.0, 100.0),
            'post_pyramid_nav': (120.0, 50.0) 
        }
    },
    'hb_glacio': {
        'id': 4, 
        'dock': [0.864, 0.204, 0.0],
        'dock_radius': 0.04,   
        'ir_topic': '/ir_sensor_4',
        'y_offset': 0.15,
        'pid_xy': (2.2, 0.1, 0.5),
        'pid_theta': (2.5, 0.0, 0.8),
        'angles': {'nav': (180.0, 90.0), 
            'grip': (125.0, 50.0), 
            'lift': (180.0, 90.0), 
            'drop': (125.0, 50.0),
            'pyramid': (170.0, 110.0),
            'post_pyramid_nav': (180.0, 90.0) 
        }
    }
}

#Drop zones
DROP_ZONES = {
    'red':   [0.900, 1.500, 0.700, 1.400], 
    'green': [0.600, 1.100, 1.600, 2.100],
    'blue':  [1.300, 1.900, 1.800, 2.300]
}

ZONE_SLOTS = {
    'red': [   
        [1.2800, 1.060, 0.0], 
        [1.2100, 1.060, 0.0]  
    ],
    'green': [
        [0.800, 1.900, 0.0], 
        [0.760, 1.850, 0.0]
    ],
    'blue':  [[1.656, 1.870, 0.0], [1.626, 1.870, 0.0]]
}

ZONE_MAP = {0: 'red', 1: 'green', 2: 'blue'}

#constants
TOLERANCE_POS = 0.015
TOLERANCE_YAW = math.radians(1) 
WHEEL_RADIUS = 0.018
ROBOT_RADIUS_L = 0.08
ALPHA_1 = math.radians(30)
ALPHA_2 = math.radians(150)
ALPHA_3 = math.radians(270)
SEARCH_SPIN_SPEED = 0.6 

AVOID_TRIGGER_DIST = 0.50 
SIDESTEP_SPEED = 0.0      

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
        self.get_logger().info("Swarm Fleet Controller Started (Optimized Stability Active)")

    def setup_bot(self, name, config):
        attach_srv_name = f'/{name}/attach_crate' 
        detach_srv_name = f'/{name}/detach_crate'
        
        kp_xy, ki_xy, kd_xy = config.get('pid_xy', (2.0, 0.0, 0.5))
        kp_th, ki_th, kd_th = config.get('pid_theta', (2.0, 0.8, 0.5))

        pid_x = AutoTunePID(f"{name}_x", kp_xy, ki_xy, kd_xy, deadband=0.008)
        pid_y = AutoTunePID(f"{name}_y", kp_xy, ki_xy, kd_xy, deadband=0.008)
        pid_w = AutoTunePID(f"{name}_theta", kp_th, ki_th, kd_th, deadband=math.radians(0.5)) 

        self.bots[name] = {
            'id': config['id'],
            'name': name,
            'state': MissionState.IDLE,
            'target_crate_id': None,
            'last_known_target': None, 
            'assigned_slot_idx': None,
            'current_destination': None, 
            'position': [0.0, 0.0, 0.0],
            'dock_pose': config['dock'],
            'angles': config['angles'],
            'y_offset': config['y_offset'], 
            'pose_initialized': False, 
            'service_busy': False,
            'ir_triggered': False,
            'ir_debounce_count': 0,
            'required_detection_count': 5,
            'last_drop_time': -100.0,
            'pyramid_target': None, 
            'pyramid_locked': False, 
            'attach_client': self.create_client(AttachLink, attach_srv_name),
            'detach_client': self.create_client(DetachLink, detach_srv_name),
            'x': pid_x, 'y': pid_y, 'theta': pid_w, 
            'prev_time': None, 'state_entry_time': None,
            'profile_dist': None, 
            'start_pos': None
        }

        self.create_subscription(String, config['ir_topic'], lambda msg, n=name: self.ir_callback(msg, n), 10, callback_group=self.cb_group)

    def ir_callback(self, msg, bot_name):
        is_detected = "CRATE_DETECTED" in msg.data
        bot = self.bots[bot_name]
        
        current_time_sec = self.get_clock().now().nanoseconds / 1e9
        if (current_time_sec - bot['last_drop_time']) < 5.0:
            return 

        bot['ir_triggered'] = is_detected

        if bot['state'] == MissionState.NAVIGATE_TO_CRATE:
            if is_detected:
                bot['ir_debounce_count'] += 1
            else:
                bot['ir_debounce_count'] = 0
            
            if bot['ir_debounce_count'] >= bot['required_detection_count']:
                self.get_logger().info(f" {bot_name} IR INTERRUPT (CONFIRMED)")
                nav_b, nav_e = bot['angles']['nav']
                self.send_single_cmd(bot_name, 0.0, 0.0, 0.0, nav_b, nav_e)
                if not bot['service_busy']: self.call_trigger_service(bot_name, 'attach')
                bot['state'] = MissionState.GRIPPING
                bot['state_entry_time'] = self.get_clock().now()
                bot['profile_dist'] = None 
                bot['ir_debounce_count'] = 0

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
        now_sec = curr_time.nanoseconds / 1e9 

        for bot_name, bot in self.bots.items():
            if not bot['pose_initialized']: continue
            
            if bot['prev_time'] is None: bot['prev_time'] = curr_time
            dt = (curr_time - bot['prev_time']).nanoseconds / 1e9
            bot['prev_time'] = curr_time
            if bot['state_entry_time'] is None: bot['state_entry_time'] = curr_time

            cmd = self.run_bot_state_machine(bot_name, bot, dt, curr_time, now_sec)
            
            if bot_name in ['hb_crystal', 'hb_frostbite']:
                glacio = self.bots.get('hb_glacio')
                if glacio and glacio['pose_initialized']:
                    mx, my, _ = bot['position']
                    gx, gy, _ = glacio['position']
                    dist_to_glacio = math.hypot(mx - gx, my - gy)
                    
                    if dist_to_glacio < AVOID_TRIGGER_DIST:
                        cmd.m1, cmd.m2, cmd.m3 = 0.0, 0.0, 0.0

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
                    if local_y > -0.05: return -SIDESTEP_SPEED
                    else: return SIDESTEP_SPEED
        return 0.0

    def get_dynamic_pyramid_slot(self, color):
        zone_info = DROP_ZONES.get(color) 
        found_coords = []
        if zone_info:
            for cid, cpose in self.detected_crates.items():
                if (zone_info[0] <= cpose[0] <= zone_info[1]) and \
                   (zone_info[2] <= cpose[1] <= zone_info[3]):
                    found_coords.append(cpose)
        
        if len(found_coords) >= 2:
            avg_x = (found_coords[0][0] + found_coords[1][0]) / 2.0
            avg_y = (found_coords[0][1] + found_coords[1][1]) / 2.0 
            
            final_avg_x = avg_x - 0.005
            final_avg_y = avg_y - 0.03
            
            return [final_avg_x, final_avg_y, math.radians(2)]
        
        return None

    def run_bot_state_machine(self, name, bot, dt, now_obj, now_sec):
        state = bot['state']
        nav_b, nav_e = bot['angles']['nav']
        grip_b, grip_e = bot['angles']['grip']
        lift_b, lift_e = bot['angles']['lift']
        drop_b, drop_e = bot['angles']['drop']
        
        pyramid_b, pyramid_e = bot['angles'].get('pyramid', (drop_b, drop_e))
        post_pyr_b, post_pyr_e = bot['angles'].get('post_pyramid_nav', (nav_b, nav_e))

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
                reached = self.navigate_to_target(bot, target[0], target[1] - bot['y_offset'], 0.0, dt, cmd, nav_b, nav_e, now_sec)
                if reached:
                    m1, m2, m3 = self.compute_kinematics(0.0, 0.0, SEARCH_SPIN_SPEED)
                    self.fill_cmd(cmd, m1, m2, m3, nav_b, nav_e)
            else:
                 bot['state'] = MissionState.IDLE

        elif state == MissionState.GRIPPING:
            bot['current_destination'] = None 
            time_in_grip = (now_obj - bot['state_entry_time']).nanoseconds / 1e9
            move_duration = 2.0 
            if time_in_grip < move_duration:
                ratio = time_in_grip / move_duration
                curr_b = nav_b + (grip_b - nav_b) * ratio
                curr_e = nav_e + (grip_e - nav_e) * ratio
                self.fill_cmd(cmd, 0.0, 0.0, 0.0, curr_b, curr_e)
            else:
                self.fill_cmd(cmd, 0.0, 0.0, 0.0, grip_b, grip_e)
            if time_in_grip > 3.0:
                bot['state'] = MissionState.LIFT_AND_STABILIZE
                bot['state_entry_time'] = now_obj
                bot['profile_dist'] = None 

        elif state == MissionState.LIFT_AND_STABILIZE:
            bot['current_destination'] = None
            time_in_lift = (now_obj - bot['state_entry_time']).nanoseconds / 1e9
            lift_duration = 3.0 
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
                    bot['profile_dist'] = None 
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
                
                if slot_idx == 2:
                    cx_now, cy_now, cw_now = bot['position']
                    
                    if bot['pyramid_target']:
                         tx_pyr, ty_pyr, _ = bot['pyramid_target']
                         dist_to_pyr = math.hypot(tx_pyr - cx_now, ty_pyr - cy_now)
                         
                         if dist_to_pyr < 0.05:
                             bot['pyramid_locked'] = True
                    
                    if not bot['pyramid_locked']:
                        live_pose = self.get_dynamic_pyramid_slot(color)
                        if live_pose:
                            bot['pyramid_target'] = live_pose
                    
                    if bot['pyramid_target'] is not None:
                        tx, ty, tw = bot['pyramid_target']
                        
                        x_err = abs(tx - cx_now)
                        th_err = abs(self.normalize_angle(tw - cw_now))
                        
                        if x_err > 0.02:
                            bot['current_destination'] = [tx, cy_now]
                        
                        elif th_err > 0.08:
                             bot['current_destination'] = [tx, cy_now]
                             
                        else:
                            bot['current_destination'] = [tx, ty]
                    
                    else:
                         self.fill_cmd(cmd, 0, 0, 0, lift_b, lift_e)
                         return cmd 
                else:
                    tx, ty, tw = available_slots[slot_idx] if slot_idx < len(available_slots) else available_slots[-1]
                    bot['current_destination'] = [tx, ty]

                cx, cy, _ = bot['position']
                dest_x, dest_y = bot['current_destination']
                
                reached = self.navigate_to_target(bot, dest_x, dest_y, tw, dt, cmd, lift_b, lift_e, now_sec)
                
                if reached:
                   self.fill_cmd(cmd, 0, 0, 0, lift_b, lift_e)
                   bot['state'] = MissionState.PLACE_CRATE
                   bot['state_entry_time'] = now_obj
            else:
                bot['state'] = MissionState.RETURN_TO_DOCK

        elif state == MissionState.PLACE_CRATE:
            bot['current_destination'] = None
            slot_idx = bot.get('assigned_slot_idx', 0)
            
            if slot_idx == 2:
                target_b, target_e = pyramid_b, pyramid_e
            else:
                target_b, target_e = drop_b, drop_e

            self.fill_cmd(cmd, 0, 0, 0, target_b, target_e)
            time_in_state = (now_obj - bot['state_entry_time']).nanoseconds / 1e9
            
            if time_in_state > 2.0 and time_in_state < 2.1:
                 if not bot['service_busy']: self.call_trigger_service(name, 'detach')
            
            if time_in_state > 3.0:
                bot['state'] = MissionState.IDLE
                bot['target_crate_id'] = bot['assigned_slot_idx'] = None
                bot['pyramid_target'] = None 
                bot['pyramid_locked'] = False 
                bot['state_entry_time'] = now_obj
                bot['last_drop_time'] = (now_obj.nanoseconds / 1e9)
                bot['profile_dist'] = None 
                
                if slot_idx == 2:
                    cmd.base, cmd.elbow = post_pyr_b, post_pyr_e

        elif state == MissionState.RETURN_TO_DOCK:
            bot['assigned_slot_idx'] = None 
            tx, ty, tw = bot['dock_pose']
            cx, cy, cw = bot['position']
            dist_to_dock = math.hypot(tx - cx, ty - cy)
            dock_radius = BOT_CONFIG[name].get('dock_radius', 0.08)
            
            if dist_to_dock <= dock_radius:
                self.get_logger().info(f"{name} reached Safe Dock Area. STOPPING.")
                self.fill_cmd(cmd, 0.0, 0.0, 0.0, nav_b, nav_e) 
                bot['target_crate_id'] = None
                bot['state'] = MissionState.IDLE 
                return cmd 

            if name in ['hb_frostbite']:
                if abs(cx - tx) > 0.05:
                    target_y = cy 
                    bot['current_destination'] = [tx, target_y]
                    self.navigate_to_target(bot, tx, target_y, tw, dt, cmd, nav_b, nav_e, now_sec)
                else:
                    bot['current_destination'] = [tx, ty]
                    reached = self.navigate_to_target(bot, tx, ty, tw, dt, cmd, nav_b, nav_e, now_sec)
                    if reached: bot['target_crate_id'] = None; bot['state'] = MissionState.IDLE
            else:
                bot['current_destination'] = [tx, ty]
                reached = self.navigate_to_target(bot, tx, ty, tw, dt, cmd, nav_b, nav_e, now_sec)
                if reached: bot['target_crate_id'] = None; bot['state'] = MissionState.IDLE

        return cmd

    def navigate_to_target(self, bot, tx, ty, tw, dt, cmd_obj, arm_base, arm_elbow, now_sec):
        cx, cy, cw = bot['position']
        
        dx, dy = tx - cx, ty - cy
        dist_total = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        
        if bot['profile_dist'] is None or abs(bot['profile_dist'].end - dist_total) > 0.05:
            bot['profile_dist'] = TrapezoidalProfile(0.0, dist_total, MAX_V, MAX_A)
            bot['start_pos'] = [cx, cy]

        ideal_dist = bot['profile_dist'].calculate(now_sec)
        ideal_x = bot['start_pos'][0] + (ideal_dist * math.cos(angle))
        ideal_y = bot['start_pos'][1] + (ideal_dist * math.sin(angle))

        out_x = bot['x'].compute(ideal_x, cx, dt)
        out_y = bot['y'].compute(ideal_y, cy, dt)
     
        ew = self.normalize_angle(tw - cw)
        out_w = bot['theta'].compute(ew, 0.0, dt) 
        
        if dist_total > 0.1 and abs(dx) > 0.05:
             pass

        if dist_total < TOLERANCE_POS and abs(ew) < TOLERANCE_YAW:
             self.fill_cmd(cmd_obj, 0, 0, 0, arm_base, arm_elbow)
             bot['profile_dist'] = None
             return True
        
        cosw, sinw = math.cos(cw), math.sin(cw)
        vx = out_x * cosw + out_y * sinw
        vy = -out_x * sinw + out_y * cosw
        vy += self.compute_sidestep(bot['name'])
        
        m1, m2, m3 = self.compute_kinematics(vx, vy, out_w)
        self.fill_cmd(cmd_obj, m1, m2, m3, arm_base, arm_elbow)
        return False

    def fill_cmd(self, cmd, m1, m2, m3, base, elbow):
        cmd.m1, cmd.m2, cmd.m3, cmd.base, cmd.elbow = float(m1), float(m2), float(m3), float(base), float(elbow)

    def send_single_cmd(self, bot_name, m1, m2, m3, base, elbow):
        cmd = BotCmd(id=self.bots[bot_name]['id'], m1=float(m1), m2=float(m2), m3=float(m3), base=float(base), elbow=float(elbow))
        self.cmd_pub.publish(BotCmdArray(cmds=[cmd]))

    def call_trigger_service(self, bot_name, mode):
        bot = self.bots[bot_name]
        if bot['service_busy']: return
        bot['service_busy'] = True
        cli = bot['attach_client'] if mode == 'attach' else bot['detach_client']
        future = cli.call_async(AttachLink.Request() if mode == 'attach' else DetachLink.Request())
        future.add_done_callback(lambda f, n=bot_name: self.srv_cb(f, n))

    def srv_cb(self, future, bot_name): self.bots[bot_name]['service_busy'] = False

    def compute_kinematics(self, vx, vy, wz):
        v1 = (-math.sin(ALPHA_1) * vx + math.cos(ALPHA_1) * vy + ROBOT_RADIUS_L * wz) / WHEEL_RADIUS
        v2 = (-math.sin(ALPHA_2) * vx + math.cos(ALPHA_2) * vy + ROBOT_RADIUS_L * wz) / WHEEL_RADIUS
        v3 = (-math.sin(ALPHA_3) * vx + math.cos(ALPHA_3) * vy + ROBOT_RADIUS_L * wz) / WHEEL_RADIUS
        return v1, v2, v3

    def normalize_angle(self, a): return math.atan2(math.sin(a), math.cos(a))

def main():
    rclpy.init()
    node = MultiBotController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally: 
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()