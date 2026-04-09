#!/usr/bin/env python3

# * Team Id:          eYRC#1832
# * Author List:      Nidhi pal, Seeya kokam, Kashisa padhy 
# * Filename:         multi_robot_controller.py
# * Theme:            Holo Battalion
# * Functions:        AutoTunePID.__init__, AutoTunePID.compute, TrapezoidalProfile.__init__, TrapezoidalProfile.calculate, MultiBotController.__init__, MultiBotController.setup_bot, MultiBotController.ir_callback, MultiBotController.crate_pose_callback, MultiBotController.bot_pose_callback, MultiBotController.fleet_loop, MultiBotController.assign_tasks, MultiBotController.compute_sidestep, MultiBotController.get_dynamic_pyramid_slot, MultiBotController.run_bot_state_machine, MultiBotController.navigate_to_target, MultiBotController.fill_cmd, MultiBotController.send_single_cmd, MultiBotController.call_trigger_service, MultiBotController.srv_cb, MultiBotController.compute_kinematics, MultiBotController.normalize_angle, main
# * Global Variables: MAX_V, MAX_A, BOT_CONFIG, DROP_ZONES, ZONE_SLOTS, ZONE_MAP, TOLERANCE_POS, TOLERANCE_YAW, WHEEL_RADIUS, ROBOT_RADIUS_L, ALPHA_1, ALPHA_2, ALPHA_3, SEARCH_SPIN_SPEED, DETOUR_X_CORRIDOR, AVOID_TRIGGER_DIST, SIDESTEP_SPEED, MissionState

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from hb_interfaces.msg import BotCmd, BotCmdArray, Poses2D
from std_msgs.msg import String
from linkattacher_msgs.srv import AttachLink, DetachLink
from rclpy.callback_groups import ReentrantCallbackGroup
import math
import time

# Variable Name: MAX_V, MAX_A
# Description of the variable: Maximum velocity and acceleration limits for the robot's motion profiling.
MAX_V, MAX_A = 0.9, 1.0

# Variable Name: BOT_CONFIG
# Description of the variable: Dictionary containing hardware IDs, PID tunings, kinematics offsets, and joint angle limits for each robot.
BOT_CONFIG = {
    'hb_crystal': {
        'id': 0,  
        'dock': [1.220, 0.205, 0.0],  
        'dock_radius': 0.01,   
        'ir_topic': '/ir_sensor_0',
        'y_offset': 0.11,
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
        'dock_radius': 0.03, 
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
        'y_offset': 0.13,
        'pid_xy': (2.2, 0.1, 0.5), 
        'pid_theta': (2.5, 0.0, 0.8),
        'angles': {'nav': (170.0, 100.0), 
            'grip': (100.0, 50.0), 
            'lift': (170.0, 100.0), 
            'drop': (100.0, 50.0),
            'pyramid': (155.0, 100.0),
            'post_pyramid_nav': (170.0, 100.0) 
        }
    }
}

DROP_ZONES = {
    'red':   [0.900, 1.500, 0.700, 1.400], 
    'green': [0.600, 1.100, 1.600, 2.100],
    'blue':  [1.300, 1.900, 1.800, 2.300]
}

ZONE_SLOTS = {
    'red': [ [1.2800, 1.060, 0.0], [1.2100, 1.060, 0.0] ],
    'green': [ [0.800, 1.900, 0.0], [0.760, 1.850, 0.0] ],
    'blue': [ [1.6200, 1.920, 0.0], [1.5500, 1.920, 0.0] ]
}

ZONE_MAP = {0: 'red', 1: 'green', 2: 'blue'}
TOLERANCE_POS = 0.015
TOLERANCE_YAW = math.radians(1) 
WHEEL_RADIUS = 0.018
ROBOT_RADIUS_L = 0.08
ALPHA_1 = math.radians(30)
ALPHA_2 = math.radians(150)
ALPHA_3 = math.radians(270)
SEARCH_SPIN_SPEED = 0.6 
DETOUR_X_CORRIDOR = 0.45 
AVOID_TRIGGER_DIST = 0.40
SIDESTEP_SPEED = 0.0      

class MissionState:
    IDLE = 0            
    NAVIGATE_TO_CRATE = 1
    GRIPPING = 3            
    LIFT_AND_STABILIZE = 4
    TRANSPORT_CRATE = 5
    PLACE_CRATE = 6
    RETURN_TO_DOCK = 7
    MISSION_COMPLETE = 8

class AutoTunePID:
    # * Function Name: __init__
    # * Input:         name -> string, kp -> float, ki -> float, kd -> float, lr -> float, deadband -> float
    # * Output:        None
    # * Logic:         Initializes the PID controller object with provided tuning parameters and state variables.
    # * Example Call:  pid = AutoTunePID("x_axis", 2.0, 0.1, 0.5)
    def __init__(self, name, kp, ki, kd, lr=0.0, deadband=0.010):
        self.name = name
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.deadband = deadband
        self.integral = 0.0
        self.prev_current = None
        self.filtered_d = 0.0 
        self.d_alpha = 0.65 

    # * Function Name: compute
    # * Input:         target -> float, current -> float, dt -> float
    # * Output:        output -> float
    # * Logic:         Calculates the Proportional, Integral, and Derivative components to compute the control effort. Includes deadband to prevent micro-oscillations.
    # * Example Call:  effort = pid.compute(1.5, 1.0, 0.05)
    def compute(self, target, current, dt):
        if self.prev_current is None:
            self.prev_current = current

        error = target - current
        abs_error = abs(error)
        
        # Absolute stop if within deadband to prevent oscillation
        if abs_error < self.deadband:
            self.integral = 0.0
            self.prev_current = current
            return 0.0 

        P_term = self.kp * error

        # Compute Integral term with anti-windup clamping
        if abs_error < 0.15:
            self.integral += error * dt
            self.integral = max(-0.05, min(self.integral, 0.05))
        else:
            self.integral = 0.0 
        I_term = self.ki * self.integral

        # Compute Derivative term with low-pass filtering
        raw_d_meas = -(current - self.prev_current) / dt if dt > 0 else 0.0
        self.filtered_d = (self.d_alpha * raw_d_meas) + ((1.0 - self.d_alpha) * self.filtered_d)
        D_term = self.kd * self.filtered_d

        output = P_term + I_term + D_term

        # Dynamic precision limits: Scales the output smoothly when close to the target
        if abs_error < 0.08:
            output = max(-0.10, min(output, 0.10))
            dynamic_min = 0.01 + (abs_error * 0.5) 
            if abs(output) < dynamic_min:
                output = math.copysign(dynamic_min, output)
        else:
            min_travel_power = 0.05
            if abs(output) < min_travel_power:
                output = math.copysign(min_travel_power, output)
            output = max(-1.5, min(output, 1.5))

        self.prev_current = current
        return output

class TrapezoidalProfile:
    # * Function Name: __init__
    # * Input:         start -> float, end -> float, max_v -> float, max_a -> float
    # * Output:        None
    # * Logic:         Initializes a trapezoidal velocity profile for smooth acceleration, cruising, and deceleration over a set distance.
    # * Example Call:  profile = TrapezoidalProfile(0.0, 1.5, 0.5, 0.5)
    def __init__(self, start, end, max_v, max_a):
        self.start = start
        self.end = end
        self.max_a = max_a
        self.dist = end - start
        self.direction = 1.0 if self.dist > 0 else -1.0
        self.total_dist = abs(self.dist)
        
        self.t_accel = max_v / max_a
        self.d_accel = 0.5 * max_a * (self.t_accel**2)
        
        # Adjust profile if the distance is too short to reach maximum velocity
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

    # * Function Name: calculate
    # * Input:         t_now -> float
    # * Output:        pos -> float
    # * Logic:         Returns the expected position at the current time based on the pre-calculated velocity profile.
    # * Example Call:  expected_dist = profile.calculate(current_time_seconds)
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

class MultiBotController(Node):
    # * Function Name: __init__
    # * Input:         None
    # * Output:        None
    # * Logic:         Initializes the ROS2 node, sets up publishers, subscribers, dictionaries for robots, and starts the main control loop timer.
    # * Example Call:  Called automatically when instantiating the class.
    def __init__(self):
        super().__init__('multibot_controller')
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
        self.get_logger().info("Multi-bot controller started")

    # * Function Name: setup_bot
    # * Input:         name -> string, config -> dictionary
    # * Output:        None
    # * Logic:         Populates the self.bots dictionary with specific configurations, PID controllers, and service clients for a given robot.
    # * Example Call:  self.setup_bot('hb_crystal', BOT_CONFIG['hb_crystal'])
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
            'pyramid_x_settled': False,
            'ignore_x_theta': False,
            'slow_mode': False,
            'attach_client': self.create_client(AttachLink, attach_srv_name),
            'detach_client': self.create_client(DetachLink, detach_srv_name),
            'x': pid_x, 'y': pid_y, 'theta': pid_w, 
            'prev_time': None, 'state_entry_time': None,
            'profile_dist': None, 
            'start_pos': None,
            'pickup_zone': None,
            'last_logged_state': None
        }

        self.create_subscription(String, config['ir_topic'], lambda msg, n=name: self.ir_callback(msg, n), 10, callback_group=self.cb_group)

    # * Function Name: ir_callback
    # * Input:         msg -> String message, bot_name -> string
    # * Output:        None
    # * Logic:         Debounces the IR sensor input. If a crate is detected persistently during the NAVIGATE phase, it triggers the attach service and transitions to GRIPPING.
    # * Example Call:  Called automatically by ROS2 subscriber.
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
                nav_b, nav_e = bot['angles']['nav']
                self.send_single_cmd(bot_name, 0.0, 0.0, 0.0, nav_b, nav_e)
                if not bot['service_busy']: self.call_trigger_service(bot_name, 'attach')
                bot['state'] = MissionState.GRIPPING
                bot['state_entry_time'] = self.get_clock().now()
                bot['profile_dist'] = None 
                bot['ir_debounce_count'] = 0

    # * Function Name: crate_pose_callback
    # * Input:         msg -> Poses2D message
    # * Output:        None
    # * Logic:         Updates the global dictionary tracking the spatial coordinates of all detected crates.
    # * Example Call:  Called automatically by ROS2 subscriber.
    def crate_pose_callback(self, msg):
        self.detected_crates = {}
        for pose in msg.poses:
            self.detected_crates[pose.id] = [pose.x / 1000.0, pose.y / 1000.0, math.radians(pose.w)]

    # * Function Name: bot_pose_callback
    # * Input:         msg -> Poses2D message
    # * Output:        None
    # * Logic:         Updates the global dictionary tracking the position and orientation of the active bots.
    # * Example Call:  Called automatically by ROS2 subscriber.
    def bot_pose_callback(self, msg):
        id_map = {cfg['id']: name for name, cfg in BOT_CONFIG.items()}
        for pose in msg.poses:
            if pose.id in id_map:
                bot_name = id_map[pose.id]
                self.bots[bot_name]['position'] = [pose.x / 1000.0, pose.y / 1000.0, math.radians(pose.w)]
                self.bots[bot_name]['pose_initialized'] = True

    # * Function Name: fleet_loop
    # * Input:         None
    # * Output:        None
    # * Logic:         Main timer loop. Assigns tasks, calculates velocity bounds, runs collision avoidance, and aggregates motor commands to publish to the fleet.
    # * Example Call:  Called automatically by the class timer.
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
            
            # Tiered Collision Avoidance Logic: Prioritizes right-of-way among the bots based on predefined hierarchy
            mx, my, _ = bot['position']
            should_yield = False
            
            if bot['state'] != MissionState.IDLE:
                if bot_name == 'hb_glacio':
                    frostbite = self.bots.get('hb_frostbite')
                    if frostbite and frostbite['pose_initialized'] and frostbite['state'] != MissionState.IDLE:
                        fx_pos, fy_pos, _ = frostbite['position']
                        if math.hypot(mx - fx_pos, my - fy_pos) < AVOID_TRIGGER_DIST:
                            should_yield = True
                            
                elif bot_name == 'hb_crystal':
                    frostbite = self.bots.get('hb_frostbite')
                    glacio = self.bots.get('hb_glacio')
                    
                    if frostbite and frostbite['pose_initialized'] and frostbite['state'] != MissionState.IDLE:
                        fx_pos, fy_pos, _ = frostbite['position']
                        if math.hypot(mx - fx_pos, my - fy_pos) < AVOID_TRIGGER_DIST:
                            should_yield = True
                            
                    if not should_yield and glacio and glacio['pose_initialized'] and glacio['state'] != MissionState.IDLE:
                        gx, gy, _ = glacio['position']
                        if math.hypot(mx - gx, my - gy) < AVOID_TRIGGER_DIST:
                            should_yield = True

            if should_yield:
                cmd.m1, cmd.m2, cmd.m3 = 0.0, 0.0, 0.0

            if cmd: cmd_array.cmds.append(cmd)

        if cmd_array.cmds: self.cmd_pub.publish(cmd_array)

    # * Function Name: assign_tasks
    # * Input:         None
    # * Output:        None
    # * Logic:         Finds idle bots and unassigned crates, uses spatial partitioning to prevent collisions, and dynamically allocates target crates to bots.
    # * Example Call:  self.assign_tasks()
    def assign_tasks(self):
        idle_bots = [name for name, b in self.bots.items() if b['state'] in [MissionState.IDLE, MissionState.RETURN_TO_DOCK] and b['pose_initialized']]
        available_crates = [cid for cid in self.detected_crates.keys() if cid not in self.assigned_crates]
        if not idle_bots or not available_crates: return

        priority_order = ['hb_frostbite', 'hb_glacio', 'hb_crystal']
        idle_bots.sort(key=lambda b: priority_order.index(b) if b in priority_order else 99)

        for bot_name in idle_bots:
            bx, by, _ = self.bots[bot_name]['position']
            
            current_available = [cid for cid in self.detected_crates.keys() if cid not in self.assigned_crates]
            if not current_available:
                continue

            valid_options = []
            for cid in current_available:
                cx, cy, _ = self.detected_crates[cid]
                
                # Spatial partitioning constraints to restrict bot areas
                if bot_name == 'hb_glacio' and cx > 1.219: continue
                if bot_name == 'hb_frostbite' and cx < 1.219: continue

                target_pose = self.detected_crates[cid]
                conflict = False
                
                # Verifies that a target area is not actively occupied by a gripping bot
                for other_bot_name, other_bot in self.bots.items():
                    if other_bot['target_crate_id'] is not None and other_bot['state'] in [MissionState.NAVIGATE_TO_CRATE, MissionState.GRIPPING]:
                        if other_bot['last_known_target']:
                            ox, oy, _ = other_bot['last_known_target']
                            tx, ty, _ = target_pose
                            if math.hypot(ox - tx, oy - ty) < 0.35: 
                                conflict = True
                                break
                
                if not conflict:
                    dist = math.hypot(cx - bx, cy - by)
                    valid_options.append((dist, cid))
            
            if valid_options:
                if bot_name == 'hb_crystal':
                    valid_options.sort(key=lambda x: x[0], reverse=False)
                else:
                    valid_options.sort(key=lambda x: x[0], reverse=True)
                
                best_dist, best_cid = valid_options[0]

                self.bots[bot_name]['target_crate_id'] = best_cid
                self.bots[bot_name]['state'] = MissionState.NAVIGATE_TO_CRATE
                self.bots[bot_name]['state_entry_time'] = self.get_clock().now()
                self.bots[bot_name]['last_known_target'] = self.detected_crates[best_cid] 
                self.assigned_crates.add(best_cid)

    # * Function Name: compute_sidestep
    # * Input:         bot_name -> string
    # * Output:        speed -> float
    # * Logic:         Calculates lateral evasion vectors if a higher-priority bot approaches within the danger radius.
    # * Example Call:  vy += self.compute_sidestep('hb_glacio')
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

    # * Function Name: get_dynamic_pyramid_slot
    # * Input:         color -> string
    # * Output:        coordinates -> list of floats (or None)
    # * Logic:         Searches the designated drop zone and calculates the geometrical midpoint for pyramid block placement.
    # * Example Call:  live_pose = self.get_dynamic_pyramid_slot('red')
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
            return [avg_x, avg_y - 0.10, math.radians(0)]
        
        return None

    # * Function Name: run_bot_state_machine
    # * Input:         name -> string, bot -> dict, dt -> float, now_obj -> Time, now_sec -> float
    # * Output:        cmd -> BotCmd
    # * Logic:         Handles the high-level state machine transitions for navigation, lifting, placing, and returning to the dock.
    # * Example Call:  cmd = self.run_bot_state_machine("hb_crystal", my_bot_dict, 0.05, current_time, 100.5)
    def run_bot_state_machine(self, name, bot, dt, now_obj, now_sec):
        state = bot['state']

        # Log changes in state for debugging and monitoring
        if bot.get('last_logged_state') != state:
            state_names = {
                0: 'IDLE', 1: 'NAVIGATE_TO_CRATE', 3: 'GRIPPING', 
                4: 'LIFT_AND_STABILIZE', 5: 'TRANSPORT_CRATE', 
                6: 'PLACE_CRATE', 7: 'RETURN_TO_DOCK', 
                8: 'MISSION_COMPLETE'
            }
            state_str = state_names.get(state, 'UNKNOWN')
            
            if bot['target_crate_id'] is not None and state != MissionState.RETURN_TO_DOCK:
                self.get_logger().info(f"[{name}] Executing State: {state_str} | Assigned Crate: {bot['target_crate_id']}")
            else:
                self.get_logger().info(f"[{name}] Executing State: {state_str}")
                
            bot['last_logged_state'] = state

        nav_b, nav_e = bot['angles']['nav']
        grip_b, grip_e = bot['angles']['grip']
        lift_b, lift_e = bot['angles']['lift']
        drop_b, drop_e = bot['angles']['drop']
        
        pyramid_b, pyramid_e = bot['angles'].get('pyramid', (drop_b, drop_e))
        post_pyr_b, post_pyr_e = bot['angles'].get('post_pyramid_nav', (nav_b, nav_e))

        cmd = BotCmd(id=bot['id'], m1=0.0, m2=0.0, m3=0.0, base=nav_b, elbow=nav_e)
        cx, cy, cw = bot['position']
        
        # Check current zone bounds to manage corridor routing
        in_zone = None
        for z_color, coords in DROP_ZONES.items():
            if coords[0] <= cx <= coords[1] and coords[2] <= cy <= coords[3]:
                in_zone = z_color
                break
        
        authorized = False
        if bot['target_crate_id'] is not None:
            if in_zone == ZONE_MAP.get(bot['target_crate_id'] % 3):
                authorized = (state in [MissionState.TRANSPORT_CRATE])

        if in_zone and not authorized:
            bot['current_destination'] = [DETOUR_X_CORRIDOR, cy]

        # Execute operations depending on current mission phase
        if state == MissionState.IDLE:
             bot['current_destination'] = None 
             bot['pyramid_x_settled'] = False
             bot['ignore_x_theta'] = False
             bot['slow_mode'] = False
             
             if bot['target_crate_id'] is None:
                dx, dy, _ = bot['dock_pose']
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
            
            # Smoothly transition arm joints over duration
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
                    if bot['pyramid_target']:
                         tx_pyr, ty_pyr, _ = bot['pyramid_target']
                         if math.hypot(tx_pyr - cx, ty_pyr - cy) < 0.05:
                             bot['pyramid_locked'] = True
                    
                    if not bot['pyramid_locked']:
                        live_pose = self.get_dynamic_pyramid_slot(color)
                        if live_pose: bot['pyramid_target'] = live_pose
                    
                    if bot['pyramid_target'] is not None:
                        tx, ty, tw = bot['pyramid_target']
                        x_err = abs(tx - cx)
                        w_err = abs(self.normalize_angle(tw - cw))
                        
                        # Two-Phase pyramid insertion ensuring safety and precision
                        if not bot.get('pyramid_x_settled', False):
                            if x_err <= TOLERANCE_POS and w_err <= TOLERANCE_YAW:
                                bot['pyramid_x_settled'] = True
                                bot['profile_dist'] = None
                        
                        if not bot.get('pyramid_x_settled', False):
                            bot['current_destination'] = [tx, cy] 
                            bot['ignore_x_theta'] = False
                            bot['slow_mode'] = False
                        else:
                            bot['current_destination'] = [tx, ty]
                            bot['ignore_x_theta'] = True
                            bot['slow_mode'] = True
                    else:
                         self.fill_cmd(cmd, 0, 0, 0, lift_b, lift_e)
                         return cmd 
                else:
                    bot['ignore_x_theta'] = False
                    bot['slow_mode'] = False
                    tx, ty, tw = available_slots[slot_idx] if slot_idx < len(available_slots) else available_slots[-1]
                    bot['current_destination'] = [tx, ty]

                dest_x, dest_y = bot['current_destination']
                reached = self.navigate_to_target(bot, dest_x, dest_y, tw, dt, cmd, lift_b, lift_e, now_sec)
                
                if reached:
                   self.fill_cmd(cmd, 0, 0, 0, lift_b, lift_e)
                   bot['state'] = MissionState.PLACE_CRATE
                   bot['state_entry_time'] = now_obj
                   bot['pyramid_x_settled'] = False
                   bot['ignore_x_theta'] = False
                   bot['slow_mode'] = False
            else:
                bot['state'] = MissionState.RETURN_TO_DOCK

        elif state == MissionState.PLACE_CRATE:
            bot['current_destination'] = None
            slot_idx = bot.get('assigned_slot_idx', 0)
            
            target_b, target_e = (pyramid_b, pyramid_e) if slot_idx == 2 else (drop_b, drop_e)
            self.fill_cmd(cmd, 0, 0, 0, target_b, target_e)
            time_in_state = (now_obj - bot['state_entry_time']).nanoseconds / 1e9
            
            if time_in_state > 2.0 and time_in_state < 2.1:
                 if not bot['service_busy']: self.call_trigger_service(name, 'detach')
            
            if time_in_state > 3.0:
                # Mission cycle completion reset
                bot['state'] = MissionState.IDLE
                bot['target_crate_id'] = bot['assigned_slot_idx'] = None
                bot['pyramid_target'] = None 
                bot['pyramid_locked'] = False 
                bot['pickup_zone'] = None 
                bot['last_drop_time'] = now_sec
                bot['profile_dist'] = None
                bot['pyramid_x_settled'] = False
                bot['ignore_x_theta'] = False
                bot['slow_mode'] = False
                self.assign_tasks()
                
        elif state == MissionState.RETURN_TO_DOCK:
            bot['assigned_slot_idx'] = None 
            tx, ty, tw = bot['dock_pose']
            cx, cy, cw = bot['position']
            
            if name == 'hb_frostbite':
                x_err = abs(tx - cx)
                if not bot.get('pyramid_x_settled', False):
                    if x_err <= TOLERANCE_POS:
                        bot['pyramid_x_settled'] = True
                        bot['profile_dist'] = None 
                        bot['current_destination'] = [tx, ty]
                    else:
                        bot['current_destination'] = [tx, cy] 
                else:
                    bot['current_destination'] = [tx, ty]
            
            elif name == 'hb_crystal':
                y_err = abs(ty - cy)
                
                if not bot.get('pyramid_x_settled', False):
                    if y_err <= 0.30: 
                        bot['pyramid_x_settled'] = True
                        bot['profile_dist'] = None 
                        bot['current_destination'] = [tx, ty]
                    else:
                        bot['current_destination'] = [cx, ty]
                else:
                    bot['current_destination'] = [tx, ty]
                    
            else:
                bot['current_destination'] = [tx, ty]

            dest_x, dest_y = bot['current_destination']
            reached = self.navigate_to_target(bot, dest_x, dest_y, tw, dt, cmd, nav_b, nav_e, now_sec)
            
            if reached: 
                if bot.get('pyramid_x_settled', True) or (name not in ['hb_frostbite', 'hb_crystal']):
                    bot['target_crate_id'] = None
                    bot['state'] = MissionState.IDLE
                    bot['pyramid_x_settled'] = False 
                    self.fill_cmd(cmd, 0.0, 0.0, 0.0, nav_b, nav_e)

        return cmd 
            
    # * Function Name: navigate_to_target
    # * Input:         bot -> dict, tx -> float, ty -> float, tw -> float, dt -> float, cmd_obj -> BotCmd, arm_base -> float, arm_elbow -> float, now_sec -> float
    # * Output:        reached -> boolean
    # * Logic:         Computes current desired velocity profile from start to target, calculates required PID output, applies sidestep avoidance, and generates wheel kinematics.
    # * Example Call:  reached = self.navigate_to_target(my_bot, 1.5, 2.0, 0.0, 0.05, cmd, 100.0, 120.0, 150.5)
    def navigate_to_target(self, bot, tx, ty, tw, dt, cmd_obj, arm_base, arm_elbow, now_sec):
        cx, cy, cw = bot['position']
        
        dx, dy = tx - cx, ty - cy
        dist_total = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        
        if bot['profile_dist'] is None or abs(bot['profile_dist'].end - dist_total) > 0.05:
            current_max_v = MAX_V * 0.20 if bot.get('slow_mode', False) else MAX_V
            current_max_a = MAX_A * 0.20 if bot.get('slow_mode', False) else MAX_A
            bot['profile_dist'] = TrapezoidalProfile(0.0, dist_total, current_max_v, current_max_a)
            bot['start_pos'] = [cx, cy]

        ideal_dist = bot['profile_dist'].calculate(now_sec)
        ideal_x = bot['start_pos'][0] + (ideal_dist * math.cos(angle))
        ideal_y = bot['start_pos'][1] + (ideal_dist * math.sin(angle))

        out_x = bot['x'].compute(ideal_x, cx, dt)
        out_y = bot['y'].compute(ideal_y, cy, dt)
     
        ew = self.normalize_angle(tw - cw)
        out_w = bot['theta'].compute(ew, 0.0, dt) 
        
        # Override PID when restricting X and Theta axes during tight maneuvers
        if bot.get('ignore_x_theta', False):
            out_x = 0.0
            out_w = 0.0
            is_reached = (abs(dy) < TOLERANCE_POS)
        else:
            is_reached = (dist_total < TOLERANCE_POS and abs(ew) < TOLERANCE_YAW)
        
        if is_reached:
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

    # * Function Name: fill_cmd
    # * Input:         cmd -> BotCmd, m1 -> float, m2 -> float, m3 -> float, base -> float, elbow -> float
    # * Output:        None
    # * Logic:         Populates the BotCmd message object with floating-point cast parameters.
    # * Example Call:  self.fill_cmd(cmd_msg, 1.5, -1.0, 0.5, 120.0, 50.0)
    def fill_cmd(self, cmd, m1, m2, m3, base, elbow):
        cmd.m1, cmd.m2, cmd.m3, cmd.base, cmd.elbow = float(m1), float(m2), float(m3), float(base), float(elbow)

    # * Function Name: send_single_cmd
    # * Input:         bot_name -> string, m1 -> float, m2 -> float, m3 -> float, base -> float, elbow -> float
    # * Output:        None
    # * Logic:         Instantiates and directly publishes a command array containing only a single command for the targeted robot.
    # * Example Call:  self.send_single_cmd('hb_glacio', 0.0, 0.0, 0.0, 100.0, 100.0)
    def send_single_cmd(self, bot_name, m1, m2, m3, base, elbow):
        cmd = BotCmd(id=self.bots[bot_name]['id'], m1=float(m1), m2=float(m2), m3=float(m3), base=float(base), elbow=float(elbow))
        self.cmd_pub.publish(BotCmdArray(cmds=[cmd]))

    # * Function Name: call_trigger_service
    # * Input:         bot_name -> string, mode -> string ("attach" or "detach")
    # * Output:        None
    # * Logic:         Invokes the ROS2 link attacher service asynchronously to grab or release a crate without locking the main thread.
    # * Example Call:  self.call_trigger_service('hb_crystal', 'attach')
    def call_trigger_service(self, bot_name, mode):
        bot = self.bots[bot_name]
        if bot['service_busy']: return
        bot['service_busy'] = True
        cli = bot['attach_client'] if mode == 'attach' else bot['detach_client']
        future = cli.call_async(AttachLink.Request() if mode == 'attach' else DetachLink.Request())
        future.add_done_callback(lambda f, n=bot_name: self.srv_cb(f, n))

    # * Function Name: srv_cb
    # * Input:         future -> Future response, bot_name -> string
    # * Output:        None
    # * Logic:         Callback for clearing the service_busy flag once the attach/detach service finishes executing.
    # * Example Call:  Called automatically when the async service call resolves.
    def srv_cb(self, future, bot_name): self.bots[bot_name]['service_busy'] = False

    # * Function Name: compute_kinematics
    # * Input:         vx -> float, vy -> float, wz -> float
    # * Output:        v1 -> float, v2 -> float, v3 -> float
    # * Logic:         Transforms linear and angular base velocities into target rotational velocities for the three omni-wheels.
    # * Example Call:  m1, m2, m3 = self.compute_kinematics(1.0, 0.0, 0.5)
    def compute_kinematics(self, vx, vy, wz):
        v1 = (-math.sin(ALPHA_1) * vx + math.cos(ALPHA_1) * vy + ROBOT_RADIUS_L * wz) / WHEEL_RADIUS
        v2 = (-math.sin(ALPHA_2) * vx + math.cos(ALPHA_2) * vy + ROBOT_RADIUS_L * wz) / WHEEL_RADIUS
        v3 = (-math.sin(ALPHA_3) * vx + math.cos(ALPHA_3) * vy + ROBOT_RADIUS_L * wz) / WHEEL_RADIUS
        return v1, v2, v3

    # * Function Name: normalize_angle
    # * Input:         a -> float (radians)
    # * Output:        normalized_angle -> float (radians)
    # * Logic:         Normalizes angular values strictly between -pi and +pi to resolve wrapping errors in PID.
    # * Example Call:  ew = self.normalize_angle(3.5)
    def normalize_angle(self, a): return math.atan2(math.sin(a), math.cos(a))

# * Function Name: main
# * Input:         None
# * Output:        None
# * Logic:         Main entry point of the script; initializes the node, spins it with a MultiThreadedExecutor, and ensures clean shutdown.
# * Example Call:  Called by interpreter at runtime via __name__ check.
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