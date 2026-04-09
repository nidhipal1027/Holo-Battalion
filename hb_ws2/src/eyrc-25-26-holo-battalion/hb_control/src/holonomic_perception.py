#!/usr/bin/env python3

# * Team Id:          eYRC#1832
# * Author List:      Nidhi pal, Seeya kokam, Kashisa padhy
# * Filename:         perception_control.py
# * Theme:            Holo Battalion
# * Functions:        PoseDetector.__init__, PoseDetector.image_callback, PoseDetector.pixel_to_world, PoseDetector.compute_homography_matrix, PoseDetector.calculate_yaw_from_corners, PoseDetector.publish_crate_poses, PoseDetector.publish_bot_poses, main
# * Global Variables: arena_size_mm, BOT_IDS, crate_ids, corner_markers_ref_mm, CORNER_INDEX_MAP

import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from hb_interfaces.msg import Pose2D, Poses2D
from collections import deque

# Variable Name: arena_size_mm
# Description: The physical dimension of the square arena in millimeters.
arena_size_mm = 2438.4

# Variable Name: BOT_IDS, crate_ids
# Description: Lists containing the predefined ArUco marker IDs for the robots and the crates respectively.
BOT_IDS = [0, 2, 4]
crate_ids = [30, 21, 12, 14, 17, 16]

# Variable Name: corner_markers_ref_mm
# Description: Dictionary mapping the corner ArUco marker IDs to their absolute physical coordinates (x, y) in the arena.
corner_markers_ref_mm = {
    1: (0.0, 0.0),
    3: (arena_size_mm, 0.0),
    7: (arena_size_mm, arena_size_mm),
    5: (0.0, arena_size_mm) 
}

# Variable Name: CORNER_INDEX_MAP
# Description: Maps the specific corner IDs to their respective index order for calculating the homography matrix.
CORNER_INDEX_MAP = {
    1: 0,
    3: 1,
    7: 2,
    5: 3   
}

class PoseDetector(Node):
    # * Function Name: __init__
    # * Input:         None
    # * Output:        None
    # * Logic:         Initializes the ROS2 node, sets up the camera intrinsics, configures the ArUco detector parameters, and creates necessary ROS2 publishers/subscribers.
    # * Example Call:  node = PoseDetector()
    def __init__(self):
        super().__init__('localization_node')
        
        self.bridge = CvBridge()
        
        # Camera matrix and distortion coefficients for undistorting raw feed
        self.camera_matrix = np.array([[1718.91673, 0., 1105.74637],
                                       [0., 1676.79612, 302.58141],
                                       [0., 0., 1.]])
        self.dist_coeffs = np.array([-0.146350, 0.053711, 0.014333, 0.008759, 0.000000])
        self.camera_info_received = True
        
        self.crates_marker_length = 0.045
        self.bots_marker_length = 0.045
        
        self.image_sub = self.create_subscription(Image, "/image_raw", self.image_callback, 10)
        
        self.crate_poses_pub = self.create_publisher(Poses2D, '/crate_pose', 10)
        self.bot_poses_pub = self.create_publisher(Poses2D, '/bot_pose', 10)
        
        self.H_matrix = None 
        
        # Setup ArUco dictionary and detection parameters with contour refinement for better precision
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR 
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        self.pose_history = {}
        self.HISTORY_LENGTH = 10  
        
        self.get_logger().info('PoseDetector initialized')

    # * Function Name: image_callback
    # * Input:         msg -> ROS2 Image message
    # * Output:        None
    # * Logic:         Processes incoming camera frames, undistorts them, detects ArUco markers, computes homography, extracts real-world poses, and visualizes the data via OpenCV windows.
    # * Example Call:  Called automatically by the ROS2 image subscriber.
    def image_callback(self, msg):
        display_image = None
        try:
            if not self.camera_info_received:
                self.get_logger().warn("Waiting for camera info...", throttle_duration_sec=5.0)
                return
            
            # Convert ROS Image to OpenCV format and undistort
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            undistorted = cv2.undistort(image, self.camera_matrix, self.dist_coeffs)
            gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
            
            # Detect ArUco markers
            corners, ids, _ = self.detector.detectMarkers(gray)
            display_image = undistorted.copy() 

            # Handle case where no markers are detected to prevent crashes
            if ids is None:                  
                display_resized = cv2.resize(display_image, (1280, 720))
                cv2.namedWindow('Detected Markers', cv2.WINDOW_NORMAL)
                cv2.imshow('Detected Markers', display_resized)
                cv2.waitKey(1)
                return
               
            ids = ids.flatten()        
            cv2.aruco.drawDetectedMarkers(display_image, corners, ids)
            
            # Compute homography matrix using corner markers if not already done
            if self.H_matrix is None:
                self.compute_homography_matrix(corners, ids)
                if self.H_matrix is None:
                    self.get_logger().warn("Homography not computed yet.", throttle_duration_sec=5.0)
                    display_resized = cv2.resize(display_image, (1280, 720))
                    cv2.namedWindow('Detected Markers', cv2.WINDOW_NORMAL)
                    cv2.imshow('Detected Markers', display_resized)
                    cv2.waitKey(1)
                    return
            
            crate_poses = {} 
            bot_poses = {}   
            
            # Iterate over all detected markers and calculate their real-world poses
            for i, id in enumerate(ids):
                # Skip the arena corner markers as they are stationary references
                if id in corner_markers_ref_mm:
                    continue
                
                marker_corners_pixel = corners[i][0]
                world_corners = []
                valid_transform = True
                
                for pt in marker_corners_pixel:
                    wx, wy = self.pixel_to_world(pt[0], pt[1])
                    if wx is None:
                        valid_transform = False
                        break
                    world_corners.append([wx, wy])
                
                if not valid_transform:
                    continue

                world_corners = np.array(world_corners)
                
                # Calculate physical center of the marker
                center_world = np.mean(world_corners, axis=0)
                raw_x, raw_y = center_world[0], center_world[1]

                # Calculate yaw based on the top edge of the marker
                top_left = world_corners[0]
                top_right = world_corners[1]
                dx = top_right[0] - top_left[0]
                dy = top_right[1] - top_left[1]
                raw_yaw = math.degrees(math.atan2(dy, dx))
                raw_yaw = (raw_yaw + 360) % 360

                # Append to history buffer for smoothing
                if id not in self.pose_history:
                    self.pose_history[id] = deque(maxlen=self.HISTORY_LENGTH)
                self.pose_history[id].append((raw_x, raw_y, raw_yaw))
                
                # Compute moving average to reduce camera jitter
                hist = list(self.pose_history[id])
                avg_x = sum(p[0] for p in hist) / len(hist)
                avg_y = sum(p[1] for p in hist) / len(hist)
                
                # Average circular angles correctly using sin/cos components
                sin_sum = sum(math.sin(math.radians(p[2])) for p in hist)
                cos_sum = sum(math.cos(math.radians(p[2])) for p in hist)
                avg_yaw = math.degrees(math.atan2(sin_sum, cos_sum))
                avg_yaw = (avg_yaw + 360) % 360

                pose_data = (avg_x, avg_y, avg_yaw)

                # Categorize detected poses into bots and crates
                if id in BOT_IDS:
                    bot_poses[id] = pose_data
                elif id in crate_ids:
                    crate_poses[id] = pose_data

            self.publish_crate_poses(crate_poses)
            self.publish_bot_poses(bot_poses)

            # Draw orientation vectors and text on the OpenCV output frame
            TEXT_COLOR = (0, 255, 0)
            FONT = cv2.FONT_HERSHEY_SIMPLEX
            FONT_SCALE = 0.5
            THICKNESS = 1
            all_poses = list(crate_poses.items()) + list(bot_poses.items())
            
            for id, (x_mm, y_mm, yaw_deg) in all_poses:
                marker_index = np.where(ids == id)[0][0]
                center_pixel = np.mean(corners[marker_index][0], axis=0).astype(int)
                text = f"ID {id}: X:{x_mm:.1f}, Y:{y_mm:.1f}, W:{yaw_deg:.1f}"
                cv2.putText(display_image, text, (center_pixel[0], center_pixel[1] + 20),
                           FONT, FONT_SCALE, TEXT_COLOR, THICKNESS, cv2.LINE_AA)
                
                center_x, center_y = center_pixel
                angle_rad = np.radians(yaw_deg)
                arrow_len = 40 
                end_x = int(center_x + arrow_len * np.cos(angle_rad))
                end_y = int(center_y - arrow_len * np.sin(angle_rad))
                cv2.arrowedLine(display_image, (center_x, center_y), (end_x, end_y), (255, 255, 0), 2)
 
            display_resized = cv2.resize(display_image, (1280, 720))
            cv2.namedWindow('Detected Markers', cv2.WINDOW_NORMAL)
            cv2.imshow('Detected Markers', display_resized)
            cv2.waitKey(1)
            
        except Exception as e:
            if display_image is not None:
                display_resized = cv2.resize(display_image, (1280, 720))
                cv2.namedWindow('Detected Markers', cv2.WINDOW_NORMAL)
                cv2.imshow('Detected Markers', display_resized)
                cv2.waitKey(1)
            self.get_logger().error(f"Image callback error: {e}")

    # * Function Name: pixel_to_world
    # * Input:         pixel_x -> float, pixel_y -> float
    # * Output:        world_x -> float, world_y -> float
    # * Logic:         Uses the pre-computed Homography matrix to project a 2D pixel coordinate into the physical arena coordinate plane (in mm).
    # * Example Call:  wx, wy = self.pixel_to_world(500, 450)
    def pixel_to_world(self, pixel_x, pixel_y):
        if self.H_matrix is None:
            return None, None
        pixel_point = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
        world_point = cv2.perspectiveTransform(pixel_point, self.H_matrix) 
        return world_point[0][0][0], world_point[0][0][1] 

    # * Function Name: compute_homography_matrix
    # * Input:         corners -> list of numpy arrays, ids -> list of integers
    # * Output:        None
    # * Logic:         Extracts the inner pixel corners of the arena boundary markers and pairs them with their known physical distances to compute the perspective transform matrix.
    # * Example Call:  self.compute_homography_matrix(detected_corners, detected_ids)
    def compute_homography_matrix(self, corners, ids):
        pixel_pts = []
        world_pts = []
        for i, id in enumerate(ids):
            if id in CORNER_INDEX_MAP:
                corner_index = CORNER_INDEX_MAP[id]
                specific_corner_pixel = corners[i][0][corner_index]
                pixel_pts.append(specific_corner_pixel)
                world_pts.append(corner_markers_ref_mm[id])
                
        # Needs at least 4 reference points to calculate Homography accurately
        if len(pixel_pts) >= 4:
            pixel_matrix = np.array(pixel_pts, dtype=np.float32)
            world_matrix = np.array(world_pts, dtype=np.float32)
            H, _ = cv2.findHomography(pixel_matrix, world_matrix, cv2.RANSAC, 5.0)
            if H is not None:
                self.H_matrix = H
                self.get_logger().info("Homography Matrix computed successfully.")
            else:
                self.get_logger().warn("Homography computation failed.")

    # * Function Name: calculate_yaw_from_corners
    # * Input:         marker_corners_pixel -> list of pixel coordinates
    # * Output:        yaw_deg -> float
    # * Logic:         Transforms all four pixel corners of a marker to world coordinates, calculates the angle of the top edge relative to the X-axis, and returns the yaw in degrees.
    # * Example Call:  yaw = self.calculate_yaw_from_corners(corners[0])
    def calculate_yaw_from_corners(self, marker_corners_pixel):
        if self.H_matrix is None:
            return 0.0
        corners_world = []
        for corner in marker_corners_pixel:
            wx, wy = self.pixel_to_world(corner[0], corner[1])
            if wx is not None and wy is not None:
                corners_world.append([wx, wy])
        if len(corners_world) < 4:
            return 0.0
        
        top_left = np.array(corners_world[0])
        top_right = np.array(corners_world[1])
        x_axis = top_right - top_left
        yaw_rad = math.atan2(x_axis[1], x_axis[0])
        yaw_deg = math.degrees(yaw_rad)
        yaw_deg = (yaw_deg + 360) % 360
        return yaw_deg

    # * Function Name: publish_crate_poses
    # * Input:         poses -> dictionary mapping marker IDs to (x, y, yaw) tuples
    # * Output:        None
    # * Logic:         Constructs and publishes a custom Poses2D ROS message containing spatial data for all tracked crates.
    # * Example Call:  self.publish_crate_poses(crate_dict)
    def publish_crate_poses(self, poses):
        poses_msg = Poses2D()
        for id, (x_mm, y_mm, yaw_deg) in poses.items():
            pose = Pose2D()
            pose.id = int(id)
            pose.x = float(x_mm) 
            pose.y = float(y_mm) 
            pose.w = float(yaw_deg)        
            poses_msg.poses.append(pose)
        self.crate_poses_pub.publish(poses_msg)

    # * Function Name: publish_bot_poses
    # * Input:         poses -> dictionary mapping marker IDs to (x, y, yaw) tuples
    # * Output:        None
    # * Logic:         Constructs and publishes a custom Poses2D ROS message containing spatial data for all tracked robots.
    # * Example Call:  self.publish_bot_poses(bot_dict)
    def publish_bot_poses(self, poses):
        poses_msg = Poses2D()
        for id, (x_mm, y_mm, yaw_deg) in poses.items():
            pose = Pose2D()
            pose.id = int(id)
            pose.x = float(x_mm)  
            pose.y = float(y_mm)
            pose.w = float(yaw_deg)        
            poses_msg.poses.append(pose)
        self.bot_poses_pub.publish(poses_msg)

# * Function Name: main
# * Input:         args -> list of arguments passed at runtime
# * Output:        None
# * Logic:         Main execution point. Initializes rclpy, spins the PoseDetector node, and handles shutdown/cleanup of cv2 windows on exit.
# * Example Call:  Called automatically if run as a standalone script.
def main(args=None):
    rclpy.init(args=args)
    pose_detector = PoseDetector()
    try:
        rclpy.spin(pose_detector)
    except KeyboardInterrupt:
        pass
    finally:
        pose_detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()