#!/usr/bin/env python3
"""
Improved Pose Detection Node with Enhanced Accuracy
This node detects ArUco markers and publishes accurate 2D poses with sub-pixel accuracy.
"""
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from hb_interfaces.msg import Pose2D, Poses2D

# Constants
arena_size_mm = 2438.4
bot_id = 9
crate_ids = range(10, 50)

# Corner markers reference positions (world coordinates in mm)
corner_markers_ref_mm = {
    1: (0.0, 0.0),                      # Top-Left
    3: (arena_size_mm, 0.0),            # Top-Right
    7: (arena_size_mm, arena_size_mm),  # Bottom-Right
    5: (0.0, arena_size_mm)             # Bottom-Left 
}

# Dictionary to map Marker ID to the required Corner Index (0=TL, 1=TR, 2=BR, 3=BL)
CORNER_INDEX_MAP = {
    1: 0,  # Use top-left corner of marker 1
    3: 1,  # Use top-right corner of marker 3
    7: 2,  # Use bottom-right corner of marker 7
    5: 3   # Use bottom-left corner of marker 5
}

class PoseDetector(Node):
    def __init__(self):
        super().__init__('localization_node')
        
        # CvBridge Initialized
        self.bridge = CvBridge()
        
        # Parameters
        self.crates_marker_length = 0.05  # Set marker size in meters
        self.bots_marker_length = 0.05    # Set bot marker size in meters
        
        # Subscription 
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)     
        self.camera_info_sub = self.create_subscription(CameraInfo, "/camera/camera_info", self.camera_info_callback, 10)
        
        # Publisher 
        self.crate_poses_pub = self.create_publisher(Poses2D, '/crate_pose', 10)
        self.bot_poses_pub = self.create_publisher(Poses2D, '/bot_pose', 10)
        
        # Camera & Homography State 
        self.camera_info_received = False
        self.camera_matrix = None  
        self.dist_coeffs = None   
        self.H_matrix = None 
        
        # ArUco detection setup with enhanced accuracy parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # ========== ACCURACY IMPROVEMENTS ==========
        # Use subpixel corner refinement for better accuracy (sub-pixel precision)
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize = 5  # Window size for corner refinement
        self.aruco_params.cornerRefinementMaxIterations = 30  # Max iterations for convergence
        self.aruco_params.cornerRefinementMinAccuracy = 0.01  # Accuracy threshold for stopping
        
        # Additional detection parameters for improved accuracy
        self.aruco_params.adaptiveThreshWinSizeMin = 3  # Minimum window size for adaptive thresholding
        self.aruco_params.adaptiveThreshWinSizeMax = 23  # Maximum window size
        self.aruco_params.adaptiveThreshWinSizeStep = 10  # Window size step
        self.aruco_params.adaptiveThreshConstant = 7  # Constant subtracted from mean for thresholding
        
        # Detector accuracy parameters
        self.aruco_params.minMarkerPerimeterRate = 0.03  # Minimum marker perimeter rate
        self.aruco_params.maxMarkerPerimeterRate = 4.0  # Maximum marker perimeter rate
        self.aruco_params.polygonalApproxAccuracyRate = 0.03  # Accuracy for polygon approximation
        self.aruco_params.minCornerDistanceRate = 0.05  # Minimum distance between corners
        self.aruco_params.minDistanceToBorder = 3  # Minimum distance to image border
        
        # Marker validation parameters
        self.aruco_params.markerBorderBits = 1  # Border bits in marker
        self.aruco_params.minOtsuStdDev = 5.0  # Minimum standard deviation for Otsu's method
        self.aruco_params.perspectiveRemovePixelPerCell = 4  # Pixels per cell for perspective removal
        self.aruco_params.perspectiveRemoveIgnoredMarginPerCell = 0.13  # Ignored margin per cell
        
        # Error correction
        self.aruco_params.maxErroneousBitsInBorderRate = 0.35  # Max erroneous bits in border
        self.aruco_params.errorCorrectionRate = 0.6  # Error correction rate
        
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.get_logger().info('PoseDetector initialized with enhanced accuracy parameters')

    # Camera info callback
    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info("Camera info received and stored.")
            self.destroy_subscription(self.camera_info_sub)

    # Conversion of pixel to world using homography
    def pixel_to_world(self, pixel_x, pixel_y):
        if self.H_matrix is None:
            return None, None
        pixel_point = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
        world_point = cv2.perspectiveTransform(pixel_point, self.H_matrix) 
        return world_point[0][0][0], world_point[0][0][1] 

    # Compute Homography using corner markers with improved accuracy
    def compute_homography_matrix(self, corners, ids):
        """
        Computes Homography matrix using specific corner pixels of reference markers.
        Enhanced with improved RANSAC parameters for better accuracy.
        """
        pixel_pts = []
        world_pts = []
        
        # Map corner marker IDs to their specific corner pixel coordinates
        for i, id in enumerate(ids):
            if id in CORNER_INDEX_MAP:
                corner_index = CORNER_INDEX_MAP[id]
                specific_corner_pixel = corners[i][0][corner_index]
                pixel_pts.append(specific_corner_pixel)
                world_pts.append(corner_markers_ref_mm[id])

        # Compute homography if we have at least 4 reference points
        if len(pixel_pts) >= 4:
            pixel_matrix = np.array(pixel_pts, dtype=np.float32)
            world_matrix = np.array(world_pts, dtype=np.float32)
            
            # ========== ACCURACY IMPROVEMENT: Enhanced RANSAC parameters ==========
            H, _ = cv2.findHomography(
                pixel_matrix, 
                world_matrix, 
                cv2.RANSAC,
                ransacReprojThreshold=1.0,  # Stricter threshold for better accuracy (reduced from 5.0)
                maxIters=2000,              # More iterations for better convergence
                confidence=0.999            # Higher confidence level (99.9%)
            )
            
            if H is not None:
                self.H_matrix = H
                self.get_logger().info("Homography Matrix computed successfully with enhanced accuracy.")
            else:
                self.get_logger().warn("Homography computation failed.")
    
    def calculate_yaw_from_corners(self, marker_corners_pixel):
        """
        Calculate yaw angle using homography-transformed marker corners.
        This eliminates parallax error for top-down cameras.
        """
        if self.H_matrix is None:
            return 0.0
        
        # Transform all 4 corners to world coordinates
        corners_world = []
        for corner in marker_corners_pixel:
            wx, wy = self.pixel_to_world(corner[0], corner[1])
            if wx is not None and wy is not None:
                corners_world.append([wx, wy])
        
        if len(corners_world) < 4:
            return 0.0
        
        # Calculate marker X-axis direction using top-left and top-right corners
        # ArUco corner order: [TL, TR, BR, BL]
        top_left = np.array(corners_world[0])
        top_right = np.array(corners_world[1])
        
        # Vector from top-left to top-right (marker's X-axis)
        x_axis = top_right - top_left
        
        # Calculate yaw angle
        yaw_rad = math.atan2(x_axis[1], x_axis[0])
        yaw_deg = math.degrees(yaw_rad)
        
        # Normalize to 0-360
        yaw_deg = (yaw_deg + 360) % 360
        
        return yaw_deg
    
    # Image callback
    def image_callback(self, msg):
        display_image = None
        try:
            if not self.camera_info_received:
                self.get_logger().warn("Waiting for camera info...", throttle_duration_sec=5.0)
                return
            
            # ROS image to OpenCV
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Correct image distortion
            undistorted = cv2.undistort(image, self.camera_matrix, self.dist_coeffs)

            # Convert to grayscale for ArUco detection
            gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
            
            # ========== ACCURACY IMPROVEMENT: Apply image enhancement ==========
            # Apply CLAHE (Contrast Limited Adaptive Histogram Equalization) for better marker detection
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            gray = clahe.apply(gray)
            
            # Optional: Apply Gaussian blur to reduce noise (very mild to preserve edges)
            gray = cv2.GaussianBlur(gray, (3, 3), 0)

            # Detect ArUco markers (now with enhanced detection parameters)
            corners, ids, _ = self.detector.detectMarkers(gray)
            display_image = undistorted.copy() 

            if ids is None:                 
                cv2.imshow('Detected Markers', display_image) 
                cv2.waitKey(1)
                return                 
              
            ids = ids.flatten()        
            cv2.aruco.drawDetectedMarkers(display_image, corners, ids)
            
            # Compute Homography matrix
            if self.H_matrix is None:
                self.compute_homography_matrix(corners, ids)
                if self.H_matrix is None:
                    self.get_logger().warn("Homography not computed yet.", throttle_duration_sec=5.0)
                    cv2.imshow('Detected Markers', display_image)
                    cv2.waitKey(1)
                    return
            
            # Detect poses of bots and crates
            crate_poses = {} 
            bot_poses = {}   
            
            for i, id in enumerate(ids):
                # Skip corner calibration markers
                if id in corner_markers_ref_mm:
                    continue

                marker_corners_pixel = corners[i][0]
                
                # Calculate CENTER position using homography
                center_pixel = np.mean(marker_corners_pixel, axis=0)
                world_x_mm, world_y_mm = self.pixel_to_world(center_pixel[0], center_pixel[1])

                if world_x_mm is None:
                    continue
                
                # Calculate yaw using homography (NO 3D methods = NO parallax error!)
                yaw_deg = self.calculate_yaw_from_corners(marker_corners_pixel)
                
                pose_data = (world_x_mm, world_y_mm, yaw_deg)

                if id == bot_id:
                    bot_poses[id] = pose_data
                elif id in crate_ids:
                    crate_poses[id] = pose_data

            self.publish_crate_poses(crate_poses)
            self.publish_bot_poses(bot_poses)

            # Visualization
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
                
                # Draw orientation arrow
                center_x, center_y = center_pixel
                angle_rad = np.radians(yaw_deg)
                arrow_len = 40 
                end_x = int(center_x + arrow_len * np.cos(angle_rad))
                end_y = int(center_y - arrow_len * np.sin(angle_rad))
                cv2.arrowedLine(display_image, (center_x, center_y), (end_x, end_y), (255, 255, 0), 2)
 
            cv2.imshow('Detected Markers', display_image)
            cv2.waitKey(1)
            
        except Exception as e:
            if display_image is not None:
                cv2.imshow('Detected Markers', display_image)
                cv2.waitKey(1)
            self.get_logger().error(f"Image callback error: {e}")

    # Publish crate poses
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

    # Publish bot poses 
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

# Main function
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