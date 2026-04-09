#!/usr/bin/env python3
"""
This Python file runs a ROS 2 node named localization_node which publishes the position of crates and a holonomic drive robot.
This node subscribes to the following topics:
 SUBSCRIPTIONS
 /camera/image_raw
 /camera/camera_info
 /crates_pose
 /bot_pose
"""
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from hb_interfaces.msg import Pose2D, Poses2D

class PoseDetector(Node):
    def __init__(self):
        super().__init__('localization_node')
        
        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()
        
        # ---------- PARAMETERS ----------
        self.crates_marker_length = 0.05  # Set marker size in meters
        self.bots_marker_length = 0.05    # Set bot marker size in meters
        self.aruco_dict_name = 'DICT_4X4_50'  # Choose ArUco dictionary
        
        # ---------- TOPICS ----------
        self.image_sub = self.create_subscription(Image, "Write the Image Topic Name Here", self.image_callback, 10)
        self.crate_poses_pub = self.create_publisher(Poses2D, 'Write Box Topic Name Here', 10)
        self.bot_poses_pub = self.create_publisher(Poses2D, 'Write Bot Topic Name Here', 10)
        
        # ---------- CAMERA PARAMETERS ----------
        self.camera_matrix = None  # load camera intrinsics (3x3 matrix)
        self.dist_coeffs = None    # load distortion coefficients (1x5 array)
        
        # ---------- IMAGE MATRICES ----------
        self.pixel_matrix = []  # derive pixel points matrix [[x1,y1], [x2,y2], ...]
        self.world_matrix = []  # derive world points matrix [[x1,y1], [x2,y2], ...]
        self.H_matrix = None    # compute homography matrix using cv2.findHomography
        
        # ---------- ARUCO SETUP ----------
        # Initialize ArUco detector
        # self.aruco_dict = cv2.aruco.getPredefinedDictionary(?)
        # self.aruco_params = cv2.aruco.DetectorParameters()
        # self.detector = cv2.aruco.ArucoDetector(?, ?)
        
        self.get_logger().info('PoseDetector initialized')

    def pixel_to_world(self, pixel_x, pixel_y):
        """
        - Calculate the H_matrix using: use cv2.findHomography
        - Convert the pixel coordinates into real world coordinates using: cv2.perspectiveTransform(src_pts, self.H_matrix)
        """
        # Implement pixel to world coordinate conversion
        # Step 1: Ensure H_matrix is computed
        # Step 2: Create pixel point in correct format for cv2.perspectiveTransform
        # Step 3: Apply transformation and return world coordinates
        return None, None

    def image_callback(self, msg):
        """
        Callback function for the image subscriber.
        Main Steps:
        1) Convert ROS Image -> cv image using CvBridge
        2) Undistort the image using camera intrinsics
        3) Detect all the markers in the world (cv2.aruco.drawDetectedMarkers)
        4) Derive the Pixel Matrix and the World Matrix using Corner Markers
        5) Compute the Homography Matrix (cv2.findHomography)
        5) Convert center pixel of crates marker and bot markers to world coordinates
        6) Using OpenCV calculate the yaw angle of each marker (cv2.aruco.estimatePoseSingleMarkers)
        7) Convert the yaw angle as per the new coordinate system
        8) Publish the bot pose and crate poses using the given custom message type
        """
        try:
            # Step 1: Convert ROS Image -> cv image using CvBridge
            # Use self.bridge.imgmsg_to_cv2() to convert ROS image to OpenCV format
            
            # Step 2: Undistort the image using camera intrinsics
            # Use cv2.undistort() with camera_matrix and dist_coeffs
            # Convert to grayscale for marker detection
            
            # Step 3: Detect all the markers in the world
            # Use self.detector.detectMarkers() to find ArUco markers
            # Use cv2.aruco.drawDetectedMarkers() to visualize detected markers
            
            # Step 4: Derive the Pixel Matrix and the World Matrix using Corner Markers
            # Identify corner markers (IDs 1, 3, 5, 7)
            # Extract their pixel coordinates and map to known world coordinates
            
            # Step 5: Compute the Homography Matrix
            # Use cv2.findHomography() with pixel and world points
            
            # Step 6: Convert center pixel of markers to world coordinates
            # For each detected marker (excluding corner markers):
            #       - Calculate center pixel coordinate
            #       - Use pixel_to_world() to convert to world coordinates
            
            # Step 7: Calculate yaw angle of each marker
            # Use cv2.aruco.estimatePoseSingleMarkers() or any other method to get rotation vectors
            # If you are going ahead with it, convert rotation vector to rotation matrix using cv2.Rodrigues()
            # Extract yaw angle from rotation matrix
            
            # Step 8: Separate and publish poses
            # Create separate dictionaries for bot_poses and crate_poses
            # Call publish_crate_poses() and publish_bot_poses()
            
            # Display the image with detected markers
            # cv2.imshow('Detected Markers', undistorted_image)
            # cv2.waitKey(1)
            
            pass
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def publish_crate_poses(self, poses):
        """
        - Convert python pose dictionary -> message (Poses2D)
        - self.crate_poses_pub.publish(msg)
        """
        # Create Poses2D message
        # For each pose in poses list:
        #       - Create Pose2D message
        #       - Set id, x, y, w fields
        #       - Append to poses message
        # Publish the message
        pass

    def publish_bot_poses(self, poses):
        """
        - Convert python pose dictionary -> message (Poses2D)
        - self.bot_poses_pub.publish(msg)
        """
        # Create Poses2D message
        # For each pose in poses list:
        #       - Create Pose2D message
        #       - Set id, x, y, w fields
        #       - Append to poses message
        # Publish the message
        pass

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