#!/usr/bin/env python3
import cv2
import subprocess
import time
import yaml
import signal
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraTester(Node):
    def __init__(self):
        super().__init__('camera_tester_node')
        
        self.CAMERA_ID = 'video2'
        self.bridge = CvBridge()
        
        # Create publisher for /image_raw topic
        self.image_publisher = self.create_publisher(Image, '/image_raw', 10)
        
        # Setup signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
        
        print("=" * 60)
        print("Camera Testing Utility")
        print("=" * 60)
        
        # Try to configure camera and print available formats
        try:
            result = subprocess.run(
                ["v4l2-ctl", "-d", f"/dev/{self.CAMERA_ID}", "--list-formats-ext"], 
                capture_output=True, text=True, check=True
            )
            print("\nAvailable camera formats:")
            print(result.stdout)

            print("\nAttempting to set camera to 1920x1080 MJPG...")
            subprocess.run(
                ["v4l2-ctl", "-d", f"/dev/{self.CAMERA_ID}", "--set-fmt-video=width=1920,height=1080,pixelformat=MJPG"]
            )
            subprocess.run(["v4l2-ctl", "-d", f"/dev/{self.CAMERA_ID}", "-c", "auto_exposure=1"])
        except subprocess.CalledProcessError as e:
            print(f"Warning: v4l2-ctl command failed: {e}")

        # Open camera
        print(f"\nOpening camera /dev/{self.CAMERA_ID}...")
        self.cap = cv2.VideoCapture(int(self.CAMERA_ID[-1]))
        
        # Set MJPG format
        fourcc = getattr(cv2, "VideoWriter_fourcc", None)
        if fourcc is not None:
            self.cap.set(cv2.CAP_PROP_FOURCC, fourcc(*"MJPG"))

        if not self.cap.isOpened():
            raise RuntimeError("Failed to open camera")
        
        # Set resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        
        # Print camera properties
        self.print_camera_info()
        
    def print_camera_info(self):
        print("\n" + "=" * 60)
        print("Camera Properties:")
        print("=" * 60)
        
        properties = {
            'Frame Width': cv2.CAP_PROP_FRAME_WIDTH,
            'Frame Height': cv2.CAP_PROP_FRAME_HEIGHT,
            'FPS': cv2.CAP_PROP_FPS,
            'Format': cv2.CAP_PROP_FORMAT,
            'FOURCC': cv2.CAP_PROP_FOURCC,
            'Brightness': cv2.CAP_PROP_BRIGHTNESS,
            'Contrast': cv2.CAP_PROP_CONTRAST,
            'Saturation': cv2.CAP_PROP_SATURATION,
            'Hue': cv2.CAP_PROP_HUE,
            'Gain': cv2.CAP_PROP_GAIN,
            'Exposure': cv2.CAP_PROP_EXPOSURE,
            'Auto Exposure': cv2.CAP_PROP_AUTO_EXPOSURE,
            'Backend': cv2.CAP_PROP_BACKEND,
            'Buffer Size': cv2.CAP_PROP_BUFFERSIZE,
        }
        
        for name, prop in properties.items():
            value = self.cap.get(prop)
            if name == 'FOURCC' and value > 0:
                fourcc_str = "".join([chr((int(value) >> 8 * i) & 0xFF) for i in range(4)])
                print(f"{name:20s}: {int(value)} ({fourcc_str})")
            else:
                print(f"{name:20s}: {value}")
        
        print("=" * 60)
        
    def get_camera_properties(self):
        """Get all camera properties as a dictionary"""
        properties = {
            'Frame Width': int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            'Frame Height': int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            'FPS': float(self.cap.get(cv2.CAP_PROP_FPS)),
            'Format': int(self.cap.get(cv2.CAP_PROP_FORMAT)),
            'FOURCC': int(self.cap.get(cv2.CAP_PROP_FOURCC)),
            'Brightness': float(self.cap.get(cv2.CAP_PROP_BRIGHTNESS)),
            'Contrast': float(self.cap.get(cv2.CAP_PROP_CONTRAST)),
            'Saturation': float(self.cap.get(cv2.CAP_PROP_SATURATION)),
            'Hue': float(self.cap.get(cv2.CAP_PROP_HUE)),
            'Gain': float(self.cap.get(cv2.CAP_PROP_GAIN)),
            'Exposure': float(self.cap.get(cv2.CAP_PROP_EXPOSURE)),
            'Auto Exposure': float(self.cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)),
            'Backend': int(self.cap.get(cv2.CAP_PROP_BACKEND)),
            'Buffer Size': int(self.cap.get(cv2.CAP_PROP_BUFFERSIZE)),
        }
        
        # Add FOURCC as string
        fourcc_value = properties['FOURCC']
        if fourcc_value > 0:
            fourcc_str = "".join([chr((int(fourcc_value) >> 8 * i) & 0xFF) for i in range(4)])
            properties['FOURCC_String'] = fourcc_str
        
        return properties
    
    def save_properties_to_yaml(self):
        """Save camera properties to YAML file"""
        properties = self.get_camera_properties()
        
        with open('camera_testing.yaml', 'w') as file:
            yaml.dump(properties, file, default_flow_style=False)
        
        print("\nCamera properties saved to camera_testing.yaml")
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C signal"""
        print("\nCtrl+C detected. Saving properties...")
        self.save_properties_to_yaml()
        self.cap.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()
        exit(0)
        
    def run(self):
        print("\nStarting video stream...")
        print("Press 'q' to quit, 'i' to print camera info again")
        print("Publishing to ROS2 topic: /image_raw")
        
        frame_count = 0
        start_time = time.time()
        
        while True:
            ret, frame = self.cap.read()
            
            if not ret:
                print("Failed to capture frame")
                break
            
            frame_count += 1
            elapsed_time = time.time() - start_time
            
            # Calculate actual FPS
            if elapsed_time > 0:
                actual_fps = frame_count / elapsed_time
            else:
                actual_fps = 0
            
            # Add info overlay on frame
            height, width = frame.shape[:2]
            info_text = [
                f"Resolution: {width}x{height}",
                f"Actual FPS: {actual_fps:.2f}",
                f"Frame: {frame_count}",
                f"Brightness: {cv2.mean(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))[0]:.1f}"
            ]
            
            y_offset = 30
            for text in info_text:
                cv2.putText(frame, text, (10, y_offset), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                y_offset += 30
            
            # Display frame
            cv2.namedWindow('Camera Test', cv2.WINDOW_NORMAL)
            cv2.imshow('Camera Test', frame)
            
            # Publish frame to ROS2 topic
            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_publisher.publish(ros_image)
            except Exception as e:
                self.get_logger().error(f"Failed to publish image: {e}")
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.save_properties_to_yaml()
                break
            elif key == ord('i'):
                self.print_camera_info()
            
            # Print FPS every 30 frames
            if frame_count % 30 == 0:
                print(f"Frames: {frame_count} | Actual FPS: {actual_fps:.2f}")
        
        print(f"\nTest completed. Total frames: {frame_count}")
        print(f"Average FPS: {frame_count / elapsed_time:.2f}")
        
    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    
    try:
        tester = CameraTester()
        tester.run()
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
