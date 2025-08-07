#!/usr/bin/env python3

"""
Simple USB Camera ROS Publisher
Publishes USB camera feed to ROS topic
"""

import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import time

# ========== CONFIGURATION ==========
USB_CAMERA_ID = 0  # Change this if your USB camera has different ID
USB_RESOLUTION = (640, 480)  # (width, height) for USB camera
FPS = 15.0
TOPIC_NAME = "/cameras/downward/image_raw"  # ROS topic name
COMPRESSED_TOPIC_NAME = "/cameras/downward/image_raw/compressed"  # Compressed image topic
JPEG_QUALITY = 80  # JPEG compression quality (0-100, higher is better quality)

class SimpleUSBCameraPublisher:
    def __init__(self):
        rospy.init_node('simple_usb_camera_publisher', anonymous=True)
        self.bridge = CvBridge()
        self.running = True
        
        # Setup publisher
        self.publisher = rospy.Publisher(TOPIC_NAME, Image, queue_size=1)
        self.compressed_publisher = rospy.Publisher(COMPRESSED_TOPIC_NAME, CompressedImage, queue_size=1)
        rospy.loginfo(f"Raw image publisher ready: {TOPIC_NAME}")
        rospy.loginfo(f"Compressed image publisher ready: {COMPRESSED_TOPIC_NAME}")
        
        # Start camera
        self.start_camera()
    
    def start_camera(self):
        """Start USB camera and publish frames"""
        try:
            # Open camera
            cap = cv2.VideoCapture(USB_CAMERA_ID)
            if not cap.isOpened():
                rospy.logerr(f"USB camera {USB_CAMERA_ID} not found")
                return
            
            # Configure camera
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, USB_RESOLUTION[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, USB_RESOLUTION[1])
            cap.set(cv2.CAP_PROP_FPS, FPS)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            rospy.loginfo(f"USB camera {USB_CAMERA_ID} opened successfully")
            rospy.loginfo(f"Resolution: {USB_RESOLUTION[0]}x{USB_RESOLUTION[1]}, FPS: {FPS}")
            
            # Main publishing loop
            while self.running and not rospy.is_shutdown():
                ret, frame = cap.read()
                if ret and frame is not None:
                    try:
                        # Convert to ROS message
                        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                        ros_image.header.stamp = rospy.Time.now()
                        ros_image.header.frame_id = "downward"
                        
                        # Publish raw image
                        self.publisher.publish(ros_image)
                        
                        # Create and publish compressed image
                        compressed_msg = CompressedImage()
                        compressed_msg.header.stamp = ros_image.header.stamp
                        compressed_msg.header.frame_id = ros_image.header.frame_id
                        compressed_msg.format = "jpeg"
                        
                        # Compress image to JPEG
                        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
                        result, compressed_data = cv2.imencode('.jpg', frame, encode_param)
                        if result:
                            compressed_msg.data = compressed_data.tobytes()
                            self.compressed_publisher.publish(compressed_msg)
                        else:
                            rospy.logwarn("Failed to compress image")
                        
                    except Exception as e:
                        rospy.logwarn(f"Frame conversion error: {e}")
                else:
                    rospy.logwarn("Failed to capture frame")
                    time.sleep(0.1)
                
                # Small delay to control frame rate
                time.sleep(1.0 / FPS)
            
            # Cleanup
            cap.release()
            rospy.loginfo("USB camera released")
            
        except Exception as e:
            rospy.logerr(f"USB camera error: {e}")
    
    def shutdown(self):
        """Shutdown camera"""
        self.running = False

def main():
    try:
        publisher = SimpleUSBCameraPublisher()
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("SIMPLE USB CAMERA PUBLISHER RUNNING")
        rospy.loginfo(f"Camera ID: {USB_CAMERA_ID}")
        rospy.loginfo(f"Resolution: {USB_RESOLUTION[0]}x{USB_RESOLUTION[1]}")
        rospy.loginfo(f"FPS: {FPS}")
        rospy.loginfo(f"Raw topic: {TOPIC_NAME}")
        rospy.loginfo(f"Compressed topic: {COMPRESSED_TOPIC_NAME}")
        rospy.loginfo(f"JPEG quality: {JPEG_QUALITY}")
        rospy.loginfo("Press Ctrl+C to stop")
        rospy.loginfo("=" * 50)
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("USB camera publisher stopped")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
    finally:
        if 'publisher' in locals():
            publisher.shutdown()

if __name__ == '__main__':
    main()
