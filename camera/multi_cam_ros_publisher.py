#!/usr/bin/env python3

"""
SIMPLE Multi-Camera ROS Publisher for 4 OAK + 1 USB Camera
No confusing socket mappings - just specify A, B, C, D sockets
Easy to understand and modify
"""

import rospy
import cv2
import numpy as np
import depthai as dai
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import signal
import sys

# ========== CONFIGURATION ==========
# Edit these variables to customize your setup

# Which camera sockets to use (A, B, C, D, E)
CAMERA_SOCKETS = ['A', 'B', 'C', 'D']  # Your 4 OAK camera sockets

# USB Camera
USB_CAMERA_ID = 0  # Change this if your USB camera has different ID

# Settings
RESOLUTION = '720'         # '720', '800', '1080'
USB_RESOLUTION = (640, 480)  # (width, height) for USB camera
FPS = 15.0

# ===================================

class SimpleMultiCameraPublisher:
    def __init__(self):
        rospy.init_node('simple_multi_camera_publisher', anonymous=True)
        self.bridge = CvBridge()
        self.publishers = {}
        self.running = True
        self.device = None
        
        # Simple socket mapping
        self.sockets = {
            'A': dai.CameraBoardSocket.CAM_A,
            'B': dai.CameraBoardSocket.CAM_B,
            'C': dai.CameraBoardSocket.CAM_C,
            'D': dai.CameraBoardSocket.CAM_D,
            'E': dai.CameraBoardSocket.CAM_E,
        }
        
        # Resolution mapping
        self.resolutions = {
            '720': dai.ColorCameraProperties.SensorResolution.THE_720_P,
            '800': dai.ColorCameraProperties.SensorResolution.THE_800_P,
            '1080': dai.ColorCameraProperties.SensorResolution.THE_1080_P,
        }
        
        # Find available cameras
        self.available_sockets = self.find_available_cameras()
        
        # Setup publishers and start cameras
        self.setup_publishers()
        self.start_cameras()
    
    def find_available_cameras(self):
        """Find which sockets have cameras"""
        available = []
        try:
            device_infos = dai.Device.getAllAvailableDevices()
            if not device_infos:
                rospy.logwarn("No DepthAI devices found")
                return available
                
            with dai.Device(device_infos[0]) as temp_device:
                connected = temp_device.getConnectedCameraFeatures()
                
                rospy.loginfo(f"Found {len(connected)} connected cameras:")
                for cam in connected:
                    rospy.loginfo(f"  - Socket {cam.socket.name}: {cam.sensorName}")
                
                # Check which of our desired sockets have cameras
                for socket_name in CAMERA_SOCKETS:
                    socket_enum = self.sockets[socket_name]
                    for cam in connected:
                        if cam.socket == socket_enum:
                            available.append(socket_name)
                            rospy.loginfo(f"✓ Will use socket {socket_name}")
                            break
                    else:
                        rospy.logwarn(f"✗ No camera on socket {socket_name}")
            
        except Exception as e:
            rospy.logerr(f"Error detecting cameras: {e}")
            
        return available
    
    def setup_publishers(self):
        """Setup ROS publishers for available cameras"""
        # Camera position mapping
        camera_positions = {
            'A': 'front_right',
            'B': 'right_side', 
            'C': 'left_side',
            'D': 'front_left'
        }
        
        # OAK cameras
        for socket in self.available_sockets:
            position = camera_positions[socket]
            topic = f"/cameras/{position}/image_raw"
            self.publishers[socket] = rospy.Publisher(topic, Image, queue_size=1)
            rospy.loginfo(f"Publisher ready: {topic}")
        
        # USB camera (downward)
        self.publishers['usb'] = rospy.Publisher("/cameras/downward/image_raw", Image, queue_size=1)
        rospy.loginfo("Publisher ready: /cameras/downward/image_raw")
    
    def create_pipeline(self):
        """Create simple pipeline for available cameras"""
        if not self.available_sockets:
            return None, []
            
        pipeline = dai.Pipeline()
        stream_names = []
        
        for socket_name in self.available_sockets:
            try:
                # Create color camera
                cam = pipeline.createColorCamera()
                cam.setResolution(self.resolutions[RESOLUTION])
                cam.setBoardSocket(self.sockets[socket_name])
                cam.setFps(FPS)
                
                # Create output
                xout = pipeline.createXLinkOut()
                stream_name = f"socket_{socket_name}"
                xout.setStreamName(stream_name)
                cam.isp.link(xout.input)
                
                stream_names.append(stream_name)
                rospy.loginfo(f"Added socket {socket_name} to pipeline")
                
            except Exception as e:
                rospy.logerr(f"Error adding socket {socket_name}: {e}")
        
        return pipeline, stream_names
    
    def depthai_thread(self):
        """Simple DepthAI thread"""
        try:
            
            pipeline, stream_names = self.create_pipeline()
            # Creates a DepthAI pipeline configuration for all available camera sockets
            # Returns the pipeline object and a list of stream names (like "socket_A", "socket_B", etc.)
            
            if pipeline is None:
                rospy.logwarn("No DepthAI cameras available")
                return
            
            # Connect to device
            self.device = dai.Device(pipeline)
            # Connects to the physical DepthAI device using the configured pipeline
            # This establishes the connection to your OAK cameras
            rospy.loginfo("DepthAI device connected")
            
            # Create queues
            queues = {}
            for stream_name in stream_names:
                queues[stream_name] = self.device.getOutputQueue(stream_name, maxSize=2, blocking=False)
            
            rospy.loginfo(f"Started {len(stream_names)} DepthAI cameras")
            
            # Main loop
            while self.running and not rospy.is_shutdown():
                for stream_name in stream_names:
                    try:
                        frame_data = queues[stream_name].tryGet()
                        if frame_data is not None:
                            frame = frame_data.getCvFrame()
                            if frame is not None and frame.size > 0:
                                # Create ROS message
                                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                                ros_image.header.stamp = rospy.Time.now()
                                
                                # Get socket name from stream name and map to position
                                socket_name = stream_name.replace("socket_", "")
                                camera_positions = {
                                    'A': 'front_right',
                                    'B': 'right_side', 
                                    'C': 'left_side',
                                    'D': 'front_left'
                                }
                                position = camera_positions[socket_name]
                                ros_image.header.frame_id = position
                                
                                self.publishers[socket_name].publish(ros_image)
                    except Exception as e:
                        rospy.logwarn(f"Frame error for {stream_name}: {e}")
                
                time.sleep(0.005)
                
        except Exception as e:
            rospy.logerr(f"DepthAI error: {e}")
        finally:
            self.cleanup_device()
    
    def usb_camera_thread(self):
        """Simple USB camera thread"""
        try:
            cap = cv2.VideoCapture(USB_CAMERA_ID)
            if not cap.isOpened():
                rospy.logwarn(f"USB camera {USB_CAMERA_ID} not found")
                return
            
            # Configure camera
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, USB_RESOLUTION[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, USB_RESOLUTION[1])
            cap.set(cv2.CAP_PROP_FPS, FPS)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            rospy.loginfo(f"USB camera {USB_CAMERA_ID} opened")
            
            while self.running and not rospy.is_shutdown():
                ret, frame = cap.read()
                if ret and frame is not None:
                    try:
                        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                        ros_image.header.stamp = rospy.Time.now()
                        ros_image.header.frame_id = "downward"
                        self.publishers['usb'].publish(ros_image)
                    except Exception as e:
                        rospy.logwarn(f"USB frame error: {e}")
                else:
                    time.sleep(0.1)
            
            cap.release()
            rospy.loginfo("USB camera released")
            
        except Exception as e:
            rospy.logerr(f"USB camera error: {e}")
    
    def start_cameras(self):
        """Start camera threads"""
        if self.available_sockets:
            threading.Thread(target=self.depthai_thread, daemon=True).start()
            rospy.loginfo("DepthAI thread started")
        
        threading.Thread(target=self.usb_camera_thread, daemon=True).start()
        rospy.loginfo("USB camera thread started")
    
    def cleanup_device(self):
        """Cleanup DepthAI device"""
        try:
            if self.device is not None:
                self.device.close()
                self.device = None
        except Exception as e:
            rospy.logerr(f"Cleanup error: {e}")
    
    def shutdown(self):
        """Shutdown everything"""
        self.running = False
        self.cleanup_device()

def main():
    try:
        publisher = SimpleMultiCameraPublisher()
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("SIMPLE MULTI-CAMERA PUBLISHER RUNNING")
        rospy.loginfo(f"OAK cameras: {len(publisher.available_sockets)}")
        rospy.loginfo(f"Resolution: {RESOLUTION}p, FPS: {FPS}")
        rospy.loginfo("Topics:")
        
        # Camera position mapping for logging
        camera_positions = {
            'A': 'front_right',
            'B': 'right_side', 
            'C': 'left_side',
            'D': 'front_left'
        }
        
        for socket in publisher.available_sockets:
            position = camera_positions[socket]
            rospy.loginfo(f"  /cameras/{position}/image_raw (Socket {socket})")
        rospy.loginfo("  /cameras/downward/image_raw (USB)")
        rospy.loginfo("Press Ctrl+C to stop")
        rospy.loginfo("=" * 50)
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {e}")
    finally:
        if 'publisher' in locals():
            publisher.shutdown()

if __name__ == '__main__':
    main()
