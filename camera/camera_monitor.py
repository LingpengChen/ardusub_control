#!/usr/bin/env python3

"""
Simple camera topic monitor
Shows FPS and basic info for all camera topics
"""

import rospy
import time
from sensor_msgs.msg import Image
from collections import defaultdict

class CameraMonitor:
    def __init__(self):
        rospy.init_node('camera_monitor', anonymous=True)
        
        # Camera topics to monitor
        self.topics = [
            '/cameras/oak_rgb/image_raw',
            '/cameras/oak_left/image_raw', 
            '/cameras/oak_right/image_raw',
            '/cameras/oak_camd/image_raw',
            '/cameras/usb_cam/image_raw'
        ]
        
        # Statistics
        self.frame_counts = defaultdict(int)
        self.last_times = defaultdict(float)
        self.fps_stats = defaultdict(list)
        
        # Subscribe to all topics
        self.subscribers = []
        for topic in self.topics:
            sub = rospy.Subscriber(topic, Image, self.image_callback, topic)
            self.subscribers.append(sub)
        
        rospy.loginfo("Camera monitor started")
        rospy.loginfo(f"Monitoring topics: {self.topics}")
        
    def image_callback(self, msg, topic):
        """Callback for image messages"""
        current_time = time.time()
        self.frame_counts[topic] += 1
        
        # Calculate FPS
        if self.last_times[topic] > 0:
            fps = 1.0 / (current_time - self.last_times[topic])
            self.fps_stats[topic].append(fps)
            
            # Keep only last 30 fps measurements
            if len(self.fps_stats[topic]) > 30:
                self.fps_stats[topic].pop(0)
        
        self.last_times[topic] = current_time
    
    def print_stats(self):
        """Print current statistics"""
        print("\n" + "="*80)
        print(f"{'Camera Topic':<35} {'Frames':<10} {'Current FPS':<12} {'Avg FPS':<10}")
        print("="*80)
        
        for topic in self.topics:
            frames = self.frame_counts[topic]
            
            if len(self.fps_stats[topic]) > 0:
                current_fps = self.fps_stats[topic][-1]
                avg_fps = sum(self.fps_stats[topic]) / len(self.fps_stats[topic])
                print(f"{topic:<35} {frames:<10} {current_fps:<12.1f} {avg_fps:<10.1f}")
            else:
                print(f"{topic:<35} {frames:<10} {'--':<12} {'--':<10}")
        
        print("="*80)
    
    def run(self):
        """Main monitoring loop"""
        rate = rospy.Rate(1)  # 1 Hz for stats
        
        while not rospy.is_shutdown():
            self.print_stats()
            rate.sleep()

def main():
    try:
        monitor = CameraMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Camera monitor stopped")

if __name__ == '__main__':
    main()
