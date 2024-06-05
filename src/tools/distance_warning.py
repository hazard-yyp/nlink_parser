#!/usr/bin/env python

import rospy
from nlink_parser.msg import LinktrackNodeframe2
import cv2
import numpy as np

class DangerZoneChecker:
    def __init__(self):
        self.nodes_in_danger = {}  # Stores the ID and distance of nodes currently in the danger zone
        self.ignored_ids = {0, 1, 2, 3}  # IDs to ignore
        self.img = np.zeros((600, 800, 3), dtype=np.uint8)  # Initialize the image

    def check_danger_zone(self, data):
        current_ids = {node.id: node.dis for node in data.nodes if node.id not in self.ignored_ids}
        
        # Check each node
        for node_id, distance in current_ids.items():
            if distance < 5:
                self.nodes_in_danger[node_id] = distance
            else:
                if node_id in self.nodes_in_danger:
                    del self.nodes_in_danger[node_id]

        # Update the visualization
        self.update_visualization(current_ids)

        # Log warnings or safety information
        if self.nodes_in_danger:
            for node_id, distance in self.nodes_in_danger.items():
                rospy.logwarn(f"Alert: ID {node_id} may have entered the danger zone! Current distance or last known distance: {distance} meters")
        else:
            safe_info = ", ".join([f"ID {node_id}: {distance} meters" for node_id, distance in current_ids.items()])
            rospy.loginfo(f"Safety Information: {safe_info}")

    def update_visualization(self, current_ids):
        self.img.fill(0)  # Clear the image
        start_x, start_y = 50, 50  # Starting position
        interval = 30  # Interval
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        font_thickness = 1
        color_safe = (0, 255, 0)  # Green color for safe nodes
        color_warn = (0, 0, 255)  # Red color for nodes in danger

        for i, (node_id, distance) in enumerate(current_ids.items()):
            color = color_warn if distance < 5 else color_safe
            cv2.putText(self.img, f"Node ID: {node_id}, Distance: {distance:.2f}", (start_x, start_y + i*interval), font, font_scale, color, font_thickness)

        cv2.imshow("Danger Zone Visualization", self.img)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('danger_zone_checker', anonymous=True)
    danger_checker = DangerZoneChecker()
    rospy.Subscriber('/node0_topic', LinktrackNodeframe2, danger_checker.check_danger_zone)
    rospy.spin()
    cv2.destroyAllWindows()  # Ensure all OpenCV windows are destroyed when the program is closed
