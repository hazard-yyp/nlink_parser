#!/usr/bin/env python

import rospy
from nlink_parser.msg import LinktrackNodeframe2,LinktrackNode2
from collections import defaultdict
import time

class LatestDistancePublisher:
    def __init__(self):
        self.latest_distances = defaultdict(lambda: {'distance': 0, 'timestamp': None})
        
    def update_distance(self, node_id, distance, timestamp):
        self.latest_distances[node_id] = {'distance': distance, 'timestamp': timestamp}
        
    def publish_latest_distances(self):
        pub = rospy.Publisher('/latest_distances', LinktrackNodeframe2, queue_size=10)
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            msg = LinktrackNodeframe2()
            
            for node_id, data in self.latest_distances.items():
                if data['timestamp'] is not None:
                    node = LinktrackNode2()
                    node.id = node_id
                    node.dis = data['distance']
                    msg.nodes.append(node)
                    
                    # Copy timestamp from original message
                    msg.header.stamp = data['timestamp']
            
            if msg.nodes:
                pub.publish(msg)
            
            rate.sleep()

distance_publisher = LatestDistancePublisher()

def callback(data):
    for node in data.nodes:
        if node.id == 4:
            distance_publisher.update_distance(data.id, node.dis, data.header.stamp)

if __name__ == '__main__':
    try:
        rospy.init_node('latest_distance_publisher', anonymous=True)
        rospy.Subscriber('/node0_topic', LinktrackNodeframe2, callback)
        rospy.Subscriber('/node1_topic', LinktrackNodeframe2, callback)
        rospy.Subscriber('/node2_topic', LinktrackNodeframe2, callback)
        rospy.Subscriber('/node3_topic', LinktrackNodeframe2, callback)
        distance_publisher.publish_latest_distances()
    except rospy.ROSInterruptException:
        pass
