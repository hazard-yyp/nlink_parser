#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from nlink_parser.msg import LinktrackNodeframe1
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def create_warning_marker(node):
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = rospy.Time.now()
    marker.ns = 'warning_nodes'
    marker.id = node.id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 2  # Adjust the height of the warning text
    marker.pose.orientation.w = 1.0
    marker.scale.z = 1  # Text size
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.text = "Warning: Worker ID {} is in danger area".format(node.id)
    marker.lifetime = rospy.Duration(0.5)
    return marker

def node_callback(data):
    # Create a marker array to hold all markers
    marker_array = MarkerArray()

    # Create a marker array to hold warning markers
    warning_marker_array = MarkerArray()

    # List of base station positions
    base_stations = [
        #{'id': 1, 'position': [0, 0, 0]},
        #{'id': 2, 'position': [-0.578, 19.693, 0]},
        #{'id': 3, 'position': [72.419, 20.797, 0]},
        #{'id': 4, 'position': [72.614, 0, 0]}, carpark
        #{'id': 1, 'position': [0, 0, 0]},
        #{'id': 2, 'position': [-0.264, 4.13, 0]},
        #{'id': 3, 'position': [7.36, 4.229, 0]},
        #{'id': 4, 'position': [7.301, 0, 0]}, # site0
        {'id': 1, 'position': [0, 0, 0]},
        {'id': 2, 'position': [-0.044, 5.998, 0]},
        {'id': 3, 'position': [6.226, 6.262, 0]},
        {'id': 4, 'position': [6.565, 0, 0]}, # site1
    ]

    # Loop through base stations and create markers
    for station in base_stations:
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'base_stations'
        marker.id = station['id']
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = station['position'][0]
        marker.pose.position.y = station['position'][1]
        marker.pose.position.z = station['position'][2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2
        marker.scale.y = 2
        marker.scale.z = 2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker_array.markers.append(marker)

    # Loop through nodes and create markers
    for node in data.nodes:
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'nodes'
        marker.id = node.id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = node.pos_3d[0]
        marker.pose.position.y = node.pos_3d[1]
        marker.pose.position.z = node.pos_3d[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker_array.markers.append(marker)

        # Check if the coordinates are outside the desired range
        if (
            node.pos_3d[0] > 50 or node.pos_3d[0] < -50 or
            node.pos_3d[1] > 50 or node.pos_3d[1] < -50 or
            node.pos_3d[2] > 50 or node.pos_3d[2] < -50
        ):
            # Create a warning marker for this node
            warning_marker = create_warning_marker(node)
            warning_marker_array.markers.append(warning_marker)

    # Publish the marker arrays
    marker_pub.publish(marker_array)
    marker_pub.publish(warning_marker_array)  # Publish the warning markers

if __name__ == '__main__':
    rospy.init_node('visualization_node')
    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    node_sub = rospy.Subscriber('/nlink_linktrack_nodeframe1', LinktrackNodeframe1, node_callback)
    rospy.spin()
