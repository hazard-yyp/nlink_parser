#!/usr/bin/env python

import rospy
from nlink_parser.msg import LinktrackNodeframe2

class DangerZoneChecker:
    def __init__(self):
        self.nodes_in_danger = {}  # 保存当前在危险区域内的节点id和其距离
        self.ignored_ids = {0, 1, 2, 3}

    def check_danger_zone(self, data):
        current_ids = {node.id: node.dis for node in data.nodes if node.id not in self.ignored_ids}
        
        # 检查每一个节点
        for node_id, distance in current_ids.items():
            if distance < 5:
                self.nodes_in_danger[node_id] = distance
            else:
                if node_id in self.nodes_in_danger:
                    del self.nodes_in_danger[node_id]

        # 如果某个ID在上一次消息中是处于危险区域，但在这次消息中没有数据，则保留该ID和它的最后已知距离
        for node_id in list(self.nodes_in_danger.keys()):
            if node_id not in current_ids:
                self.nodes_in_danger[node_id] = self.nodes_in_danger.get(node_id)
        
        if self.nodes_in_danger:
            for node_id, distance in self.nodes_in_danger.items():
                rospy.logwarn(f"警报：ID {node_id} 可能进入危险区域！当前距离或最后的已知距离：{distance}米")
        else:
            safe_info = ", ".join([f"ID {node_id}: {distance}米" for node_id, distance in current_ids.items()])
            rospy.loginfo(f"安全信息：{safe_info}")


if __name__ == '__main__':
    rospy.init_node('danger_zone_checker', anonymous=True)
    danger_checker = DangerZoneChecker()
    #rospy.Subscriber('/nlink_linktrack_nodeframe2', LinktrackNodeframe2, danger_checker.check_danger_zone)
    rospy.Subscriber('/node0_topic', LinktrackNodeframe2, danger_checker.check_danger_zone)
    rospy.spin()
