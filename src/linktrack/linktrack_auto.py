#!/usr/bin/env python
import rospy
import serial.tools.list_ports
from subprocess import call

def start_node(port):
    # Calculate baud rate and topic name from port
    baud_rate = "921600"
    topic_name = port.replace('/dev/', '')
    
    # Command to launch ROS node
    command = "rosrun nlink_parser linktrack _port_name:={} _baud_rate:={} _topic_name:={}".format(port, baud_rate, topic_name)
    call(command.split())

def main():
    rospy.init_node('linktrack_auto', anonymous=True)
    
    # List all serial ports
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if "ttyUSB" in port.device or "ttyACM" in port.device:
            rospy.loginfo("Starting node for port: {}".format(port.device))
            start_node(port.device)
    
    rospy.spin()

if __name__ == '__main__':
    main()
