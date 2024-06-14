#!/usr/bin/env python3
import rospy
import serial.tools.list_ports
from subprocess import Popen

def start_node(port):
    # Calculate baud rate and topic name from port
    baud_rate = "921600"
    topic_name = port.replace('/dev/', '')
    
    # Command to launch ROS node
    command = "rosrun nlink_parser linktrack _port_name:={} _baud_rate:={} _topic_name:={}".format(port, baud_rate, topic_name)
    # Start the process detached from the parent
    return Popen(command.split(), preexec_fn=lambda: os.setpgrp())

def main():
    rospy.init_node('linktrack_auto', anonymous=True)
    
    # List all serial ports
    ports = list(serial.tools.list_ports.comports())
    processes = []
    for port in ports:
        if "ttyUSB" in port.device or "ttyACM" in port.device:
            rospy.loginfo("Starting node for port: {}".format(port.device))
            processes.append(start_node(port.device))
    
    rospy.spin()  # Keep the main script running to keep ROS node alive

if __name__ == '__main__':
    main()
