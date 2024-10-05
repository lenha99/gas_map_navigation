#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from geometry_msgs.msg import Twist
import os

# 데이터 저장 상태
is_recording = False
file_path = os.path.expanduser("~/cmd_vel_log.txt")

def callback(data):
    global is_recording
    if is_recording:
        with open(file_path, 'a') as f:
            f.write("{}, {}, {}, {}, {}, {}\n".format(data.linear.x, data.linear.y, data.linear.z, 
                                                      data.angular.x, data.angular.y, data.angular.z))

def listener():
    global is_recording
    rospy.init_node('cmd_vel_listener', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, callback)

    print("Press 's' to start recording, 'e' to stop recording, and 'q' to quit.")

    try:
        while not rospy.is_shutdown():
            user_input = raw_input().strip().lower()
            if user_input == 's':
                print("Recording started...")
                is_recording = True
            elif user_input == 'e':
                print("Recording stopped.")
                is_recording = False
            elif user_input == 'q':
                print("Exiting...")
                break
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        rospy.signal_shutdown("User exited.")

if __name__ == '__main__':
    listener()

