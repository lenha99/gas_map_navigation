#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from geometry_msgs.msg import Twist
import os

# 데이터 재생 상태
is_playing = False
file_path = os.path.expanduser("~/cmd_vel_log.txt")

def auto_drive():
    global is_playing
    rospy.init_node('auto_drive_node', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    try:
        while not rospy.is_shutdown():
            user_input = raw_input("Press 'p' to start playback, 's' to stop playback, and 'q' to quit: ").strip().lower()

            if user_input == 'p':
                is_playing = True
                print("Playback started...")

                # 저장된 데이터를 파일에서 읽음
                with open(file_path, 'r') as f:
                    lines = f.readlines()

                for line in lines:
                    if not is_playing:
                        break  # 중지 신호를 받으면 루프를 나감

                    cmd_vel = Twist()
                    linear_x, linear_y, linear_z, angular_x, angular_y, angular_z = map(float, line.split(','))

                    # Twist 메시지에 데이터 입력
                    cmd_vel.linear.x = linear_x
                    cmd_vel.linear.y = linear_y
                    cmd_vel.linear.z = linear_z
                    cmd_vel.angular.x = angular_x
                    cmd_vel.angular.y = angular_y
                    cmd_vel.angular.z = angular_z

                    # cmd_vel 메시지를 발행
                    pub.publish(cmd_vel)
                    rate.sleep()  # 주기적으로 발행

            elif user_input == 's':
                is_playing = False
                print("Playback stopped.")

            elif user_input == 'q':
                print("Exiting...")
                break
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        rospy.signal_shutdown("User exited.")

if __name__ == '__main__':
    auto_drive()

