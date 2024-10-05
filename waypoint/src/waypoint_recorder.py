#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
import time
import os
import sys
import select

waypoints = []

def record_waypoint():
    global waypoints
    try:
        (trans, rot) = tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        _, _, yaw = tf.transformations.euler_from_quaternion(rot)

        if len(waypoints) == 0 or time.time() - waypoints[-1]['time'] >= 2:
            waypoints.append({'x': trans[0], 'y': trans[1], 'yaw': yaw, 'time': time.time()})
            rospy.loginfo("Waypoint recorded: x={:.2f}, y={:.2f}, yaw={:.2f}".format(trans[0], trans[1], yaw))

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Failed to get transform between /map and /base_footprint")

def save_waypoints():
    with open('/home/ubuntu/catkin_ws/data/waypoints.txt', 'w') as file:
        for waypoint in waypoints:
            file.write("{:.2f} {:.2f} {:.2f}\n".format(waypoint['x'], waypoint['y'], waypoint['yaw']))
    rospy.loginfo("Waypoints saved to /home/ubuntu/catkin_ws/data/waypoints.txt")

def save_pbstream():
    try:
        rospy.wait_for_service('/write_state', timeout=5)
        rospy.loginfo("Saving Cartographer state to pbstream file...")
        os.system("rosservice call /write_state \"{filename: '/home/ubuntu/catkin_ws/src/maps/my_map.pbstream'}\"")
        rospy.loginfo("Cartographer state saved to /home/ubuntu/catkin_ws/src/maps/my_map.pbstream")
    except rospy.ROSException as e:
        rospy.logerr("Service /write_state is not available: {}".format(e))
    except Exception as e:
        rospy.logerr("Failed to save Cartographer state: {}".format(e))

def shutdown_hook():
    rospy.loginfo("Shutting down, saving waypoints...")
    save_waypoints()
    rospy.loginfo("Shutdown complete.")

def check_for_save_command():
    # 터미널 입력을 비동기로 처리하여 사용자가 's'를 입력하면 pbstream을 저장합니다.
    i, _, _ = select.select([sys.stdin], [], [], 0)
    if i:
        input_char = sys.stdin.read(1)
        if input_char == 's':
            rospy.loginfo("Saving pbstream file due to 's' command...")
            save_pbstream()

if __name__ == '__main__':
    rospy.init_node('waypoint_recorder')

    tf_listener = tf.TransformListener()

    rate = rospy.Rate(1)  # 1 Hz

    rospy.on_shutdown(shutdown_hook)

    while not rospy.is_shutdown():
        record_waypoint()
        check_for_save_command()  # 사용자의 입력을 체크하여 pbstream 저장
        rate.sleep()

