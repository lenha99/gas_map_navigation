#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import sys
import select
import tty
import termios

waypoints = []
obstacle_detected = False
manual_stop = False

def load_waypoints():
    global waypoints
    with open('/home/your_username/waypoints.txt', 'r') as file:
        for line in file:
            x, y, yaw = map(float, line.strip().split())
            waypoints.append({'x': x, 'y': y, 'yaw': yaw})
    rospy.loginfo("Waypoints loaded:")
    for wp in waypoints:
        rospy.loginfo("x={}, y={}, yaw={}".format(wp['x'], wp['y'], wp['yaw']))

def publish_waypoints_as_markers():
    marker_pub = rospy.Publisher('/waypoint_markers', MarkerArray, queue_size=10)
    marker_array = MarkerArray()

    for i, waypoint in enumerate(waypoints):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "waypoints"
        marker.id = i
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose.position.x = waypoint['x']
        marker.pose.position.y = waypoint['y']
        marker.pose.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, waypoint['yaw'])
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]

        marker.scale.x = 0.5  # 화살표 길이
        marker.scale.y = 0.1  # 화살표 너비
        marker.scale.z = 0.1  # 화살표 높이

        marker.color.a = 1.0  # 투명도
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker_array.markers.append(marker)

    marker_pub.publish(marker_array)
    rospy.loginfo("Published waypoint markers in RViz.")

def laser_callback(msg):
    global obstacle_detected
    angle_min = 30  # 60도 범위의 시작 (각도)
    angle_max = 330  # 60도 범위의 끝 (각도)
    
    ranges = msg.ranges
    front_ranges = ranges[int(len(ranges)*angle_min/360):int(len(ranges)*angle_max/360)]
    min_distance = min(front_ranges)
    if min_distance < 0.3:  # 장애물 감지 거리 기준 (예: 0.3미터)
        obstacle_detected = True
    else:
        obstacle_detected = False

def navigate_to_waypoints():
    global obstacle_detected, manual_stop
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    for waypoint in waypoints:
        rospy.loginfo("Navigating to waypoint: x={}, y={}, yaw={}".format(waypoint['x'], waypoint['y'], waypoint['yaw']))
        while not rospy.is_shutdown():
            if manual_stop:
                rospy.loginfo("Manual stop activated, stopping the robot.")
                stop_robot(cmd_vel_pub)
                return

            if obstacle_detected:
                rospy.loginfo("Obstacle detected, stopping the robot.")
                stop_robot(cmd_vel_pub)
                rospy.sleep(1)  # 장애물이 제거될 때까지 대기
                continue

            current_position, current_yaw = get_current_position_orientation()
            distance = math.sqrt((waypoint['x'] - current_position[0])**2 + (waypoint['y'] - current_position[1])**2)
            angle_to_goal = math.atan2(waypoint['y'] - current_position[1], waypoint['x'] - current_position[0])

            if distance < 0.1:
                rospy.loginfo("Reached waypoint")
                break

            move_cmd = Twist()
            move_cmd.linear.x = 0.1  # 고정된 직진 속도
            move_cmd.angular.z = 1.0 * (angle_to_goal - current_yaw)
            cmd_vel_pub.publish(move_cmd)

            rate.sleep()

    rospy.loginfo("All waypoints reached.")
    stop_robot(cmd_vel_pub)

def stop_robot(cmd_vel_pub):
    stop_cmd = Twist()
    cmd_vel_pub.publish(stop_cmd)

def get_current_position_orientation():
    try:
        (trans, rot) = tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        current_position = (trans[0], trans[1])
        current_yaw = tf.transformations.euler_from_quaternion(rot)[2]
        return current_position, current_yaw
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return (0, 0), 0

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('waypoint_follower')

    tf_listener = tf.TransformListener()

    load_waypoints()

    rospy.sleep(1)

    publish_waypoints_as_markers()  # RViz에서 경로점 표시

    rospy.Subscriber('/scan', LaserScan, laser_callback)

    while not rospy.is_shutdown():
        key = get_key()
        if key == 's':
            rospy.loginfo("Stop key pressed!")
            manual_stop = True
            stop_robot(rospy.Publisher('/cmd_vel', Twist, queue_size=10))
            break

        navigate_to_waypoints()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

