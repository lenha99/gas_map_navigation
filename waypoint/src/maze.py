#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# 설정된 벽과의 목표 거리 (단위: 미터)
desired_distance_from_wall = 0.5
# P 제어기의 계수 (Kp)
kp = 0.5

# 슬라이딩 윈도우 필터 함수
def sliding_window_filter(data, window_size=5):
    return [sum(data[i:i+window_size])/window_size for i in range(len(data)-window_size+1)]

def callback(scan):
    # LiDAR 데이터 처리 (슬라이딩 윈도우 필터 적용)
    front_data = sliding_window_filter(scan.ranges[350:360] + scan.ranges[0:10])
    left_data = sliding_window_filter(scan.ranges[55:125])  # 오버랩 범위 적용
    right_data = sliding_window_filter(scan.ranges[235:305])  # 오버랩 범위 적용

    front_min = min(min(front_data), 10)
    left_min = min(left_data)
    right_min = min(right_data)

    # Twist 메시지 객체 생성
    vel_msg = Twist()

    # 코너 및 벽 감지 및 회피 로직
    if front_min < desired_distance_from_wall:  # 앞에 장애물이 있으면 회피
        vel_msg.linear.x = 0.0
        if left_min > right_min:
            vel_msg.angular.z = 0.3  # 왼쪽이 더 넓으므로 왼쪽으로 회전
        else:
            vel_msg.angular.z = -0.3  # 오른쪽이 더 넓으므로 오른쪽으로 회전
    else:
        # 벽 따라가기 로직
        vel_msg.linear.x = 0.1  # 직진 속도 조정
        if left_min < desired_distance_from_wall:  # 왼쪽에 벽이 가까울 때
            error = desired_distance_from_wall - left_min
            vel_msg.angular.z = -kp * error  # 오른쪽으로 회전하면서 벽에서 멀어짐
        elif right_min < desired_distance_from_wall:  # 오른쪽에 벽이 가까울 때
            error = desired_distance_from_wall - right_min
            vel_msg.angular.z = kp * error  # 왼쪽으로 회전하면서 벽에서 멀어짐
        else:
            vel_msg.angular.z = 0.0  # 회전 없이 직진

    # cmd_vel 토픽으로 속도 명령 전송
    velocity_publisher.publish(vel_msg)

def shutdown_hook():
    # 노드 종료 시 로봇을 멈추도록 속도를 0으로 설정
    rospy.loginfo("Shutting down. Stopping the robot...")
    stop_msg = Twist()
    stop_msg.linear.x = 0.0
    stop_msg.angular.z = 0.0
    velocity_publisher.publish(stop_msg)

def main():
    rospy.init_node('maze_escape_robot', anonymous=True)
    
    # cmd_vel 퍼블리셔 생성
    global velocity_publisher
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # LiDAR 데이터 구독
    rospy.Subscriber('/scan', LaserScan, callback)

    # 노드 종료 시 호출될 함수를 등록
    rospy.on_shutdown(shutdown_hook)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

