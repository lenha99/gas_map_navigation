#include <ros.h>
#include <geometry_msgs/Twist.h>

// 상수 정의
#define WHEEL_BASE 0.5
#define K_P 1.0
#define K_b 0.1
#define K_bias 0.05
#define PWM_MIN 10

// PWM 핀 정의
#define ENA_PIN 12
#define ENB_PIN 13

// ROS 핸들러
ros::NodeHandle nh;

// 전역 변수
int pwmLeftReq = 0;
int pwmRightReq = 0;

// PWM 값을 계산하는 함수
void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
    float vLeft = cmdVel.linear.x - cmdVel.angular.z * WHEEL_BASE / 2.0;
    float vRight = cmdVel.linear.x + cmdVel.angular.z * WHEEL_BASE / 2.0;

    if (vLeft > 0) {
        pwmLeftReq = int(K_P * vLeft + K_b + K_bias);
    } else {
        pwmLeftReq = int(K_P * vLeft - K_b - K_bias);
    }

    if (vRight > 0) {
        pwmRightReq = K_P * vRight + K_b;
    } else {
        pwmRightReq = K_P * vRight - K_b;
    }

    if (abs(pwmLeftReq) < PWM_MIN) {
        pwmLeftReq = 0;
    }

    if (abs(pwmRightReq) < PWM_MIN) {
        pwmRightReq = 0;
    }
}

// PWM 값을 설정하는 함수
void set_pwm_values() {
    if (pwmLeftReq > 0) {
        analogWrite(ENA_PIN, pwmLeftReq);
    } else {
        analogWrite(ENA_PIN, -pwmLeftReq);
    }

    if (pwmRightReq > 0) {
        analogWrite(ENB_PIN, pwmRightReq);
    } else {
        analogWrite(ENB_PIN, -pwmRightReq);
    }
}

// ROS 콜백 함수
void cmdVelCallback(const geometry_msgs::Twist& msg) {
    calc_pwm_values(msg);
    set_pwm_values();
}

// ROS 구독자 정의
ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", cmdVelCallback);

void setup() {
    nh.initNode();
    nh.subscribe(cmdVelSub);

    // PWM 핀을 출력으로 설정
    pinMode(ENA_PIN, OUTPUT);
    pinMode(ENB_PIN, OUTPUT);
}

void loop() {
    nh.spinOnce();
    delay(1);
}
