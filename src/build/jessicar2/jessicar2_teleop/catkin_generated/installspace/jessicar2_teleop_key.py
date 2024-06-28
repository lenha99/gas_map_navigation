#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

msg = """
Control Your Robot!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (~ 1.2 m/s)
a/d : increase/decrease angular velocity (~ 1.8)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "linear vel:%0.2f\t angular vel:%0.2f" % (target_linear_vel, target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input

    return input

def checkLinearLimitVelocity(vel, min, max):
    vel = constrain(vel, min, max)
    return vel

def checkAngularLimitVelocity(vel, max):
    vel = constrain(vel, -max, max)
    return vel

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('jessicar2_teleop')
    tf_prefix = rospy.get_param("~tf_prefix", "")
    print("tf_prefix:" + tf_prefix)
    max_lin_vel = rospy.get_param("~max_lin_vel")
    min_lin_vel = rospy.get_param("~min_lin_vel")
    max_ang_vel = rospy.get_param("~max_ang_vel")
    lin_vel_step_size = rospy.get_param("~lin_vel_step")
    ang_vel_step_size = rospy.get_param("~ang_vel_step")
    ang_vel_rev = rospy.get_param("~ang_vel_reverse")
    pub = rospy.Publisher(tf_prefix + '/cmd_vel', Twist, queue_size=10)

    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_angular_vel = 0.0

    try:
        print(msg)
        while(1):
            key = getKey()
            if key == 'w':
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + lin_vel_step_size, min_lin_vel, max_lin_vel)
                status += 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'x':
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - lin_vel_step_size, min_lin_vel, max_lin_vel)
                status += 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'a':
                if ang_vel_rev == 1:
                    target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ang_vel_step_size, max_ang_vel)
                else:
                    target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ang_vel_step_size, max_ang_vel)
                status += 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'd':
                if ang_vel_rev == 1:
                    target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ang_vel_step_size, max_ang_vel)
                else:
                    target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ang_vel_step_size, max_ang_vel)
                status += 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == ' ' or key == 's':
                target_linear_vel = 0.0
                control_linear_vel = 0.0
                target_angular_vel = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))
            else:
                if key == '\x03':  # CTRL-C to quit
                    break

            if status == 20:
                print(msg)
                status = 0

            twist = Twist()
            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (lin_vel_step_size / 2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ang_vel_step_size / 2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
            pub.publish(twist)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

