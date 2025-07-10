#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist

class TurtlebotMovement():
    def __init__(self):
        rospy.init_node('TurtlebotMovement', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        rospy.sleep(1)  # Espera a que se conecte el publisher

        self.move_straight(duration=1, speed=0.2)
        self.stop_robot()

        self.rotate_90_degrees(direction='right')
        self.rotate_90_degrees(direction='left')

        self.move_straight(duration=1, speed=0.2)
        self.stop_robot()

        self.move_straight(duration=2, speed=-0.2)  # Retrocede
        self.stop_robot()

    def move_straight(self, duration, speed=0.2):
        move_cmd = Twist()
        move_cmd.linear.x = speed
        move_cmd.angular.z = 0

        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(10)
        while rospy.Time.now() < end_time:
            self.cmd_vel.publish(move_cmd)
            rate.sleep()

    def rotate_90_degrees(self, direction='right'):
        rotate_cmd = Twist()
        rotate_cmd.linear.x = 0
        angular_speed = 1.57  # aprox. 90 grados/seg en rad/s

        rotate_cmd.angular.z = -angular_speed if direction == 'right' else angular_speed

        duration = 1.5  # 90 grados a 1.57 rad/s ≈ 1 segundo
        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(10)
        while rospy.Time.now() < end_time:
            self.cmd_vel.publish(rotate_cmd)
            rate.sleep()
        self.stop_robot()

    def stop_robot(self):
        stop_cmd = Twist()
        self.cmd_vel.publish(stop_cmd)
        rospy.sleep(1)
        
    def shutdown(self):
        rospy.loginfo("Deteniendo el TurtleBot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        TurtlebotMovement()
    except rospy.ROSInterruptException:
        pass
