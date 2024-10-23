#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

def move_robot():
    rospy.init_node('move_robot', anonymous=True)

    # Publicadores para as rodas traseiras (velocidade)
    rear_left_wheel_pub = rospy.Publisher('/ackermann_robot/rear_left_wheel_joint_velocity_controller/command', Float64, queue_size=10)
    rear_right_wheel_pub = rospy.Publisher('/ackermann_robot/rear_right_wheel_joint_velocity_controller/command', Float64, queue_size=10)

    # Publicador para a direção (ângulo da roda dianteira)
    front_steer_pub = rospy.Publisher('/ackermann_robot/front_left_wheel_steer_joint_position_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        # Publicar uma velocidade constante nas rodas traseiras
        rear_left_wheel_pub.publish(1.0)
        rear_right_wheel_pub.publish(1.0)

        # Publicar um ângulo de direção
        front_steer_pub.publish(0.2)

        rate.sleep()

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
