#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Float32MultiArray

class WheelVelocityPublisher:
    def __init__(self):
        rospy.init_node("wheel_velocity_publisher", anonymous=True)

        # Subscrição ao tópico do Gazebo que contém estados dos links
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.link_callback)

        # Publicador das velocidades das rodas
        self.pub = rospy.Publisher("/wheel_speeds", Float32MultiArray, queue_size=10)

        # Nomes dos links das rodas no Gazebo (ajuste conforme seu modelo URDF/SDF)
        self.wheel_links = ["skid_steer_mid360::front_left_wheel", 
                            "skid_steer_mid360::front_right_wheel",
                            "skid_steer_mid360::rear_left_wheel",
                            "skid_steer_mid360::rear_right_wheel"]

        self.wheel_speeds = []

        #rospy.spin()


    def run(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if len(self.wheel_speeds) == 4:
                speed_msg = Float32MultiArray(data=self.wheel_speeds)
                self.pub.publish(speed_msg)

            rate.sleep()


    def link_callback(self, msg):
        self.wheel_speeds = []

        for wheel in self.wheel_links:
            if wheel in msg.name:
                index = msg.name.index(wheel)
                angular_velocity_y = -msg.twist[index].angular.y  # Eixo Y é o de rotação da roda
                #if wheel == "skid_steer_mid360::front_right_wheel" or wheel == "skid_steer_mid360::rear_right_wheel":
                #    wheel_speeds.append(-angular_velocity_y)
                #else:
                self.wheel_speeds.append(angular_velocity_y)

if __name__ == "__main__":
    try:
        wvp = WheelVelocityPublisher()
        wvp.run()
    except rospy.ROSInterruptException:
        pass
