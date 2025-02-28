#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Quaternion
from math import cos, sin

class SkidSteerOdometry:
    def __init__(self):
        rospy.init_node("skid_steer_odometry", anonymous=True)

        # Subscrição ao tópico de velocidades das rodas
        rospy.Subscriber("/wheel_speeds", Float32MultiArray, self.wheel_callback)

        # Publicador da odometria
        self.odom_pub = rospy.Publisher("/wheel_odom", Odometry, queue_size=10)

        # Publicador de transformação TF (odom -> base_link)
        self.odom_broadcaster = tf.TransformBroadcaster()

        # Parâmetros do robô
        self.wheel_radius = 0.1   # Raio da roda (m)
        self.wheel_base = 0.5     # Distância entre rodas esquerda e direita (m)

        # Estado da odometria
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = rospy.Time.now()

        rospy.spin()

    def wheel_callback(self, msg):
        if len(msg.data) != 4:
            rospy.logwarn("Dados de velocidade das rodas inválidos!")
            return

        left_front, left_rear, right_front, right_rear = msg.data

        # Média das velocidades das rodas de cada lado (convertendo de radianos para metros/s)
        v_left = (left_front + left_rear) / 2.0 * self.wheel_radius
        v_right = (right_front + right_rear) / 2.0 * self.wheel_radius

        # Velocidade linear e angular do robô
        v = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / self.wheel_base

        # Atualização da odometria
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        delta_x = v * dt * cos(self.theta)
        delta_y = v * dt * sin(self.theta)
        delta_theta = omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Criando mensagem de odometria
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Posição
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, self.theta))

        # Velocidade
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        # Covariância
        # **Adicionando covariâncias fixas**
        odom.pose.covariance = [0.01, 0,    0,    0,   0,   0,   # x
                                0,    0.01, 0,    0,   0,   0,   # y
                                0,    0,    0.05, 0,   0,   0,   # z
                                0,    0,    0,    0.1, 0,   0,   # roll
                                0,    0,    0,    0,   0.1, 0,   # pitch
                                0,    0,    0,    0,   0,   0.2] # yaw

        odom.twist.covariance = [0.01, 0,    0,    0,   0,   0,  # v_x
                                 0,    0.01, 0,    0,   0,   0,  # v_y
                                 0,    0,    0.05, 0,   0,   0,  # v_z
                                 0,    0,    0,    10,  0,   0,  # omega_x
                                 0,    0,    0,    0,   10,  0,  # omega_y
                                 0,    0,    0,    0,   0,   10] # omega_z

        # Publicar a odometria
        self.odom_pub.publish(odom)

        # Publicar transformação TF
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
            tf.transformations.quaternion_from_euler(0, 0, self.theta),
            current_time,
            "base_link",
            "odom"
        )

if __name__ == "__main__":
    try:
        SkidSteerOdometry()
    except rospy.ROSInterruptException:
        pass
