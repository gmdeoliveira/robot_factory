#!/usr/bin/env python3

import csv
import rospy
from nav_msgs.msg import Odometry

# Inicializa os arquivos CSV p/ salvar os dados 
file_fast_lio = open('odometria_fast_lio2_5m.csv', mode='w')
file_husky= open('odometria_husky.csv_5m.csv', mode='w')

# Cria os escritores nos arquivos   
writer_fast_lio = csv.writer(file_fast_lio)
writer_husky = csv.writer(file_husky)

# Escreve o cabeçalho nos arquivos CSV
writer_fast_lio.writerow(['time', 'x', 'y', 'z'])
writer_husky.writerow(['time', 'x', 'y', 'z'])

def odom_callback_fast_lio(msg):
    """Callback para a odometria gerada pelo algoritmo Fast-LIO 2"""
    timestamp = msg.header.stamp.to_sec()
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    
    # Salva os dados no arquivo CSV
    writer_fast_lio.writerow([timestamp,x,y,z])

def odom_callback_t265(msg):
    """Callback para a odometria de rodas do robô Husky"""
    timestamp = msg.header.stamp.to_sec()
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    
    # Salva os dados no arquivo CSV
    writer_husky.writerow([timestamp,x,y,z])

def save_odometries_to_csv():
    """Função para salvar as odometrias em um arquivo CSV"""
    rospy.init_node('odometry_saver', anonymous=True)

    # Subscrição às odometrias
    rospy.Subscriber('/Odometry', Odometry, odom_callback_fast_lio)
    rospy.Subscriber('/husky_velocity_controller/odom', Odometry, odom_callback_t265)

    rospy.spin()  # Mantém o nó ativo até ser interrompido


if __name__ == '__main__':
    try:
        save_odometries_to_csv()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Fecha os arquivos quando o nó for encerrado
        file_fast_lio.close()
        file_husky.close()