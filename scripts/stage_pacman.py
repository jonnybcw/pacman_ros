#!/usr/bin/env python
# -*- coding: utf-8 -*-

######################
# Trabalho final
# Aula 13/07/2022
######################

import math
import os
import sys
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import kbhit
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose

pose_pacman = Pose()
pose_ghost = Pose()

ranges = []
msg_pacman = Twist()


def pacmanCallback(data):
    pose_pacman.x = data.pose.pose.position.x # posicao x do robo no mundo
    pose_pacman.y = data.pose.pose.position.y # posicao y do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion ([x_q , y_q , z_q , w_q ])
    pose_pacman.theta = euler [2] # orientacao theta do robo no mundo


def ghostCallback(data):
    pose_ghost.x = data.pose.pose.position.x # posicao x do robo no mundo
    pose_ghost.y = data.pose.pose.position.y # posicao y do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion ([x_q , y_q , z_q , w_q ])
    pose_ghost.theta = euler [2] # orientacao theta do robo no mundo


def baseScanCallback(data):
    global ranges
    ranges = data.ranges
    # rospy.loginfo('Ranges>>%s' % str(ranges))
    

def main(robo):
    rospy.init_node('stage_pacman', anonymous=True)
    pub = rospy.Publisher('/robot_%s/cmd_vel' % robo, Twist, queue_size=10)
    if robo != 0:
        sub_pacman = rospy.Subscriber('/robot_0/base_pose_ground_truth', Odometry, callback=pacmanCallback)
        sub_ghost = rospy.Subscriber('/robot_%s/base_pose_ground_truth' % robo, Odometry, callback=ghostCallback)
    sub = rospy.Subscriber('/robot_%s/base_scan' % robo, LaserScan, callback=baseScanCallback)

    rate = rospy.Rate(10)
        
    if robo == '0':
        kbhit.init()
        print('Pressione uma tecla para guiar o pacman:'
        '\nW - frente\nA - esquerda\nS - ré\nD - direita\nR - respawn\nQ - parar')
        while True:
            ch = 'None'
            if kbhit.kbhit():
                ch = kbhit.getch()
            comandarPacman(pub, rate, ch)
    else:
        comandarBot(pub, rate, robo)

        
def comandarPacman(pub, rate, ch):
    global msg_pacman
    if len(ranges) > 0:
        right_side = ranges[0:29]
        center = ranges[30:59]
        left_side = ranges[60:89]

        min_right = min(right_side)
        min_center = min(center)
        min_left = min(left_side)
        # rospy.loginfo('Min right>>%.2f\nMin center>>%.2f\nMin left>>%.2f\n\n' % (min_right, min_center, min_left))

        if min_right < 0.2 or min_center < 0.2 or min_right < 0.2:
            rospy.loginfo('GAME OVER!')
            msg_pacman.linear.x = 0
            msg_pacman.angular.z = 0
            os.system('rosservice call /reset_positions')
            return

    msg_pacman.angular.z = 0
    x_before = None

    if ch.upper() == 'W':
        msg_pacman.linear.x = 1
        msg_pacman.angular.z = 0
    elif ch.upper() == 'S':
        msg_pacman.linear.x = -1  
        msg_pacman.angular.z = 0
    elif ch.upper() == 'A':
        x_before = msg_pacman.linear.x
        for i in range(15):
            msg_pacman.linear.x = 0.1
            msg_pacman.angular.z = 1
            pub.publish(msg_pacman)
            rate.sleep()
    elif ch.upper() == 'D':
        x_before = msg_pacman.linear.x
        for i in range(15):
            msg_pacman.linear.x = 0.1
            msg_pacman.angular.z = -1
            pub.publish(msg_pacman)
            rate.sleep()
    elif ch.upper() == 'R':
        msg_pacman.linear.x = 0
        msg_pacman.angular.z = 0
        os.system('rosservice call /reset_positions')
    elif ch.upper() == 'Q':
        msg_pacman.linear.x = 0
        msg_pacman.angular.z = 0

        
    
    pub.publish(msg_pacman)
    if x_before is not None:
        msg_pacman.linear.x = x_before
    rate.sleep()

kpos = 0.75
ki = 0.000001
kd = 0.000001

korient = 10

erro = 99
erroInt = 0
erroOrient = 0
lastErro = 0
last_time = None


def comandarBot(pub, rate, robo):
    msg = Twist()
    tolerancePos = 0.05
    toleranceOrient = 0.005

    while abs(erroOrient) > toleranceOrient or abs(erro) > tolerancePos:
        posdesejada = [pose_pacman.x, pose_pacman.y]
        if len(ranges) > 0:
            right_side = ranges[0:20]
            center = ranges[30:59]
            left_side = ranges[60:89]

            min_right = min(right_side)
            min_center = min(center)
            min_left = min(left_side)
            #rospy.loginfo('Min right>>%.2f\nMin center>>%.2f\nMin left>>%.2f\n\n' % (min_right, min_center, min_left))

            msg.linear.x = 0.5
            msg.angular.z = 0

            if min_center < 0.7:
                msg.linear.x = 0.1
                msg.angular.z = 1 if robo == '1' or robo == '2' else -1
            elif min_left < 0.5:
                msg.linear.x = 0.1
                msg.angular.z = -1
            elif min_right < 0.5:
                msg.linear.x = 0.1
                msg.angular.z = 1
            else:
                pid(msg, last_time, posdesejada)
        pub.publish(msg)
        rate.sleep()


def pid(msg, last_time, posdesejada):
    global erro
    global erroInt
    global erroOrient
    global lastErro
    if last_time is None:
        last_time = rospy.get_time()
    # variação no tempo
    actual_time = rospy.get_time() # in seconds
    dt = actual_time - last_time
    if dt == 0:
        dt = 1

    lastErro  = erro 

    # erro = distancia entre dois pontos
    erro = math.sqrt(math.pow(posdesejada[0] - pose_ghost.x, 2) + math.pow(posdesejada[1]-pose_ghost.y,2))
    erroInt = erroInt + (erro * dt)
    erroDer = (erro - lastErro) / dt

    # calcula a saída PID
    msg.linear.x = (kpos * erro + ki * erroInt + kd * erroDer) / 2

    erroOrient = math.atan2(posdesejada[1] - pose_ghost.y, posdesejada[0] - pose_ghost.x) - pose_ghost.theta
    msg.angular.z = korient * erroOrient
    #rospy.loginfo('Theta>>%f,Erro>>%f' % (pose_ghost.theta, erroOrient))
    #rospy.loginfo('X>>%f,Erro>>%f' % (pose_ghost.x, erro))


if __name__ == '__main__':
    try:
        if len(sys.argv) == 2:
            main(sys.argv[1])
        else:
            print('\nVocê deve informar qual robô você deseja controlar, seguindo este padrão:\n\n' +
             '   rosrun pacman_ros stage_pacman.py [NÚM. DO ROBÔ]\n')
    except rospy.ROSInterruptException:
        pass
    kbhit.restore()