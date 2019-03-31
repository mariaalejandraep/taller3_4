#!/usr/bin/env python
#Se importan las librerias necesarias junto con los mensajes a utilizar
import rospy
import threading
import math
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import os
import sys
import matplotlib.pyplot as plt
import roslaunch




#Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 1.
twistInfoPos = Twist()
#Es la variable en donde se almacena las dimensiones y orientacion actual del obstaculo 1.
twistInfoDim = Twist()
#Es la variable en donde se almacena las dimensiones y orientacion actual del obstaculo 1.
twistInfo = Twist()


#En este metodo se inicializa el nodo, se suscribe a los topicos necesarios, se crea la variable
#para publicar al topico de motorsVel y tambien se lanza el nodo encargado de graficar.
def punto2_a():
    global twistInfoDim, twistInfoPos
    rospy.init_node('punto2_a', anonymous=True)
    rospy.Subscriber('obsPos', Twist, setObst)
    rospy.Subscriber('obsRad', Twist, setObsRad)
    rospy.Subscriber('obsPos', Twist, setObst1)
    rospy.Subscriber('obsRad', Twist, setObsRad1)
    rospy.Subscriber('obsPos', Twist, setObst2)
    rospy.Subscriber('obsRad', Twist, setObsRad2)
    rospy.Subscriber('obsPos', Twist, setObst3)
    rospy.Subscriber('obsRad', Twist, setObsRad3)
    rospy.Subscriber('obsPos', Twist, setObst4)
    rospy.Subscriber('obsRad', Twist, setObsRad4)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

#Posion de obstaculo 1
def setObst(posicionObstacle):
    global twistInfoPos
    twistInfoPos=posicionObstacle
#Radio de obstaculo 1
def setObsRad(dimObstacle):
    global twistInfoDim, radio
    twistInfoDim=dimObstacle
    radio=twistInfoDim.linear.x/2

#Posion de obstaculo 2
def setObst1(posicionObstacle):
    global twistInfoPos2
    twistInfoPos2=posicionObstacle
#Radio de obstaculo 2
def setObsRad1(dimObstacle):
    global twistInfoDim2, radio2
    twistInfoDim2=dimObstacle
    radio2=twistInfoDim2.linear.x/2

#Posion de obstaculo 3
def setObst2(posicionObstacle):
    global twistInfoPos3
    twistInfoPos3=posicionObstacle
#Radio de obstaculo 3
def setObsRad2(dimObstacle):
    global twistInfoDim3, radio3
    twistInfoDim3=dimObstacle
    radio3=twistInfoDim3.linear.x/2

#Posion de obstaculo 4
def setObst3(posicionObstacle):
    global twistInfoPos3
    twistInfoPos3=posicionObstacle
#Radio de obstaculo 4
def setObsRad3(dimObstacle):
    global twistInfoDim4, radio4
    twistInfoDim4=dimObstacle
    radio4=twistInfoDim4.linear.x/2

#Posion de obstaculo 5
def setObst4(posicionObstacle):
    global twistInfoPos5
    twistInfoPos5=posicionObstacle
#Radio de obstaculo 5
def setObsRad4(dimObstacle):
    global twistInfoDim5, radio5
    twistInfoDim5=dimObstacle
    radio5=twistInfoDim5.linear.x/2


if __name__ == '__main__':
    try:
        punto2_a()
    except rospy.ROSInterruptException:
        pass
