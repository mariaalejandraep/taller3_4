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
import networkx as nx
import matplotlib.pyplot as plt


#Iniciar grafico de networkx
g=nx.Graph()

#l = math.sqrt(np.power(0.38/2,2)+np.power(0.51/2,2)) #metros
l=0.4/2

#Variables tipo Twist
#Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 1.
twistInfoPos1 = Twist()



#ARRAYS
#Almacena posiciones de coordenadas
Equivalente=[]
#ALmacena conexiones entre nodos
Links=[]



#CONTADORES
#Contador creador de
contador1=-1


distanciaCuadricula=1 #Distancias entre cuadriculas de grafo
n=int(10/distanciaCuadricula) #Acordarse que 10 debe ser divisible por distanciaCuadricula



#En este metodo se inicializa el nodo, se suscribe a los topicos necesarios, se crea la variable
#para publicar al topico de motorsVel y tambien se lanza el nodo encargado de graficar.
def punto2_a():
    global twistInfoDim, twistInfoPos
    rospy.init_node('punto2_a', anonymous=True)
    rospy.Subscriber('InfoObs0', Twist, setObst)
    rospy.Subscriber('pioneerPosition', Twist, setPosAndGeomP) # Se subscribe al topico de posicion del robot
    rate = rospy.Rate(10)
    creadorMatriz()
    Graficador_network()
    while not rospy.is_shutdown():
        rate.sleep()



#FUNCIONES ANCLADAS A TOPICOS SUBSCRITOS
#Info de obstaculo 1
def setObst(posicionObstacle):
    global twistInfoPos1, twistInfoPos2, twistInfoPos3, twistInfoPos4, twistInfoPos5
    if posicionObstacle.angular.x==1:
        twistInfoPos1=posicionObstacle
    elif posicionObstacle.angular.x==2:
        twistInfoPos2=posicionObstacle
    elif posicionObstacle.angular.x==3:
        twistInfoPos3=posicionObstacle
    elif posicionObstacle.angular.x==4:
        twistInfoPos4=posicionObstacle
    elif posicionObstacle.angular.x==5:
        twistInfoPos5=posicionObstacle



if __name__ == '__main__':
    try:
        punto2_a()

    except rospy.ROSInterruptException:
        pass
