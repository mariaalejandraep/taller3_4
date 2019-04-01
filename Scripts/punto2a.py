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




#Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 1.
twistInfoPos = Twist()
#Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 2.
twistInfoPos2 = Twist()
#Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 3.
twistInfoPos3 = Twist()
#Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 4.
twistInfoPos4 = Twist()
#Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 5.
twistInfoPos5 = Twist()
#Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 5.
twistInfoPioneer = Twist()

#Almacena posiciones de coordenadas i
Equivalente=None

#Contador que inicializa funcion creadora de matriz
yeta=0
#Contador creador de

distanciaCuadricula=1
n=10/distanciaCuadricula #Acordarse que 10 debe ser divisible por distanciaCuadricula

class Casilla:
    def __init__(self, xP, yP):
        self.x=float(xP)
        self.y=float(yP)



#En este metodo se inicializa el nodo, se suscribe a los topicos necesarios, se crea la variable
#para publicar al topico de motorsVel y tambien se lanza el nodo encargado de graficar.
def punto2_a():
    global twistInfoDim, twistInfoPos
    rospy.init_node('punto2_a', anonymous=True)
    rospy.Subscriber('InfoObs0', Twist, setObst)
    rospy.Subscriber('InfoObs1', Twist, setObst1)
    rospy.Subscriber('InfoObs2', Twist, setObst2)
    rospy.Subscriber('InfoObs3', Twist, setObst3)
    rospy.Subscriber('InfoObs4', Twist, setObst4)
    rospy.Subscriber('pioneerPosition', Twist, setPosAndGeomP) # Se subscribe al topico de posicion del robot
    rate = rospy.Rate(10)
    creadorMatriz()
    while not rospy.is_shutdown():
        rate.sleep()

#Info de obstaculo 1
def setObst(posicionObstacle):
    global twistInfoPos
    twistInfoPos=posicionObstacle

#Info de obstaculo 2
def setObst1(posicionObstacle):
    global twistInfoPos2
    twistInfoPos2=posicionObstacle


#Info de obstaculo 3
def setObst2(posicionObstacle):
    global twistInfoPos3
    twistInfoPos3=posicionObstacle
#

#Info de obstaculo 4
def setObst3(posicionObstacle):
    global twistInfoPos4
    twistInfoPos4=posicionObstacle


#Info de obstaculo 5
def setObst4(posicionObstacle):
    global twistInfoPos5
    twistInfoPos5=posicionObstacle

#Info de pioneer
def setPosAndGeomP(posyGeo):
    global twistInfoPioneer, yeta
    twistInfoPioneer=posyGeo


def creadorMatriz():
    global distanciaCuadricula, Equivalente, contador1,numeroCasillas, n
    Equivalente=[]
    xInic=-5.0+np.divide(float(distanciaCuadricula),2.0)
    yInic=5.0-np.divide(float(distanciaCuadricula),2.0)
    for i in range(0,n*n):
        mod=np.mod(i,n)
        div=int(np.floor(i/n))
        if len(Equivalente)<div+1:
            Equivalente.append([])
        nuevaCasilla = Casilla(xInic+mod*distanciaCuadricula,yInic-div*distanciaCuadricula)
        Equivalente[div].append(nuevaCasilla)
    print(Equivalente[9][9].x,Equivalente[9][9].y)

def conexion(i,j):
    global n
    divi=int(np.floor(i,n))
    divj=int(np.floor(j,n))
    modi=np.mod(i,n)
    modj=np.mod(j,n)
    dist=np.abs(divi-divj)+np.abs(modi-modj)
    if dist>1:
        return False
    else:
        return True





if __name__ == '__main__':
    try:
        punto2_a()

    except rospy.ROSInterruptException:
        pass
