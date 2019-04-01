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



#Variables tipo Twist
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



#ARRAYS
#Almacena posiciones de coordenadas
Equivalente=[]
#ALmacena conexiones entre nodos
Links=[]



#CONTADORES
#Contador creador de
contador1=-1


distanciaCuadricula=0.5#Distancias entre cuadriculas de grafo
n=int(10/distanciaCuadricula) #Acordarse que 10 debe ser divisible por distanciaCuadricula

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
    global distanciaCuadricula, Equivalente, contador1,numeroCasillas, n, Links, mirar
    xInic=-5.0+np.divide(float(distanciaCuadricula),2.0)
    yInic=5.0-np.divide(float(distanciaCuadricula),2.0)
    for i in range(0,n*n):
        mod=np.mod(i,n)#columnas de la matriz
        div=int(np.floor(i/n))#filas de la matriz
        if len(Equivalente)<div+1:
            Equivalente.append([])
        nuevaCasilla = Casilla(xInic+mod*distanciaCuadricula,yInic-div*distanciaCuadricula)
        Equivalente[div].append(nuevaCasilla)

    for h in range(0,n*n*n*n):

        if contador1<n*n-1:
            contador1=contador1+1
            div2=int(np.floor((h)/(n*n)))
        else:
            contador1=0
            div2=int(np.floor((h+1)/(n*n)))


        if len(Links)<div2+1:
            Links.append([])

        if conexion(div2,contador1)==True:
            Links[div2].append(1)
        else:
            Links[div2].append(0)



def conexion(i,j):
    divi=int(np.math.floor(i/n))
    divj=int(np.math.floor(j/n))
    modi=np.mod(i,n)
    modj=np.mod(j,n)
    dist=np.abs(divi-divj)+np.abs(modi-modj)
    if dist>1:
        return False
    else:
        disti_obst_0=math.sqrt(np.power(Equivalente[divi][modi].x-twistInfoPos.linear.x,2)+np.power(Equivalente[divi][modi].y-twistInfoPos.linear.y,2))

        if (disti_obst_0)<(((twistInfoPos.linear.z)/2)+math.sqrt(np.power(distanciaCuadricula/2,2)+np.power(distanciaCuadricula/2,2))):
            print "entro mk"
            return False

        else:
            return True







if __name__ == '__main__':
    try:
        punto2_a()

    except rospy.ROSInterruptException:
        pass
