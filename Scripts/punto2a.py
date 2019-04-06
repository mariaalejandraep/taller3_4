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


distanciaCuadricula=1 #Distancias entre cuadriculas de grafo
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


def creadorMatriz():
    global distanciaCuadricula, Equivalente,numeroCasillas, n, Links, g
    xInic=-5.0+np.divide(float(distanciaCuadricula),2.0)
    yInic=5.0-np.divide(float(distanciaCuadricula),2.0)
    for i in range(0,n*n):
        g.add_node(i)
        mod=np.mod(i,n)#columnas de la matriz
        div=int(np.floor(i/n))#filas de la matriz
        if len(Equivalente)<div+1:
            Equivalente.append([])
        nuevaCasilla = Casilla(xInic+mod*distanciaCuadricula,yInic-div*distanciaCuadricula)
        Equivalente[div].append(nuevaCasilla)




def Graficador_network():
    global contador1,Links, g, n
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
            g.add_edge(div2,contador1)
        else:
            Links[div2].append(0)
    nx.draw(g,with_labels=True)
    plt.show()


def conexion(i,j):#como variables entran dos numeros relacionados con cada casilla
    divi=int(np.math.floor(i/n))
    divj=int(np.math.floor(j/n))
    modi=np.mod(i,n)
    modj=np.mod(j,n)
    dist=np.abs(divi-divj)+np.abs(modi-modj)
    if dist>1:
        return False
    else:
        #Distancias limites a cada obstaculo
        dist_limite_1=((twistInfoPos1.linear.z)/2.0)+math.sqrt(np.power(distanciaCuadricula/2.0,2.0)+np.power(distanciaCuadricula/2.0,2.0)) #Distancia limite a obstaculo 1
        dist_limite_2=((twistInfoPos2.linear.z)/2.0)+math.sqrt(np.power(distanciaCuadricula/2.0,2.0)+np.power(distanciaCuadricula/2.0,2.0)) #Distancia limite a obstaculo 2
        dist_limite_3=((twistInfoPos3.linear.z)/2.0)+math.sqrt(np.power(distanciaCuadricula/2.0,2.0)+np.power(distanciaCuadricula/2.0,2.0)) #Distancia limite a obstaculo 3
        dist_limite_4=((twistInfoPos4.linear.z)/2.0)+math.sqrt(np.power(distanciaCuadricula/2.0,2.0)+np.power(distanciaCuadricula/2.0,2.0)) #Distancia limite a obstaculo 4
        dist_limite_5=((twistInfoPos5.linear.z)/2.0)+math.sqrt(np.power(distanciaCuadricula/2.0,2.0)+np.power(distanciaCuadricula/2.0,2.0)) #Distancia limite a obstaculo 5


        #Distancias a obstaculos de casilla i considerando distancia limite
        disti_obst_1=math.sqrt(np.power(Equivalente[divi][modi].x-twistInfoPos1.linear.x,2.0)+np.power(Equivalente[divi][modi].y-twistInfoPos1.linear.y,2.0))-dist_limite_1 #Diferencia entre limite 1 y distancia de casilla i a obstaculo 1
        disti_obst_2=math.sqrt(np.power(Equivalente[divi][modi].x-twistInfoPos2.linear.x,2.0)+np.power(Equivalente[divi][modi].y-twistInfoPos2.linear.y,2.0))-dist_limite_2 #Diferencia entre limite 2 y distancia de casilla i a obstaculo 2
        disti_obst_3=math.sqrt(np.power(Equivalente[divi][modi].x-twistInfoPos3.linear.x,2.0)+np.power(Equivalente[divi][modi].y-twistInfoPos3.linear.y,2.0))-dist_limite_3 #Diferencia entre limite 3 y distancia de casilla i a obstaculo 3
        disti_obst_4=math.sqrt(np.power(Equivalente[divi][modi].x-twistInfoPos4.linear.x,2.0)+np.power(Equivalente[divi][modi].y-twistInfoPos4.linear.y,2.0))-dist_limite_4 #Diferencia entre limite 4 y distancia de casilla i a obstaculo 4
        disti_obst_5=math.sqrt(np.power(Equivalente[divi][modi].x-twistInfoPos5.linear.x,2.0)+np.power(Equivalente[divi][modi].y-twistInfoPos5.linear.y,2.0))-dist_limite_5 #Diferencia entre limite 5 y distancia de casilla i a obstaculo 5

        #Distancias a obstaculos de casilla j considerando distancia limite
        distj_obst_1=math.sqrt(np.power(Equivalente[divj][modj].x-twistInfoPos1.linear.x,2.0)+np.power(Equivalente[divj][modj].y-twistInfoPos1.linear.y,2.0))-dist_limite_1 #Diferencia entre limite 1 y distancia de casilla j a obstaculo 1
        distj_obst_2=math.sqrt(np.power(Equivalente[divj][modj].x-twistInfoPos2.linear.x,2.0)+np.power(Equivalente[divj][modj].y-twistInfoPos2.linear.y,2.0))-dist_limite_2 #Diferencia entre limite 2 y distancia de casilla j a obstaculo 1
        distj_obst_3=math.sqrt(np.power(Equivalente[divj][modj].x-twistInfoPos3.linear.x,2.0)+np.power(Equivalente[divj][modj].y-twistInfoPos3.linear.y,2.0))-dist_limite_3 #Diferencia entre limite 3 y distancia de casilla j a obstaculo 1
        distj_obst_4=math.sqrt(np.power(Equivalente[divj][modj].x-twistInfoPos4.linear.x,2.0)+np.power(Equivalente[divj][modj].y-twistInfoPos4.linear.y,2.0))-dist_limite_4 #Diferencia entre limite 4 y distancia de casilla j a obstaculo 1
        distj_obst_5=math.sqrt(np.power(Equivalente[divj][modj].x-twistInfoPos5.linear.x,2.0)+np.power(Equivalente[divj][modj].y-twistInfoPos5.linear.y,2.0))-dist_limite_5 #Diferencia entre limite 5 y distancia de casilla j a obstaculo 1

        #Distancias minima a obstaculo
        dist_obst=np.amin([disti_obst_1,disti_obst_2,disti_obst_3,disti_obst_4,disti_obst_5,distj_obst_1,distj_obst_2,distj_obst_3,distj_obst_4,distj_obst_5])


        if (dist_obst)<=(0.0):
            print i,j
            return False

        else:
            return True




if __name__ == '__main__':
    try:
        punto2_a()

    except rospy.ROSInterruptException:
        pass
