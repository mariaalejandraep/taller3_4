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
import time


# Iniciar grafico de networkx
g=nx.Graph()
# Variables tipo Twist
# Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 1.
twistInfoPos1 = Twist()
# Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 2.
twistInfoPos2 = Twist()
# Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 3.
twistInfoPos3 = Twist()
# Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 4.
twistInfoPos4 = Twist()
# Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 5.
twistInfoPos5 = Twist()
# Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 5.
twistInfoPioneer = Twist()
# ARRAYS
# Almacena posiciones de coordenadas
Equivalente=[]
# Distancia entre centro de cuadriculas, 10 debe ser divisible por esta distancia
distanciaCuadricula = 0.25
# Numero de cuadriculas de la escena seleccionada
n=int(10/distanciaCuadricula)

# Clase que representa una casilla, tiene ubicacion o punto que la define (mitad) y si un objeto la cubre o no.
class Casilla:
    def __init__(self, xP, yP, pE):
        self.x=float(xP)
        self.y=float(yP)
        self.libre=pE


#En este metodo se inicializa el nodo, se suscribe a los topicos necesarios, se crea la variable
#para publicar al topico de motorsVel y tambien se lanza el nodo encargado de graficar.
def punto2_a():
    global twistInfoDim, twistInfoPos

    rospy.init_node('punto2_bJDS', anonymous=True)
    rospy.Subscriber('InfoObs0', Twist, setObst)
    rospy.Subscriber('InfoObs1', Twist, setObst1)
    rospy.Subscriber('InfoObs2', Twist, setObst2)
    rospy.Subscriber('InfoObs3', Twist, setObst3)
    rospy.Subscriber('InfoObs4', Twist, setObst4)
    time.sleep(2) # Espera a que se actualice informacion de todos los obstaculos
    creadorVerticesCasillas()
    creadorArcos()
    visualizacionGrafica()
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
        #rate.sleep()

#Info de obstaculo 1
def setObst(posicionObstacle):
    global twistInfoPos1
    twistInfoPos1=posicionObstacle
    # print twistInfoPos1,linear

#Info de obstaculo 2
def setObst1(posicionObstacle):
    global twistInfoPos2
    twistInfoPos2=posicionObstacle


#Info de obstaculo 3
def setObst2(posicionObstacle):
    global twistInfoPos3
    twistInfoPos3=posicionObstacle


#Info de obstaculo 4
def setObst3(posicionObstacle):
    global twistInfoPos4
    twistInfoPos4=posicionObstacle


#Info de obstaculo 5
def setObst4(posicionObstacle):
    global twistInfoPos5
    twistInfoPos5=posicionObstacle


def creadorVerticesCasillas():
    global distanciaCuadricula, Equivalente,numeroCasillas, n, Links, g
    xInic=-5.0+np.divide(float(distanciaCuadricula),2.0)
    yInic=5.0-np.divide(float(distanciaCuadricula),2.0)
    for i in range(0,n*n):
        g.add_node(i)
        mod=np.mod(i,n)#columnas de la matriz
        div=int(np.floor(i/n))#filas de la matriz
        if len(Equivalente)<div+1:
            Equivalente.append([])
        nuevaCasilla = Casilla(xInic+mod*distanciaCuadricula,yInic-div*distanciaCuadricula, libre(xInic+mod*distanciaCuadricula,yInic-div*distanciaCuadricula) )
        Equivalente[div].append(nuevaCasilla)


def creadorArcos():
    for i in range(0,n**2):
        for j in range(i, n**2):
            if j!=i and math.sqrt((i%n-j%n)**2 +(i//n-j//n)**2)<=math.sqrt(2) and libre(Equivalente[i//n][i%n].x, Equivalente[i//n][i%n].y) and libre(Equivalente[j//n][j%n].x, Equivalente[j//n][j%n].y):
                g.add_edge(i,j)


def libre(xCas, yCas):# Si se encuentra un obstaculo en ella
    distanciaCarro=0.544
    dist0 = math.sqrt ((twistInfoPos1.linear.x - xCas) ** 2 + (twistInfoPos1.linear.y - yCas) ** 2)
    dist1 = math.sqrt ((twistInfoPos2.linear.x - xCas) ** 2 + (twistInfoPos2.linear.y - yCas) ** 2)
    dist2 = math.sqrt ((twistInfoPos3.linear.x - xCas) ** 2 + (twistInfoPos3.linear.y - yCas) ** 2)
    dist3 = math.sqrt ((twistInfoPos4.linear.x - xCas) ** 2 + (twistInfoPos4.linear.y - yCas) ** 2)
    dist4 = math.sqrt ((twistInfoPos5.linear.x - xCas) ** 2 + (twistInfoPos5.linear.y - yCas) ** 2)
    distRef0 = twistInfoPos1.linear.z/2 + distanciaCarro
    distRef1 = twistInfoPos2.linear.z/2 + distanciaCarro
    distRef2 = twistInfoPos3.linear.z/2 + distanciaCarro
    distRef3 = twistInfoPos4.linear.z/2 + distanciaCarro
    distRef4 = twistInfoPos5.linear.z/2 + distanciaCarro
    return (dist0>=distRef0) and (dist1>=distRef1) and (dist2>=distRef2) and (dist3>=distRef3) and (dist4>=distRef4)


def imprimirCasillasTerminal():
    global n,Equivalente
    fila = ""
    for i in range(0, n**2):
        columna = i%n
        if Equivalente[i//n][columna].libre:
            fila = fila + "1"
        else:
            fila = fila + "0"
        if columna == n-1:
            print(fila)
            fila=""

def visualizacionGrafica():
    global n
    xLibres = []
    yLibres = []
    xOcupadas = []
    yOcupadas = []
    for i in range(0, n**2):
        if Equivalente[i//n][i%n].libre:
            xLibres.append(Equivalente[i//n][i%n].x)
            yLibres.append (Equivalente[i // n][i % n].y)
        else:
            xOcupadas.append (Equivalente[i // n][i % n].x)
            yOcupadas.append (Equivalente[i // n][i % n].y)
    plt.plot(xLibres, yLibres, 'ro')
    plt.plot(xOcupadas, yOcupadas, 'bo')
    plt.show()



if __name__ == '__main__':
    try:
        punto2_a()
    except rospy.ROSInterruptException:
        pass
