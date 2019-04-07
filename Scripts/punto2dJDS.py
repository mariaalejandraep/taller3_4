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
import matplotlib.pyplot as plt1
import time
import random

#l = math.sqrt(np.power(0.38/2,2)+np.power(0.51/2,2)) #metros
l=0.4/2
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


casillasRRT = []

xDescartados = []
yDescartados = []

# Clase que representa una casilla, tiene ubicacion o punto que la define (mitad) y si un objeto la cubre o no.
class Casilla:
    def __init__(self, xP, yP):
        self.x=float(xP)
        self.y=float(yP)


#En este metodo se inicializa el nodo, se suscribe a los topicos necesarios, se crea la variable
#para publicar al topi    plt.plot(xLibres, yLibres, 'ro')

def punto2_d():
    rospy.init_node('punto2_d', anonymous=True)
    rospy.Subscriber('InfoObs', Twist, setObst)
    rospy.Subscriber ('pioneerPosition', Twist, setPositionCallback)
    time.sleep(2) # Espera a que se actualice informacion de todos los obstaculos
    RRT(twistInfoPioneer.linear.x, twistInfoPioneer.linear.y, -4, 4)
    # creadorVerticesCasillas()
    # creadorArcos()
    visualizacionGrafica()
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
        #rate.sleep()


def setObst(posicionObstacle):
    global twistInfoPos1, twistInfoPos2, twistInfoPos3, twistInfoPos4, twistInfoPos5
    if posicionObstacle.angular.x == 1 :
        twistInfoPos1=posicionObstacle
    elif posicionObstacle.angular.x == 2 :
        twistInfoPos2=posicionObstacle
    elif posicionObstacle.angular.x == 3 :
        twistInfoPos3=posicionObstacle
    elif posicionObstacle.angular.x == 4 :
        twistInfoPos4=posicionObstacle
    else:
        twistInfoPos5=posicionObstacle


def setPositionCallback(pos):
    global  twistInfoPioneer
    twistInfoPioneer = pos



def libre(xCas, yCas):# Si se encuentra un obstaculo en ella
    distanciaCarro=l
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


def RRT(xIni, yIni, xFin, yFin):
    global g, distanciaCuadricula
    g.add_node(0)
    step = distanciaCuadricula
    radioError = .25
    casillasRRT.append(Casilla(xIni,yIni))
    contador = 1
    llego = False
    while not llego and contador != 20000:
        xAleatorio = random.randrange(-500,500,1)/100
        yAleatorio = random.randrange(-500,500,1)/100
        posCasillaCercana = posCasillaMasCercana(xAleatorio, yAleatorio)
        xCercana = casillasRRT[posCasillaCercana].x
        yCercana = casillasRRT[posCasillaCercana].y
        teta = math.atan2((yAleatorio-yCercana),(xAleatorio-xCercana))
        xPropuesto = xCercana + step * math.cos(teta)
        yPropuesto = yCercana + step * math.sin(teta)
        if libre(xPropuesto, yPropuesto):
            g.add_node(contador)
            casillasRRT.append(Casilla(xPropuesto, yPropuesto))
            g.add_edge(posCasillaCercana,contador)
            contador = contador + 1
            if math.sqrt((xFin-xPropuesto)**2+(yFin-yPropuesto)**2)<=radioError:
                llego = True
            if contador%100==0:
                print contador
        else:
            xDescartados.append(xPropuesto)
            yDescartados.append(yPropuesto)
    print "Cantidad final de nodos: ",contador


def posCasillaMasCercana(x, y):
    global casillasRRT
    distancia = float('Inf')
    numCasilla = -1
    for i in range(0, len(casillasRRT)):
        casillaActual = casillasRRT[i]
        distCasillaActual = math.sqrt((casillaActual.x-x)**2+(casillaActual.y-y)**2)
        if distCasillaActual < distancia:
            numCasilla = i
            distancia = distCasillaActual
    return numCasilla


def visualizacionGrafica():
    global casillasRRT, xDescartados, yDescartados, g
    cordX = []
    cordY = []
    xPath = []
    yPath = []
    path = nx.astar_path(g, 0, len(casillasRRT)-1)
    for i in range(0, len(casillasRRT)):
        casillaActual = casillasRRT[i]
        if i not in path:
            cordX.append(casillaActual.x)
            cordY.append(casillaActual.y)
        else:
            xPath.append (casillaActual.x)
            yPath.append (casillaActual.y)
    plt.plot(cordX, cordY, 'bo')
    plt.plot(xDescartados, yDescartados, 'ro')
    plt.plot(xPath,yPath,'go')
    plt.show()
    # plt.clf()
    # nx.draw(g,with_labels=True)
    # nx.draw(g)
    # plt.show()




if __name__ == '__main__':
    try:

        punto2_d()
    except rospy.ROSInterruptException:
        pass