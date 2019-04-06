#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from geometry_msgs.msg import Twist
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import Float32MultiArray
import time

import sys

distanciaCuadricula=0.2 #Distancias entre cuadriculas de grafo
n=int(10/distanciaCuadricula) #Acordarse que 10 debe ser divisible por distanciaCuadricula
fig = None
axs = None
xCord = []
yCord = []
ani = None
angulo=[]

Librex=[]
Librey=[]
Obstaculoy=[]
Obstaculox=[]


matriz_Obs_Links=np.zeros((n*n,3))


def setNewPosition(pos): #SE obtienen posiciones de coordenadas del robot
    global xCord, yCord, angulo, n
    angulo.append(pos.angular.z)
    xCord.append(pos.linear.x)
    xCord = xCord[-100:]
    yCord.append(pos.linear.y)
    yCord = yCord[-100:]

def LX(LXX):
    global Librex
    Librex=LXX.data
def LY(LYY):
    global Librey
    Librey=LYY.data
def OY(OYY):
    global Obstaculoy
    Obstaculoy=OYY.data
def OX(OXX):
    global Obstaculox
    Obstaculox=OXX.data


def animate(i): #Se anima la grafica en tiempo real y se ajustan parametros de la grafica
    global axs, xCord, yCord, siga20, Librex,Librey,Obstaculoy,Obstaculox
    axs.axes.set_xlim(-5, 5)
    axs.axes.set_ylim(-5, 5)
    axs.plot(Librex,Librey,'yo')
    axs.plot(Obstaculox,Obstaculoy,'bo')
    axs.plot(xCord, yCord, 'ro' )
    plt.title('Posicion en tiempo real de Robot')
    plt.grid()



def graficar(): #metodo principal
    global fig, xCord, axs, ani
    fig = plt.figure(1)
    axs = fig.add_subplot(111)
    rospy.init_node('graficador_punto_3', anonymous=True) #se inicia el nodo
    rospy.Subscriber('pioneerPosition', Twist, setNewPosition) #SE subscribe a el topico para obtener posiciones del robot
    rospy.Subscriber('Libresx', Float32MultiArray,  LX)#Se publican velocidades de los motores.
    rospy.Subscriber('Libresy', Float32MultiArray,  LY)#Se publican velocidades de los motores.
    rospy.Subscriber('Obsx', Float32MultiArray,  OX)#Se publican velocidades de los motores.
    rospy.Subscriber('Obsy', Float32MultiArray,  OY)#Se publican velocidades de los motores.
    ani = animation.FuncAnimation(fig, animate)
    plt.show() #SE muestra constantemente la grafica
    #fig.savefig('../catkin_ws/src/taller2_4/results/resultsPunto3/TiempoReal.png')




if __name__ == '__main__':
    try:

        graficar()
    except rospy.ROSInterruptException:
        pass
