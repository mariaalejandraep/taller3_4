#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from geometry_msgs.msg import Twist


fig = None
axs = None
xCord = []
yCord = []
angulo = []
ani = None


def setNewPosition(pos): #SE obtienen posiciones de coordenadas del robot
    global xCord, yCord, angulo
    angulo.append(pos.angular.z)
    xCord.append(pos.linear.x)
    xCord = xCord[-100:]
    yCord.append(pos.linear.y)
    yCord = yCord[-100:]



def animate(i): #Se anima la grafica en tiempo real y se ajustan parametros de la grafica
    global axs, xCord, yCord, siga20
    axs.axes.set_xlim(-5, 5)
    axs.axes.set_ylim(-5, 5)
    axs.plot(xCord, yCord)
    plt.title('Posicion en tiempo real de Robot')
    plt.grid()


def graficar(): #metodo principal
    global fig, xCord, axs, ani
    fig = plt.figure(1)
    axs = fig.add_subplot(111)
    rospy.init_node('graficador_punto_3', anonymous=True) #se inicia el nodo
    rospy.Subscriber('pioneerPosition', Twist, setNewPosition) #SE subscribe a el topico para obtener posiciones del robot
    ani = animation.FuncAnimation(fig, animate)
    plt.show() #SE muestra constantemente la grafica
    fig.savefig('../catkin_ws/src/taller2_4/results/resultsPunto3/TiempoReal.png')

if __name__ == '__main__':
    try:
        graficar()# Se
    except rospy.ROSInterruptException:
        pass
