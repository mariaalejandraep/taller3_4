#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math

import numpy as np

fig = None
axs = None
xCord = []
yCord = []
xActual = 0
yActual = 0
pActual = 0

obs = []
num = 0
numRectas = 0

m = []
b = []
xmin = []
xmax = []

ani = None


def graficar():
    global fig, xCord, axs, ani, xmin, xmax
    fig = plt.figure()
    axs = fig.add_subplot(111)
    rospy.init_node('graficador3c', anonymous=True)
    rospy.Subscriber('topico_Posicion', Twist, setNewPosition)
    rospy.Subscriber('rectas', Float32MultiArray, ponerRectas)
    rospy.Subscriber('scanner', Float32MultiArray, setObstacles)
    ani = animation.FuncAnimation(fig, animate)
    plt.show()

def ponerRectas(rectas):
    global num, numRectas, obs, m, b, xmin, xmax

    m[:] = []
    b[:] = []
    xmin[:] = []
    xmax[:] = []

    numRectas = len(rectas.data)/4.0
    #rospy.loginfo(numRectas)

    i = 0
    while i < numRectas*4:

        m.append(rectas.data[i])
        b.append(rectas.data[i+1])
        xmin.append(rectas.data[i+2])
        xmax.append(rectas.data[i+3])

        #rospy.loginfo(m)
        #rospy.loginfo(b)
        i = i+4

def setNewPosition(pos):
    global xActual, yActual, pActual
    xActual = pos.linear.x
    yActual = pos.linear.y
    pActual = pos.angular.z

    # xCord.append(xActual)
    # xCord = xCord[-100:]
    # yCord.append(yActual)
    # yCord = yCord[-100:]

def setObstacles(puntos):
    global num, obs

    num = puntos.layout.data_offset
    obs = puntos.data

def animate(i):
    global axs, xCord, yCord, num, m, b, xmin, xmax, xActual, yActual, pActual
    axs.clear()
    axs.axes.set_xlim(-5, 5)
    axs.axes.set_ylim(-5, 5)
    axs.scatter(xActual, yActual)

    i = 0

    #rospy.loginfo("numRectas:{}".format(numRectas))
    #rospy.loginfo("lenm:{} y lenb:{}".format(len(m),len(b)))
    #rospy.loginfo(m)
    #rospy.loginfo(b)

    while i < numRectas:

        x = np.linspace(xmin[i],xmax[i],10)
        y = x*m[i] + b[i]

        #rospy.loginfo("xmin:{} y xmax:{}".format(xmin,xmax))
        #rospy.loginfo("Entre")

        axs.plot(x,y,'r')
        rospy.loginfo("imprimi")

        i = i+1


    # i = 0
    #
    # while i < num:
    #     p = obs[i]
    #     d = obs[i+1]
    #
    #     x = math.cos(p+pActual)*d
    #     y = math.sin(p+pActual)*d
    #
    #     axs.scatter(x+xActual,y+yActual)
    #
    #     i = i+2

    plt.title('Posicion en tiempo real de Robot')
    plt.grid()


if __name__ == '__main__':
    try:
        graficar()
    except rospy.ROSInterruptException:
        pass
