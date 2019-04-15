#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math

fig = None
axs = None
xActual = 0
yActual = 0
pActual  = 0

obs = []
num = 0

ani = None


def graficar():
    global fig, xCord, axs, ani
    fig = plt.figure()
    axs = fig.add_subplot(111)
    rospy.init_node('graficador3a', anonymous=True)
    rospy.Subscriber('pioneerPosition', Twist, setNewPosition)
    rospy.Subscriber('scanner', Float32MultiArray, setObstacles)
    ani = animation.FuncAnimation(fig, animate)
    plt.show()

def setObstacles(puntos):
    global num, obs

    num = puntos.layout.data_offset
    obs = puntos.data

def setNewPosition(pos):
    global xCord, yCord, xActual, yActual, pActual
    xActual = pos.linear.x
    yActual = pos.linear.y
    pActual = pos.angular.z

    # xCord.append(xActual)
    # xCord = xCord[-100:]
    # yCord.append(yActual)
    # yCord = yCord[-100:]


def animate(i):
    global axs, num, xActual, yActual, pActual
    axs.clear()
    axs.axes.set_xlim(-5, 5)
    axs.axes.set_ylim(-5, 5)
    axs.scatter(xActual, yActual, c = 'b')

    i = 0

    while i < num:
        p = obs[i]
        d = obs[i+1]

        x = math.cos(p+pActual)*d
        y = math.sin(p+pActual)*d

        axs.scatter(x+xActual,y+yActual, c = 'r')

        i = i+2

    plt.title('Posicion en tiempo real de Robot y puntos crudos detectados')
    plt.grid()


if __name__ == '__main__':
    try:
        graficar()
    except rospy.ROSInterruptException:
        pass
