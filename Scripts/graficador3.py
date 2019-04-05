#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math

fig = None
axs = None
xCord = []
yCord = []
xActual = 0
yActual = 0

obs = []
num = 0

ani = None


def graficar():
    global fig, xCord, axs, ani
    fig = plt.figure()
    axs = fig.add_subplot(111)
    rospy.init_node('graficador', anonymous=True)
    rospy.Subscriber('topico_Posicion', Twist, setNewPosition)
    rospy.Subscriber('scanner', Float32MultiArray, setObstacles)
    ani = animation.FuncAnimation(fig, animate)
    plt.show()

def setObstacles(puntos):
    global num, obs

    num = puntos.layout.data_offset
    obs = puntos.data

    pass

def setNewPosition(pos):
    global xCord, yCord, xActual, yActual
    xActual = pos.linear.x
    yActual = pos.linear.y
    xCord.append(xActual)
    xCord = xCord[-100:]
    yCord.append(yActual)
    yCord = yCord[-100:]


def animate(i):
    global axs, xCord, yCord, num
    axs.clear()
    axs.axes.set_xlim(-5, 5)
    axs.axes.set_ylim(-5, 5)
    axs.plot(xCord, yCord)

    i = 0

    while i < num:
        p = obs[i]
        d = obs[i+1]

        rospy.loginfo("P: {}, D: {}, N: {}".format(p,d,num))

        x = math.cos(p)*d
        y = math.sin(p)*d

        axs.scatter(x+xActual,y+yActual)

        i = i+2

    plt.title('Posicion en tiempo real de Robot')
    plt.grid()


if __name__ == '__main__':
    try:
        graficar()
    except rospy.ROSInterruptException:
        pass
