#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from geometry_msgs.msg import Twist

fig = None
axs = None
xCord = []
yCord = []
ani = None


def graficar():
    global fig, xCord, axs, ani
    fig = plt.figure()
    axs = fig.add_subplot(111)
    rospy.init_node('graficador', anonymous=True)
    rospy.Subscriber('topico_Posicion', Twist, setNewPosition)
    ani = animation.FuncAnimation(fig, animate)
    plt.show()


def setNewPosition(pos):
    global xCord, yCord
    xCord.append(pos.linear.x)
    xCord = xCord[-100:]
    yCord.append(pos.linear.y)
    yCord = yCord[-100:]


def animate(i):
    global axs, xCord, yCord
    axs.clear()
    axs.axes.set_xlim(-5, 5)
    axs.axes.set_ylim(-5, 5)
    axs.plot(xCord, yCord)
    plt.title('Posicion en tiempo real de Robot')
    plt.grid()


if __name__ == '__main__':
    try:
        graficar()
    except rospy.ROSInterruptException:
        pass
