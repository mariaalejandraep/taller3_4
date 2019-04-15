#!/usr/bin/env python
#Las librerias que se importan
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math

#Son las variables asociadas a la ventana emergente donde se grafican los puntos.
fig = None
axs = None
ani = None

#Son las variables donde se almacena la posicion actual del robot.
xActual = 0
yActual = 0
pActual  = 0

#Son las variables donde se almacena el numero y el arreglo de los puntos detectados.
obs = []
num = 0

#En este metodo se inicializa el nodo y se suscribe a los topicos de la posicion actual del robot y los puntos detectados.
def graficar():
    global fig, xCord, axs, ani
    fig = plt.figure()
    axs = fig.add_subplot(111)
    rospy.init_node('graficador3a', anonymous=True)
    rospy.Subscriber('pioneerPosition', Twist, setNewPosition)
    rospy.Subscriber('scanner', Float32MultiArray, setObstacles)
    ani = animation.FuncAnimation(fig, animate)
    plt.show()

#En este metodo se actualiza la variables con el numero de puntos y el arreglo de puntos.
def setObstacles(puntos):
    global num, obs

    num = puntos.layout.data_offset
    obs = puntos.data

#En este metodo se actualiza la posicion actual del robot.
def setNewPosition(pos):
    global xCord, yCord, xActual, yActual, pActual
    xActual = pos.linear.x
    yActual = pos.linear.y
    pActual = pos.angular.z

#En este metodo se grafican los puntos crudos detectados en rojo junto con la posicion actual del robot en azul.
#ES importante recalcar que todas las posiciones son en el marco global de referencia.
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
