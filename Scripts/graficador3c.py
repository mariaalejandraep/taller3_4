#!/usr/bin/env python
#Las librerias que se importan.
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

#Son las variables asociadas a la ventana emergente para graficar.
fig = None
axs = None
ani = None

#Son las variables donde se almacena la posicion actual del robot.
xActual = 0
yActual = 0

#Es el numero de rectas-
numRectas = 0

#Son los vectores donde se almacenanas los parametros de cada recta.
m = []
b = []
xmin = []
xmax = []

#Este metodo inicializa el nodo graficador.
def graficar():
    global fig, xCord, axs, ani, xmin, xmax
    fig = plt.figure()
    axs = fig.add_subplot(111)
    rospy.init_node('graficador3c', anonymous=True)
    rospy.Subscriber('pioneerPosition', Twist, setNewPosition)
    rospy.Subscriber('rectas', Float32MultiArray, ponerRectas)
    ani = animation.FuncAnimation(fig, animate)
    plt.show()

#Este metodo actualiza las variables m,b,xmin,xmax con los valores del topico rectas.
def ponerRectas(rectas):
    global num, numRectas, obs, m, b, xmin, xmax

    m[:] = []
    b[:] = []
    xmin[:] = []
    xmax[:] = []

    numRectas = len(rectas.data)/4.0

    i = 0
    while i < numRectas*4:

        m.append(rectas.data[i])
        b.append(rectas.data[i+1])
        xmin.append(rectas.data[i+2])
        xmax.append(rectas.data[i+3])
        i = i+4

#Este metodo actualiza la posicion actual del robot.
def setNewPosition(pos):
    global xActual, yActual
    xActual = pos.linear.x
    yActual = pos.linear.y

#Este metodo grafica las rectas en rojo y la posicion actual del robot.
def animate(i):
    global axs, m, b, xmin, xmax, xActual, yActual

    axs.clear()
    axs.axes.set_xlim(-5, 5)
    axs.axes.set_ylim(-5, 5)

    axs.scatter(xActual, yActual, c='b')

    i = 0

    while i < numRectas:

        x1 = xmin[i]
        x2 = xmax[i]

        y1 = m[i]*x1 + b[i]
        y2 = m[i]*x2 + b[i]


        axs.plot([x1, x2],[y1,y2],'r')

        i = i+1

    plt.title('Posicion en tiempo real del Robot y lineas detectadas')
    plt.grid()


if __name__ == '__main__':
    try:
        graficar()
    except rospy.ROSInterruptException:
        pass
