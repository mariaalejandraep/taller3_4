#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from geometry_msgs.msg import Twist

# Variable global referencia de la figura
fig = None
# Variable global referencia de los ejes de la figura
axs = None
# Arreglo cordenadas trayectoria en x robot
xCord = []
# Arreglo cordenadas trayectoria en y robot
yCord = []
# Variable global referencia de la animacion
ani = None
# Variable tipo Twist() con posicion actual robot
position = Twist()
#Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 1.
twistInfoPos1 = Twist()
#Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 2.
twistInfoPos2 = Twist()
#Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 3.
twistInfoPos3 = Twist()
#Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 4.
twistInfoPos4 = Twist()
#Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 5.
twistInfoPos5 = Twist()

# Funcion que recibe y actualiza informacion de topicos de obstaculos, numero obstaculo esta guardado en twist.angular.x
def setObst(posicionObstacle):
    global twistInfoPos1, twistInfoPos2, twistInfoPos3, twistInfoPos4, twistInfoPos5
    if posicionObstacle.angular.x==1:
        twistInfoPos1=posicionObstacle
    elif posicionObstacle.angular.x==2:
        twistInfoPos2=posicionObstacle
    elif posicionObstacle.angular.x==3:
        twistInfoPos3=posicionObstacle
    elif posicionObstacle.angular.x==4:
        twistInfoPos4=posicionObstacle
    elif posicionObstacle.angular.x==5:
        twistInfoPos5=posicionObstacle


# Metodo principal: iniciliza nodo, suscribe a topico posicion, crea figura, crea ejes, llama metodo actualizacion de
# datos animate cada 0.3 segundos, muestra figura y cuando esta se cierre las guarda
def graficar():
    global fig, xCord, axs, ani
    fig = plt.figure()
    axs = fig.add_subplot(111)
    rospy.init_node('graficador', anonymous=True)
    rospy.Subscriber('pioneerPosition', Twist, setNewPosition)
    rospy.Subscriber ('InfoObs', Twist, setObst)
    ani = animation.FuncAnimation(fig, animate, interval=300)
    plt.show()
    # fig.savefig ('../catkin_ws/src/taller2_4/results/resultsPunto2/figure.png')


# Metodo para actualizar posicion robot en variable position una vez otro nodo publique en el topico
def setNewPosition(pos):
    global position
    position = pos


# Metodo para actualizar los arreglos de cordenadas, es llamado cada 0.3 segundos por meotodo principal
def animate(i):
    global axs, xCord, yCord, position
    xCord.append (position.linear.x)
    # xCord = xCord[-100:]
    yCord.append (position.linear.y)
    # yCord = yCord[-100:]
    axs.clear()
    axs.axes.set_xlim(-40, 40)
    axs.axes.set_ylim(-40, 40)
    axs.add_artist (plt.Circle ((twistInfoPos1.linear.x, twistInfoPos1.linear.y), twistInfoPos1.linear.z/2, color='r'))
    axs.add_artist (plt.Circle ((twistInfoPos2.linear.x, twistInfoPos2.linear.y), twistInfoPos2.linear.z/2, color='r'))
    axs.add_artist (plt.Circle ((twistInfoPos3.linear.x, twistInfoPos3.linear.y), twistInfoPos3.linear.z/2, color='r'))
    axs.add_artist (plt.Circle ((twistInfoPos4.linear.x, twistInfoPos4.linear.y), twistInfoPos4.linear.z/2, color='r'))
    axs.add_artist (plt.Circle ((twistInfoPos5.linear.x, twistInfoPos5.linear.y), twistInfoPos5.linear.z/2, color='r'))
    axs.plot(xCord, yCord)
    plt.title('Posicion en tiempo real de Robot')
    plt.grid()

# Metodo Main, simplemente ejecuta el metodo principal
if __name__ == '__main__':
    try:
        graficar()
    except rospy.ROSInterruptException:
        pass