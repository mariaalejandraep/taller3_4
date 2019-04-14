#!/usr/bin/env python
#Se importan las librerias necesarias junto con los mensajes a utilizar
import rospy
from geometry_msgs.msg import Twist

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
        print "Coordenada x de obstaculo 1 ", twistInfoPos1.linear.x
        print "Coordenada y de obstaculo 1 ", twistInfoPos1.linear.y
        print "Radio de obstaculo 1 ", twistInfoPos1.linear.z
    elif posicionObstacle.angular.x==2:
        twistInfoPos2=posicionObstacle
        print "Coordenada x de obstaculo 2 ", twistInfoPos2.linear.x
        print "Coordenada y de obstaculo 2 ", twistInfoPos2.linear.y
        print "Radio de obstaculo 2 ", twistInfoPos1.linear.z
    elif posicionObstacle.angular.x==3:
        twistInfoPos3=posicionObstacle
        print "Coordenada x de obstaculo 3 ", twistInfoPos3.linear.x
        print "Coordenada y de obstaculo 3 ", twistInfoPos3.linear.y
        print "Radio de obstaculo 3 ", twistInfoPos3.linear.z
    elif posicionObstacle.angular.x==4:
        twistInfoPos4=posicionObstacle
        print "Coordenada x de obstaculo 4 ", twistInfoPos4.linear.x
        print "Coordenada y de obstaculo 4 ", twistInfoPos4.linear.y
        print "Radio de obstaculo 4 ", twistInfoPos4.linear.z
    elif posicionObstacle.angular.x==5:
        twistInfoPos5=posicionObstacle
        print "Coordenada x de obstaculo 5 ", twistInfoPos5.linear.x
        print "Coordenada y de obstaculo 5 ", twistInfoPos5.linear.y
        print "Radio de obstaculo 5 ", twistInfoPos5.linear.z

# Metodo principal, crea el nodo de ROS, se suscribe a topico de informacion obstaculos e imprime su informacion
# mientras que el nodo se este ejecutando
if __name__ == '__main__':
    try:
        rospy.init_node ('punto2a', anonymous=True)
        rospy.Subscriber ('InfoObs', Twist, setObst)
        rate = rospy.Rate (10)
        while not rospy.is_shutdown ():
            rate.sleep ()
    except rospy.ROSInterruptException:
        pass
