#!/usr/bin/env python
#Se importan las librerias necesarias junto con los mensajes a utilizar
import rospy
import math
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import networkx as nx
import time


# Clase que representa una casilla, tiene ubicacion o punto que la define (mitad) y si un objeto la cubre o no.
class Casilla:
    def __init__(self, xP, yP, pE):
        self.x=float(xP)
        self.y=float(yP)
        self.libre=pE

#l = math.sqrt(np.power(0.38/2,2)+np.power(0.51/2,2)) #metros
l=0.27
# Grafo de networkx con informacion de vertices y arcos
g=nx.Graph()
# Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 1.
twistInfoPos1 = Twist()
# Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 2.
twistInfoPos2 = Twist()
# Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 3.
twistInfoPos3 = Twist()
# Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 4.
twistInfoPos4 = Twist()
# Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 5.
twistInfoPos5 = Twist()
# Matriz de distribucion e informacion de casillas
Equivalente=[]
# Distancia entre centro de cuadriculas, 10 debe ser divisible por esta distancia
distanciaCuadricula = 0.25
# Numero de cuadriculas en un lado de la escena seleccionada
n=int(10/distanciaCuadricula)


# Metodo de asociado a topico de informacion de obstaculos
def setObst(posicionObstacle):
    global twistInfoPos1, twistInfoPos2, twistInfoPos3, twistInfoPos4, twistInfoPos5
    if posicionObstacle.angular.x == 1 :
        twistInfoPos1=posicionObstacle
    elif posicionObstacle.angular.x == 2 :
        twistInfoPos2=posicionObstacle
    elif posicionObstacle.angular.x == 3 :
        twistInfoPos3=posicionObstacle
    elif posicionObstacle.angular.x == 4 :
        twistInfoPos4=posicionObstacle
    else:
        twistInfoPos5=posicionObstacle


# Metodo que genera y anade los vertices a la matriz y el grafo segun el distanciamiento elegido
def creadorVerticesCasillas():
    global distanciaCuadricula, Equivalente, n, g
    xInic=-5.0+distanciaCuadricula/2
    yInic=5.0-distanciaCuadricula/2
    for i in range(0,n*n):
        g.add_node(i)
        mod=i%n
        div=i//n
        if len(Equivalente)<div+1:
            Equivalente.append([])
        nuevaCasilla = Casilla(xInic+mod*distanciaCuadricula,yInic-div*distanciaCuadricula, libre(xInic+mod*distanciaCuadricula,yInic-div*distanciaCuadricula) )
        Equivalente[div].append(nuevaCasilla)


# Crea los arcos entre los vertices creados en el grafo, solo es necesario recorrer la mitad de la matriz debido a que
# es un grafo no dirigido
def creadorArcos():
    for i in range(0,n**2):
        for j in range(i, n**2):
            if j!=i and math.sqrt((i%n-j%n)**2 +(i//n-j//n)**2)<=math.sqrt(2) and libre(Equivalente[i//n][i%n].x, Equivalente[i//n][i%n].y) and libre(Equivalente[j//n][j%n].x, Equivalente[j//n][j%n].y):
                g.add_edge(i,j)


# Metodo que indica si cierta posicion en la escena esta ocupada por alguno de los obstaculos, devuelve True en caso de
# que no haya obstaculo en ese punto y False de lo contrario
def libre(xCas, yCas):# Si se encuentra un obstaculo en ella
    distanciaCarro=l
    dist0 = math.sqrt ((twistInfoPos1.linear.x - xCas) ** 2 + (twistInfoPos1.linear.y - yCas) ** 2)
    dist1 = math.sqrt ((twistInfoPos2.linear.x - xCas) ** 2 + (twistInfoPos2.linear.y - yCas) ** 2)
    dist2 = math.sqrt ((twistInfoPos3.linear.x - xCas) ** 2 + (twistInfoPos3.linear.y - yCas) ** 2)
    dist3 = math.sqrt ((twistInfoPos4.linear.x - xCas) ** 2 + (twistInfoPos4.linear.y - yCas) ** 2)
    dist4 = math.sqrt ((twistInfoPos5.linear.x - xCas) ** 2 + (twistInfoPos5.linear.y - yCas) ** 2)
    distRef0 = twistInfoPos1.linear.z/2 + distanciaCarro
    distRef1 = twistInfoPos2.linear.z/2 + distanciaCarro
    distRef2 = twistInfoPos3.linear.z/2 + distanciaCarro
    distRef3 = twistInfoPos4.linear.z/2 + distanciaCarro
    distRef4 = twistInfoPos5.linear.z/2 + distanciaCarro
    return (dist0>=distRef0) and (dist1>=distRef1) and (dist2>=distRef2) and (dist3>=distRef3) and (dist4>=distRef4)

# Metodo que grafica distribucion de casillas y grafo generados.
def visualizacionGrafica():
    global n, g
    xLibres = []
    yLibres = []
    xOcupadas = []
    yOcupadas = []
    for i in range(0, n**2):
        if Equivalente[i//n][i%n].libre:
            xLibres.append(Equivalente[i//n][i%n].x)
            yLibres.append (Equivalente[i // n][i % n].y)
        else:
            xOcupadas.append (Equivalente[i // n][i % n].x)
            yOcupadas.append (Equivalente[i // n][i % n].y)
    plt.plot(xLibres, yLibres, 'ro')
    plt.plot(xOcupadas, yOcupadas, 'bo')
    plt.title("Distribucion de casillas y lugar de obstaculos")
    plt.show()
    plt.clf()
    nx.draw(g)
    plt.show()


# Metodo principal, crea el nodo, se suscribe al topico de informacion de obstaculos crea vertices y arcos y luego los
# grafica.
if __name__ == '__main__':
    try:
        rospy.init_node ('punto2b', anonymous=True)
        rospy.Subscriber ('InfoObs', Twist, setObst)
        time.sleep (.1)
        creadorVerticesCasillas()
        creadorArcos()
        visualizacionGrafica()
    except rospy.ROSInterruptException:
        pass