#!/usr/bin/env python
#Se importan las librerias necesarias junto con los mensajes a utilizar
import rospy, time, math
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import networkx as nx


# Clase que representa una casilla, tiene ubicacion o punto que la define (mitad) y si un objeto la cubre o no.
class Casilla:
    def __init__(self, xP, yP, pE):
        self.x=float(xP)
        self.y=float(yP)
        self.libre=pE


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
distanciaCuadricula = .5
# Numero de cuadriculas en un lado de la escena seleccionada
n=int(80/distanciaCuadricula)
# Senal para saber si ya se esta corriendo la simulacion de ROS
empezar = False


# Metodo de asociado a topico de informacion de obstaculos
def setObst(posicionObstacle):
    global twistInfoPos1, twistInfoPos2, twistInfoPos3, twistInfoPos4, twistInfoPos5, empezar
    # Los condicionales a continuacion definen de cual obstaculo es que se debe actualizar la informacion una vez se
    # ejecute el metodo por el topico
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
    # Este condicional me permite salir del loop del metodo principal una vez se empiece a simular la escena en V-REP
    if not empezar:
        empezar = True


# Metodo que genera y anade los vertices a la matriz y el grafo segun el distanciamiento elegido
def creadorVerticesCasillas():
    global distanciaCuadricula, Equivalente, n, g
    # Las variables a continuacion definen desde que posicion se empieza a discretizar la escena
    xInic=-40.0+distanciaCuadricula/2
    yInic=40-distanciaCuadricula/2
    # El ciclo empieza a generar la matriz de casillas donde se guarda la discretizacion y anade el nodo al grafo.
    for i in range(0,n*n):
        g.add_node(i)
        mod=i%n
        div=i//n
        if len(Equivalente)<div+1:
            Equivalente.append([])
        nuevaCasilla = Casilla(xInic+mod*distanciaCuadricula,yInic-div*distanciaCuadricula, libre(xInic+mod*distanciaCuadricula,yInic-div*distanciaCuadricula) )
        Equivalente[div].append(nuevaCasilla)

# Crea los arcos entre los vertices creados en el grafo, solo es necesario revisar los nodos que pueden ser vecinos, en
# de que ambos esten vacios genera el arco.
def creadorArcos():
    global n
    # El ciclo revisa cada uno de los nodos del grafo
    for i in range(0, n**2):
        # Las siguientes variables definen cual es la fila y columna del nodo correspondiente en la matriz Equivalente
        c = i % n
        f = i // n
        # Se revisa primero si la casilla actual esta libre de obstaculos, de lo contrario no es necesario generar arcos
        if Equivalente[f][c].libre:
            # El ciclo revisa los posibles ocho vecinos de la casilla actual en cada una de las iteraciones
            for j in range (1, 9):
                if j == 1:
                    cP = c-1
                    fP = f
                elif j == 2:
                    cP = c-1
                    fP = f-1
                elif j == 3:
                    cP = c
                    fP = f-1
                elif j == 4:
                    cP = c+1
                    fP = f-1
                elif j == 5:
                    cP = c+1
                    fP = f
                elif j == 6:
                    cP = c+1
                    fP = f+1
                elif j == 7:
                    cP = c
                    fP = f+1
                elif j == 8:
                    cP = c-1
                    fP = f+1
                # En caso de que la casilla si pertenezca a los rangos de discretizacion y este libre se crea un arco
                # entre la casilla actual y el vecino revisado
                if cP in range(0, n) and fP in range(0, n) and Equivalente[fP][cP].libre:
                    g.add_edge (i, fP*n+cP)


# Metodo que indica si cierta posicion en la escena esta ocupada por alguno de los obstaculos, devuelve True en caso de
# que no haya obstaculo en ese punto y False de lo contrario
def libre(xCas, yCas):# Si se encuentra un obstaculo en ella
    # Dependiendo de si la discretizacion de las casillas es fina con respecto al tamano del robot se toma como
    # distancia minima la distancia maxima entre el punto P y el borde del Pioneer o si es grande se toma como la
    # distancia de la diagonal entre el el centro una casilla y una de sus esquinas
    if math.sqrt( (0.51 / 2) ** 2 + 0.41 ** 2) > distanciaCuadricula/math.sqrt(2):
        distanciaCarro = math.sqrt( (0.51 / 2) ** 2 + 0.41 ** 2)
    else:
        distanciaCarro = distanciaCuadricula/math.sqrt(2)
    # Se saca la distancia entre el centor de los obstaculos y el punto pasado por parametro
    dist0 = math.sqrt ((twistInfoPos1.linear.x - xCas) ** 2 + (twistInfoPos1.linear.y - yCas) ** 2)
    dist1 = math.sqrt ((twistInfoPos2.linear.x - xCas) ** 2 + (twistInfoPos2.linear.y - yCas) ** 2)
    dist2 = math.sqrt ((twistInfoPos3.linear.x - xCas) ** 2 + (twistInfoPos3.linear.y - yCas) ** 2)
    dist3 = math.sqrt ((twistInfoPos4.linear.x - xCas) ** 2 + (twistInfoPos4.linear.y - yCas) ** 2)
    dist4 = math.sqrt ((twistInfoPos5.linear.x - xCas) ** 2 + (twistInfoPos5.linear.y - yCas) ** 2)
    # La distancia referencia para cada obstaculo teniendo en cuanta la variable generada anteriormente
    distRef0 = twistInfoPos1.linear.z/2 + distanciaCarro
    distRef1 = twistInfoPos2.linear.z/2 + distanciaCarro
    distRef2 = twistInfoPos3.linear.z/2 + distanciaCarro
    distRef3 = twistInfoPos4.linear.z/2 + distanciaCarro
    distRef4 = twistInfoPos5.linear.z/2 + distanciaCarro
    # En caso de que alguna distancia de referencia mayor que la distancia al obstaculo se considera que ese punto esta
    # ocupado por un obstaculo
    return (dist0>=distRef0) and (dist1>=distRef1) and (dist2>=distRef2) and (dist3>=distRef3) and (dist4>=distRef4)

# Metodo que grafica distribucion de casillas y grafo generados.
def visualizacionGrafica():
    global n, g
    # Se generan los arreglos para identificar cordenadas lobres y ocupadas
    xLibres = []
    yLibres = []
    xOcupadas = []
    yOcupadas = []
    # Se revisan las casillas de la matriz
    for i in range(0, n**2):
        # En caso de que esten libres o ocupadas se agregan al arreglo correspondiente
        if Equivalente[i//n][i%n].libre:
            xLibres.append(Equivalente[i//n][i%n].x)
            yLibres.append (Equivalente[i // n][i % n].y)
        else:
            xOcupadas.append (Equivalente[i // n][i % n].x)
            yOcupadas.append (Equivalente[i // n][i % n].y)
    # Se graficas con puntos azules las casillas libres y con puntos rojos las casillas ocupadas
    plt.plot(xLibres, yLibres, 'bo')
    plt.plot(xOcupadas, yOcupadas, 'ro')
    # Se pone titulo y grafica la imagen
    plt.title("Distribucion de casillas y lugar de obstaculos")
    plt.show()



# Metodo principal, crea el nodo, se suscribe al topico de informacion de obstaculos crea vertices y arcos y luego los
# grafica.
if __name__ == '__main__':
    try:
        # Se crean el nodo
        rospy.init_node ('punto2b', anonymous=True)
        # Se suscribe a al topico de la informacion de los obstaculos
        rospy.Subscriber ('InfoObs', Twist, setObst)
        # Se espera a que se publique por primera vez a traves del topico
        while not empezar:
            empezar = empezar or False
        # Como medida de seguridad se espera .1 segundos para asegurarse de que si se actulizaron los valores de los
        # obstaculos
        time.sleep(.1)
        # Se crean los vertices y casillas del grafo y matriz respectivamente
        creadorVerticesCasillas()
        # Se crean los arcos del grafo
        creadorArcos()
        # Se muestra la grafica de la discretizacion
        visualizacionGrafica()
    except rospy.ROSInterruptException:
        pass