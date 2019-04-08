#!/usr/bin/env python
#Se importan las librerias necesarias junto con los mensajes a utilizar
import rospy, math, roslaunch, time, sys
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import networkx as nx

# Clase que representa una casilla, tiene ubicacion o punto que la define (mitad) y si un objeto la cubre o no.
class Casilla:
    def __init__(self, xP, yP, pE):
        self.x=float(xP)
        self.y=float(yP)
        self.libre=pE

# Clase que hace referenia a una posicion en especifico.
class Posicion:
    def __init__(self, xP, yP, pTeta):
        self.x=float(xP)
        self.y=float(yP)
        self.teta=pTeta

# Iniciar grafico de networkx
g=nx.Graph()
# Variables tipo Twist
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
# Distancia entre centro de cuadriculas, 10 debe ser divisible por esta distancia
distanciaCuadricula = 0.25
# Numero de cuadriculas de la escena seleccionada
n = int(10/distanciaCuadricula)
# Arreglo con la informacion de cada una de las casillas
casillas = []
# Posicion actual del robot, debe actualizarse por el topico
posicionActual = Posicion(0,0,0)
# Posicion final del robot, inicialmente se toma como la cuadricula superior derecha con angulo 0
posicionFinal=Posicion(5-distanciaCuadricula/2,5-distanciaCuadricula/2,0)
#Es el diametro de la rueda del Pioneer 3dx en metros.
diametroRueda = 195.3/1000#metros
#Es el radio de la rueda del Pioneer 3dx en metros.
radioRueda = diametroRueda/2 #metros
#Es la distancia entre el punto P y el eje de cada rueda.
l = 0.19 # metros
#Es la variable donde se almacena el valor de p (rho) que equivale a la distancia entre el punto actual y el final.
p = 0
#Es un umbral que se define para indicarle al robot cuando llega al punto final.
umbralP = 0.3
#Es una variable booleana que indica que el robot ha llegado al punto final.
arrivedP = False
#Es la variable donde se guarda la distancia en x entre el punto actual y final.
dx = 0
#Es la variable donde se guarda la distancia en y entre el punto actual y final.
dy = 0
#Es la variable donde se almacena el valor de a (alpha).
a = 0
#Es la variable donde se almacena el valor de b (beta).
b = 0
#Es la variable en donde se guarda el resultado de aplicar la funcion atan2 al triangulo formado por dy y dx.
# Equivale al angulo que se forma en el triangulo formado por el punto actual y final.
t = 0
#Es la constante kp. Debe ser mayor que 0 para que el sistema sea localmente estable.
kp = 0.1 # mayor que 0
#Es la constante ka. ka-kp debe ser mayor que 0 para que el sistema sea localmente estable.
ka = 0.5 # ka-kp mayor que 0
#Es la constante kb. Debe ser menor a 0 para que el sistema sea localmente estable.
kb = -0.01 # menor que 0
#Es la variable utilizada para publicar en el topico motorsVel la velocidad de cada motor.
mot = Float32MultiArray()
#En esta se almacenan las velocidades de cada motor.
mot.data = [0, 0]

#En este metodo se inicializa el nodo, se suscribe a los topicos necesarios, se crea la variable para publicar al
# topico de motorsVel y tambien se lanza el nodo encargado de graficar. Ademas es el metodo encargado de realizar
# las acciones de control necesarias segun la ruta dada para llevar el robot a la posicion final.
def punto2c():
    global posicionActual, g, ruta, pubMot, arrivedP, p, umbralP, kp
    rospy.init_node('punto2c', anonymous=True)
    rospy.Subscriber ('InfoObs', Twist, setObst)
    rospy.Subscriber ('pioneerPosition', Twist, setPositionCallback)
    pubMot = rospy.Publisher ('motorsVel', Float32MultiArray, queue_size=10)
    iniciarGraficador()
    time.sleep (.1)  # Espera a que se actualice informacion de todos los obstaculos
    creadorVerticesCasillas()
    creadorArcos()
    ruta = nx.astar_path(g,numCasillas(posicionActual.x,posicionActual.y),numCasillas(posicionFinal.x,posicionFinal.y) , heuristic=heuristic)
    if len(ruta)>1:
        teta = math.atan2(casillas[ruta[1]].y-casillas[ruta[0]].y, casillas[ruta[1]].x-casillas[ruta[0]].x)
    else:
        teta = math.atan2(posicionFinal.y-casillas[ruta[0]].y, posicionFinal.x-casillas[ruta[0]].x)
    posInter = Posicion(casillas[ruta[0]].x, casillas[ruta[0]].y, teta)
    iRuta = 0
    rate = rospy.Rate (10)
    fin = False
    while (not rospy.is_shutdown()) and (not fin):
        if arrivedP:
            if iRuta == len(ruta)-1:
                iRuta = iRuta + 1
                arrivedP = False
                posInter = posicionFinal
            elif iRuta < len(ruta)-1:
                posInter = Posicion(casillas[ruta[iRuta+1]].x, casillas[ruta[iRuta+1]].y, math.atan2(casillas[ruta[iRuta+1]].y-casillas[ruta[iRuta]].y, casillas[ruta[iRuta+1]].x-casillas[ruta[iRuta]].x))
                iRuta = iRuta + 1
                arrivedP = False
            elif iRuta == len(ruta):
                fin = True
        calcularVelocidades(posInter)
        pubMot.publish(mot)
        rate.sleep()
    mot.data[0] = 0
    mot.data[1] = 0
    pubMot.publish (mot)


# Metodo que arroja la distancia euclidiana entre dos casillas segun su numeramiento (no posicion exacta)
def heuristic(i, j):
    global n
    return math.sqrt((i%n-j%n)**2+(i//n-j//n)**2)

# Busca la casilla mas cercana para dos coordenadas en la distribucion
def numCasillas(x, y):
    dist = float('Inf')
    indice = -1
    for i in range(0, n**2):
        distancia = math.sqrt((casillas[i].x-x)**2+(casillas[i].y-y)**2)
        if distancia<dist:
            indice = i
            dist = distancia
    return indice

# Metodo asociedo a topico para actualizar la posicon del pioneer
def setPositionCallback(pos):
    global posicionActual
    posicionActual.x=pos.linear.x
    posicionActual.y=pos.linear.y
    posicionActual.teta=pos.angular.z


# Metodo asociedo a topico para actualizar la posicion de los obstaculos
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


# Metodo que crea los vertices y casillas del arreglo y del grafo no dirigido
def creadorVerticesCasillas():
    global distanciaCuadricula, n
    xInic=-5.0+distanciaCuadricula/2
    yInic=5.0-distanciaCuadricula/2
    for i in range(0,n*n):
        g.add_node(i)
        mod=i%n
        div=i//n
        nC = Casilla(xInic+mod*distanciaCuadricula,yInic-div*distanciaCuadricula, libre(xInic+mod*distanciaCuadricula,yInic-div*distanciaCuadricula) )
        casillas.append(nC)


# Metodo que crea los arcos del grafo
def creadorArcos():
    for i in range(0,n**2):
        for j in range(i, n**2):
            if j!=i and math.sqrt((i%n-j%n)**2 +(i//n-j//n)**2)<=math.sqrt(2) and libre(casillas[i].x, casillas[i].y) and libre(casillas[j].x, casillas[j].y):
                g.add_edge(i,j)


# Metodo que dice si hay o no obstaculo para cierta posicion (x,y) del mapa, arroja True si no hay obstaculo y False
# de lo contrario
def libre(xCas, yCas):# Si se encuentra un obstaculo en ella
    distanciaCarro = 0.27
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



#Este metodo calcula la distancia p (rho) equivalente a la distancia entre el punto final y el actual y tambien calcula
#las distancias en el eje X y Y entre dichos puntos. Ademas, si p (rho) es menor al umbral definido se le indica al resto
#del codigo que se llego al punto final.
def calcularDistancia(pos):
    global p,dx,dy,arrivedP
    dx = pos.x-posicionActual.x
    dy = pos.y-posicionActual.y
    p = math.sqrt(dx**2 + dy**2)
    if p <= umbralP:
        arrivedP = True

#Aqui se calculan las velocidades de los motores que mueven al robot.
def calcularVelocidades(pos):
    global v,w,p,kp,ka,kb
    calcularDistancia(pos)
    calcularAngulos(pos)
    v = kp*p
    w = ka*a+kb*b
    mot.data[0] = (v-l*w)/radioRueda
    mot.data[1] =(v+l*w)/radioRueda

#En este metodo se calculan los angulos a (alpha) y b (beta). Si el umbral se cumplio, las variables p (rho)
#y a (alpha) se igualan a 0 para permitirle al robot girar y lograr orientarse de manera correcta.
def calcularAngulos(pos):
    global p,a,b,t,arrivedP
    t = math.atan2(dy,dx)
    if not arrivedP:
        a = -posicionActual.teta + t
    else:
        p = 0
        a = 0
    b = posicionActual.teta-pos.teta - a
    if b>math.pi:
        b = b-2*math.pi
    elif b<-math.pi:
        b = 2+math.pi+b

# Metodo que ejecuta nodo graficador usanto herramiento roslaunch, crea un nuevo proceso.
def iniciarGraficador():
    package = 'taller3_4'
    script = 'graficador.py'
    node = roslaunch.core.Node (package, script)
    launch = roslaunch.scriptapi.ROSLaunch ()
    launch.start ()
    process = launch.launch (node)

# Metodo main, mira si existen parametros para la posicion final deseada y ejecuta el metodo principal
if __name__ == '__main__':
    try:
        if len (sys.argv) > 3:
            try:
                posicionFinal.x = float (sys.argv[1])
                posicionFinal.y = float (sys.argv[2])
                posicionFinal.teta = float (sys.argv[3])
            except ValueError:
                pass
        punto2c()
    except rospy.ROSInterruptException:
        pass