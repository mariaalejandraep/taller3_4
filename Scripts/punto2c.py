#!/usr/bin/env python
#Se importan las librerias necesarias junto con los mensajes a utilizar
import rospy
import threading
import math
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import os
import sys
import matplotlib.pyplot as plt
import roslaunch
import networkx as nx
import matplotlib.pyplot as plt



#PARAMETROS DE PUNTO 4 TALLER 2
#Es el diametro de la rueda del Pioneer 3dx en metros.
diametroRueda = 195.3/1000#metros

#Es el radio de la rueda del Pioneer 3dx en metros.
radioRueda = diametroRueda/2 #metros

#Es la distancia entre el punto P y el eje de cada rueda.
l = 0.19 #metros

#Es la variable donde se almacena el valor de p (rho) que equivale
#a la distancia entre el punto actual y el final.
p = 0

#Es un umbral que se define para indicarle al robot cuando llega al punto final.
umbralP = 0.5

#Es una variable booleana que indica que el robot ha llegado al punto final.
umbral = True

#Es la variable donde se guarda la distancia en x entre el punto actual y final.
dx = 0

#Es la variable donde se guarda la distancia en y entre el punto actual y final.
dy = 0

#Es la variable donde se almacena el valor de a (alpha).
a = 0

#Es la variable donde se almacena el valor de b (beta).
b = 0

#Es la variable en donde se guarda el resultado de aplicar la funcion atan2 al triangulo formado por
#dy y dx. Equivale al angulo que se forma en el triangulo formado por el punto actual y final.
t = 0

#Es la constante kp. Debe ser mayor que 0 para que el sistema sea localmente estable.
kp = 0.02 # mayor que 0

#Es la constante ka. ka-kp debe ser mayor que 0 para que el sistema sea localmente estable.
ka = 0.4 # ka-kp mayor que 0

#Es la constante kb. Debe ser menor a 0 para que el sistema sea localmente estable.
kb = -0.01 # menor que 0

#Es la variable utilizada para publicar en el topico motorsVel la velocidad de cada motor.
mot = Float32MultiArray()
#En esta se almacenan las velocidades de cada motor.
mot.data = [0, 0]

#Es la variable en donde se almacena la posicion y orientacion actual del robot.
twistInfo = Twist()

#Es la variable en donde se almacena la posicion inicial del robot.
posInicial = [0, 0, -math.pi]

#Es la variable en donde se guarda la posicion actual del robot.
posActual = posInicial

#Es la variable en donde se guarda la posicion final del robot.
posFinal = [2, 2, -math.pi]





#PARAMETROS DE PUNTO 2

#Iniciar grafico de networkx
g=nx.Graph()



#Variables tipo Twist
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
#Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 5.
twistInfoPioneer = Twist()



#ARRAYS
#Almacena posiciones de coordenadas
Equivalente=[]
#ALmacena conexiones entre nodos
Links=[]



#CONTADORES
#Contador creador de
contador1=-1
#Contaor que determina posicion inicial del robot
contador_Inicial=0
#INDICADOR PARA EMPEZAR MOVIMIENTO
MUEVETE=0
#Contador para determinar cambio de posicion de path
contador_path=1


Nodo_Inicial=0 #Nodo en donde esta ubicado pioneer
Nodo_Final=0 #Nodo destino de pioneer final
distanciaCuadricula=0.5 #Distancias entre cuadriculas de grafo
n=int(10/distanciaCuadricula) #Acordarse que 10 debe ser divisible por distanciaCuadricula


# Se crea una clase tipo casilla para cada casilla de division del mapa
class Casilla:
    def __init__(self, xP, yP):
        self.x=float(xP)
        self.y=float(yP)



#En este metodo se inicializa el nodo, se suscribe a los topicos necesarios, se crea la variable
#para publicar al topico de motorsVel y tambien se lanza el nodo encargado de graficar.
def punto2_a():
    global twistInfoDim, twistInfoPos
    rospy.init_node('punto2_a', anonymous=True)

    #Se subscribe a los siguientes topicos
    rospy.Subscriber('InfoObs0', Twist, setObst)#Topico que contiene informacion sobre obstaculo 1
    rospy.Subscriber('InfoObs1', Twist, setObst1)
    rospy.Subscriber('InfoObs2', Twist, setObst2)
    rospy.Subscriber('InfoObs3', Twist, setObst3)
    rospy.Subscriber('InfoObs4', Twist, setObst4)
    rospy.Subscriber('pioneerPosition', Twist, setPositionCallback) # Se subscribe al topico de posicion del robot
    rospy.Subscriber('simulationTime', Float32, actualizar)

    #SE publica informacion en los siguientes topicos
    pubMot = rospy.Publisher('motorsVel', Float32MultiArray, queue_size=10)#Se publican velocidades de los motores.

    #Se ejectuta el graficador que permite graficar posicion del robot en tiempo real
    package = 'taller3_4'
    script = 'graficador_punto_2.py'
    node = roslaunch.core.Node(package, script)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)



    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        pubMot.publish(mot)
        rate.sleep()




#FUNCIONES ANCLADAS A TOPICOS SUBSCRITOS
#Info de obstaculo 1
def setObst(posicionObstacle):
    global twistInfoPos
    twistInfoPos1=posicionObstacle

#Info de obstaculo 2
def setObst1(posicionObstacle):
    global twistInfoPos2
    twistInfoPos2=posicionObstacle


#Info de obstaculo 3
def setObst2(posicionObstacle):
    global twistInfoPos3
    twistInfoPos3=posicionObstacle
#

#Info de obstaculo 4
def setObst3(posicionObstacle):
    global twistInfoPos4
    twistInfoPos4=posicionObstacle


#Info de obstaculo 5
def setObst4(posicionObstacle):
    global twistInfoPos5
    twistInfoPos5=posicionObstacle


#BASADO EN TOPICO DE PUNTO ANTERIOS

#Este metodo se llama cada vez que al topico simulationTime se le publica un nuevo dato.
#Es el encargado de actualizar los valores de las velocidades de los motores que se publican.
def actualizar(time):
    global MUEVETE
    if MUEVETE>1:
        calcularVelocidades()


#Este metodo se llama cada vez que se al topico pioneerPosition se le publica un nuevo dato.
#En este se actualiza la posicion del robot.
def setPositionCallback(pos):
    global posInicial
    global twistInfo
    global posActual
    global contador_Inicial
    if contador_Inicial<2:
        posInicial=[twistInfo.linear.x, twistInfo.linear.y, twistInfo.angular.z]
        contador_Inicial=contador_Inicial+1
    elif contador_Inicial<3:
        creadorMatriz()
        contador_Inicial=contador_Inicial+1

    posActual = [twistInfo.linear.x, twistInfo.linear.y, twistInfo.angular.z]
    twistInfo = pos

#METODOS DE TALLER PASADO



#Aqui se calculan las velocidades de los motores que mueven al robot.
def calcularVelocidades():
    global v,w,p,kp,ka,kb
    calcularDistancia()
    calcularAngulos()

    v = kp*p
    w = ka*a+kb*b

    #rospy.loginfo(" v {}".format(v))
    #rospy.loginfo(" w {}".format(w))

    mot.data[0] = (v-l*w)/radioRueda
    mot.data[1] =(v+l*w)/radioRueda

#Este metodo calcula la distancia p (rho) equivalente a la distancia entre el punto final y el actual y tambien calcula
#las distancias en el eje X y Y entre dichos puntos. Ademas, si p (rho) es menor al umbral definido se le indica al resto
#del codigo que se llego al punto final.
def calcularDistancia():
    global p,dx,dy,umbral,camino,Equivalente,contador_path,n

    if umbral and contador_path<(len(camino)-1):
        columna = np.mod(camino[contador_path],n)
        fila = int(np.floor(camino[contador_path]/n))
        posFinal[0] = Equivalente[fila][columna].x
        posFinal[1] = Equivalente[fila][columna].y
        contador_path=contador_path+1
        umbral = False
        print posFinal[0]
        print posFinal[1]

    dx = posFinal[0]-posActual[0]
    dy = posFinal[1]-posActual[1]
    p = math.sqrt((dx)**2 + (dy)**2)

    if p <= umbralP:
        umbral = True
#En este metodo se calculan los angulos a (alpha) y b (beta). Si el umbral se cumplio, las variables p (rho)
#y a (alpha) se igualan a 0 para permitirle al robot girar y lograr orientarse de manera correcta.
def calcularAngulos():
    global p,a,b,t,umbral

    t = math.atan2(dy,dx)

    if not umbral:
        a = -posActual[2] + t
    else:
        p = 0
        a = 0
    b = posActual[2]-posFinal[2] - a

    if (b>math.pi):
        b = b-2*math.pi
    elif (b<-math.pi):
        b = 2+math.pi+b






#METODOS PARA ESTE TALLER

def creadorMatriz():
    global distanciaCuadricula, Equivalente,numeroCasillas, n, Links, g, Nodo_Inicial,Nodo_Final
    xInic=-5.0+np.divide(float(distanciaCuadricula),2.0)
    yInic=5.0-np.divide(float(distanciaCuadricula),2.0)
    for i in range(0,n*n):
        g.add_node(i)
        mod=np.mod(i,n)#columnas de la matriz
        div=int(np.floor(i/n))#filas de la matriz
        if len(Equivalente)<div+1:
            Equivalente.append([])
        coordenadax=xInic+mod*distanciaCuadricula
        coordenaday=yInic-div*distanciaCuadricula
        nuevaCasilla = Casilla(coordenadax,coordenaday)
        dist_inicial_vs_dist_nodo=math.sqrt(np.power((posInicial[0]-coordenadax),2.0)+np.power((posInicial[1]-coordenaday),2.0)) #Distancia entre punto inicial y casilla actual
        dist_final_vs_dist_nodo=math.sqrt(np.power((posFinal[0]-coordenadax),2.0)+np.power((posFinal[1]-coordenaday),2.0)) #Distancia entre punto final y casilla actual
        dist_casilla_diagonal=float(math.sqrt(np.power(distanciaCuadricula/2.0,2.0)+np.power(distanciaCuadricula/2.0,2.0))) #Distancia diagonal de ina casilla
        if dist_inicial_vs_dist_nodo <= dist_casilla_diagonal:
            Nodo_Inicial=i

        elif dist_final_vs_dist_nodo <= dist_casilla_diagonal:
            Nodo_Final=i
        Equivalente[div].append(nuevaCasilla)
    Graficador_network(Nodo_Inicial,Nodo_Final)


def heuristica(i,j): #Metodo calcuala heuristica retorna distancia entre nodos
    divi=int(np.math.floor(i/n)) #fila en el que se encuentra nodo i
    divj=int(np.math.floor(j/n)) #Columna en la que se encuentra nodo j
    modi=np.mod(i,n) #columna de nodo i
    modj=np.mod(j,n) #columna de nodo j
    distancia=math.sqrt(np.power((Equivalente[divi][modi].x-Equivalente[divj][modj].x),2)+np.power((Equivalente[divi][modi].y-Equivalente[divj][modj].y),2))
    return distancia

#Metodo que define la red sobre la cual se trabajara
def Graficador_network(nodoInicial,nodoFinal):
    global contador1,Links, g, n, MUEVETE,camino
    for h in range(0,n*n*n*n):

        if contador1<n*n-1:
            contador1=contador1+1
            div2=int(np.floor((h)/(n*n)))
        else:
            contador1=0
            div2=int(np.floor((h+1)/(n*n)))


        if len(Links)<div2+1:
            Links.append([])

        if conexion(div2,contador1)==True:
            Links[div2].append(1)
            g.add_edge(div2,contador1)
        else:
            Links[div2].append(0)

    camino=A_estrella(nodoInicial,nodoFinal,g)
    MUEVETE=2

#Metodo que obiene el path hacia final considerando algoritmo a estrella
def A_estrella(nodoInicial,nodoFinal,g):
    path=nx.astar_path(g,nodoInicial,nodoFinal,heuristica)
    return path


def conexion(i,j):#como variables entran dos numeros relacionados con cada casilla
    divi=int(np.math.floor(i/n)) #fila en el que se encuentra nodo i
    divj=int(np.math.floor(j/n)) #Columna en la que se encuentra nodo j
    modi=np.mod(i,n) #columna de nodo i
    modj=np.mod(j,n) #columna de nodo j
    dist=np.abs(divi-divj)+np.abs(modi-modj)
    if dist>1:
        return False
    else:
        #Distancias limites a cada obstaculo
        dist_limite_1=((twistInfoPos1.linear.z))+math.sqrt(np.power(distanciaCuadricula/2.0,2.0)+np.power(distanciaCuadricula/2.0,2.0)) #Distancia limite a obstaculo 1
        dist_limite_2=((twistInfoPos2.linear.z))+math.sqrt(np.power(distanciaCuadricula/2.0,2.0)+np.power(distanciaCuadricula/2.0,2.0)) #Distancia limite a obstaculo 2
        dist_limite_3=((twistInfoPos3.linear.z))+math.sqrt(np.power(distanciaCuadricula/2.0,2.0)+np.power(distanciaCuadricula/2.0,2.0)) #Distancia limite a obstaculo 3
        dist_limite_4=((twistInfoPos4.linear.z))+math.sqrt(np.power(distanciaCuadricula/2.0,2.0)+np.power(distanciaCuadricula/2.0,2.0)) #Distancia limite a obstaculo 4
        dist_limite_5=((twistInfoPos5.linear.z))+math.sqrt(np.power(distanciaCuadricula/2.0,2.0)+np.power(distanciaCuadricula/2.0,2.0)) #Distancia limite a obstaculo 5


        #Distancias a obstaculos de casilla i considerando distancia limite
        disti_obst_1=math.sqrt(np.power(Equivalente[divi][modi].x-twistInfoPos1.linear.x,2.0)+np.power(Equivalente[divi][modi].y-twistInfoPos1.linear.y,2.0))-dist_limite_1-2.0*l #Diferencia entre limite 1 y distancia de casilla i a obstaculo 1
        disti_obst_2=math.sqrt(np.power(Equivalente[divi][modi].x-twistInfoPos2.linear.x,2.0)+np.power(Equivalente[divi][modi].y-twistInfoPos2.linear.y,2.0))-dist_limite_2-2.0*l #Diferencia entre limite 2 y distancia de casilla i a obstaculo 2
        disti_obst_3=math.sqrt(np.power(Equivalente[divi][modi].x-twistInfoPos3.linear.x,2.0)+np.power(Equivalente[divi][modi].y-twistInfoPos3.linear.y,2.0))-dist_limite_3-2.0*l #Diferencia entre limite 3 y distancia de casilla i a obstaculo 3
        disti_obst_4=math.sqrt(np.power(Equivalente[divi][modi].x-twistInfoPos4.linear.x,2.0)+np.power(Equivalente[divi][modi].y-twistInfoPos4.linear.y,2.0))-dist_limite_4-2.0*l #Diferencia entre limite 4 y distancia de casilla i a obstaculo 4
        disti_obst_5=math.sqrt(np.power(Equivalente[divi][modi].x-twistInfoPos5.linear.x,2.0)+np.power(Equivalente[divi][modi].y-twistInfoPos5.linear.y,2.0))-dist_limite_5-2.0*l #Diferencia entre limite 5 y distancia de casilla i a obstaculo 5

        #Distancias a obstaculos de casilla j considerando distancia limite
        distj_obst_1=math.sqrt(np.power(Equivalente[divj][modj].x-twistInfoPos1.linear.x,2.0)+np.power(Equivalente[divj][modj].y-twistInfoPos1.linear.y,2.0))-dist_limite_1-2.0*l #Diferencia entre limite 1 y distancia de casilla j a obstaculo 1
        distj_obst_2=math.sqrt(np.power(Equivalente[divj][modj].x-twistInfoPos2.linear.x,2.0)+np.power(Equivalente[divj][modj].y-twistInfoPos2.linear.y,2.0))-dist_limite_2-2.0*l #Diferencia entre limite 2 y distancia de casilla j a obstaculo 1
        distj_obst_3=math.sqrt(np.power(Equivalente[divj][modj].x-twistInfoPos3.linear.x,2.0)+np.power(Equivalente[divj][modj].y-twistInfoPos3.linear.y,2.0))-dist_limite_3-2.0*l #Diferencia entre limite 3 y distancia de casilla j a obstaculo 1
        distj_obst_4=math.sqrt(np.power(Equivalente[divj][modj].x-twistInfoPos4.linear.x,2.0)+np.power(Equivalente[divj][modj].y-twistInfoPos4.linear.y,2.0))-dist_limite_4-2.0*l #Diferencia entre limite 4 y distancia de casilla j a obstaculo 1
        distj_obst_5=math.sqrt(np.power(Equivalente[divj][modj].x-twistInfoPos5.linear.x,2.0)+np.power(Equivalente[divj][modj].y-twistInfoPos5.linear.y,2.0))-dist_limite_5-2.0*l #Diferencia entre limite 5 y distancia de casilla j a obstaculo 1

        #Distancias minima a obstaculo
        dist_obst=np.amin([disti_obst_1,disti_obst_2,disti_obst_3,disti_obst_4,disti_obst_5,distj_obst_1,distj_obst_2,distj_obst_3,distj_obst_4,distj_obst_5])


        if (dist_obst)<=(0.0):
            return False

        else:
            return True






#METODO DE INICIALIZACION
if __name__ == '__main__':

    try:
        if len(sys.argv) > 1:
            try:
                global posFinal
                # lee los argumentos que acompanan el rosrun y los guarda como la posicion final
                pos1= float(sys.argv[1])
                pos2= float(sys.argv[2])
                pos3= float(sys.argv[3])
                posFinal = [pos1,pos2,pos3]
            except ValueError:
                pass
        punto2_a()

    except rospy.ROSInterruptException:
        pass

