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
import time
import random



#PARAMETROS DE PUNTO 4 TALLER 2
#Es el diametro de la rueda del Pioneer 3dx en metros.
diametroRueda = 195.3/1000#metros

#Es el radio de la rueda del Pioneer 3dx en metros.
radioRueda = diametroRueda/2 #metros

#Es la distancia entre el punto P y el eje de cada rueda.
#l = math.sqrt(np.power(0.38/2,2)+np.power(0.51/2,2)) #metros
l=0.45/2


#Es la variable donde se almacena el valor de p (rho) que equivale
#a la distancia entre el punto actual y el final.
p = 0

#Es un umbral que se define para indicarle al robot cuando llega al punto final.
umbralP = 0.3

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
kp = 0.1 # mayor que 0 antes era 0.02, despues fue 0.1

#Es la constante ka. ka-kp debe ser mayor que 0 para que el sistema sea localmente estable.
ka = 0.5 # ka-kp mayor que 0 antes era 0.4, despues fue 0.7

#Es la constante kb. Debe ser menor a 0 para que el sistema sea localmente estable.
kb = -0.01 # menor que 0

#Es la variable utilizada para publicar en el topico motorsVel la velocidad de cada motor.
mot = Float32MultiArray()


#Vabiable utilizada para mandar posiciones de obstaculos y links
Linksx=Float32MultiArray()
#Vabiable utilizada para mandar posiciones de obstaculos y links
Linksy=Float32MultiArray()
#Vabiable utilizada para mandar posiciones de obstaculos y links
Obsx=Float32MultiArray()
#Vabiable utilizada para mandar posiciones de obstaculos y links
Obsy=Float32MultiArray()


#En esta se almacenan las velocidades de cada motor.
mot.data = [0, 0]

#Se almacenan valiarbles de obs y links
Linksx.data=[]
#Se almacenan valiarbles de obs y links
Linksy.data=[]
#Se almacenan valiarbles de obs y links
Obsx.data=[]
#Se almacenan valiarbles de obs y links
Obsy.data=[]

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

posFinal_Fija=[]# Fijha la posicion final sin importar los nodos



limite=500



#DISTANCIA CUADRICULA
distanciaCuadricula=0.2 #Distancias entre cuadriculas de grafo




casillasRRT = []

n=int(10/distanciaCuadricula) #Acordarse que 10 debe ser divisible por distanciaCuadricula


# Se crea una clase tipo casilla para cada casilla de division del mapa
class Casilla:
    def __init__(self, xP, yP):
        self.x=float(xP)
        self.y=float(yP)







#En este metodo se inicializa el nodo, se suscribe a los topicos necesarios, se crea la variable
#para publicar al topico de motorsVel y tambien se lanza el nodo encargado de graficar.
def punto2_a():
    global twistInfoDim, twistInfoPos, MUEVETE, posInicial,posFinal
    rospy.init_node('punto2_c', anonymous=True)
    #Se subscribe a los siguientes topicos
    rospy.Subscriber('InfoObs0', Twist, setObst)#Topico que contiene informacion sobre obstaculo 1
    rospy.Subscriber('pioneerPosition', Twist, setPositionCallback) # Se subscribe al topico de posicion del robot
    rospy.Subscriber('simulationTime', Float32, actualizar)
    time.sleep(0.2) # Espera a que se actualice informacion de todos los obstaculos
    #SE publica informacion en los siguientes topicos
    RRT (posInicial[0], posInicial[1], posFinal[0],  posFinal[1])
    pubMot = rospy.Publisher('motorsVel', Float32MultiArray, queue_size=10)#Se publican velocidades de los motores.
    #pubLibresx = rospy.Publisher('Libresx', Float32MultiArray,  queue_size=10)#Se publican velocidades de los motores.
    # pubLibresy = rospy.Publisher('Libresy', Float32MultiArray,  queue_size=10)#Se publican velocidades de los motores.
    # pubObsx = rospy.Publisher('Obsx', Float32MultiArray,  queue_size=10)#Se publican velocidades de los motores.
    # pubObsy = rospy.Publisher('Obsy', Float32MultiArray,  queue_size=10)#Se publican velocidades de los motores.
    #SE inicializa el nodo que grafica en tiempo real
    #package = 'taller2_4'
    #script = 'graficador_punto_3.py'
    #node = roslaunch.core.Node(package, script)
    #launch = roslaunch.scriptapi.ROSLaunch()
    #launch.start()
    #process = launch.launch(node)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        pubMot.publish(mot)
        rate.sleep()




#FUNCIONES ANCLADAS A TOPICOS SUBSCRITOS
#Info de obstaculo 1
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
    global p,dx,dy,umbral,camino,Equivalente,contador_path,n,umbralP
    if umbral and contador_path<=(len(camino)-2):
        columna = np.mod(camino[contador_path],n)
        fila = int(np.floor(camino[contador_path]/n))
        posFinal[0] = Equivalente[fila][columna].x
        posFinal[1] = Equivalente[fila][columna].y
        contador_path=contador_path+1
        umbral = False
        print posFinal[0]
        print posFinal[1]
    elif umbral and contador_path==(len(camino)-1): #Esto se realiza para que en su ultima etapa ignore el nodo final y vaya a la pos exacta que se pide
        posFinal[0]=posFinal_Fija[0]
        posFinal[1]=posFinal_Fija[1]
        print posFinal[0]
        print posFinal[1]
        contador_path=contador_path+1
        umbralP=0.05
        umbral = False
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
    if b>math.pi:
        b = b-2*math.pi
    elif b<-math.pi:
        b = 2+math.pi+b


#METODOS PARA ESTE TALLER





def heuristica(i,j): #Metodo calcula heuristica retorna distancia entre nodos
    divi=int(np.math.floor(i/n)) #fila en el que se encuentra nodo i
    divj=int(np.math.floor(j/n)) #Columna en la que se encuentra nodo j
    modi=np.mod(i,n) #columna de nodo i
    modj=np.mod(j,n) #columna de nodo j
    distancia=math.sqrt(np.power((Equivalente[divi][modi].x-Equivalente[divj][modj].x),2)+np.power((Equivalente[divi][modi].y-Equivalente[divj][modj].y),2))
    return distancia


#Metodo que define la red sobre la cual se trabajara
def Graficador_network(nodoInicial,nodoFinal):
    global MUEVETE,camino,Nodo_Actual, limite
    Nodo_Actual=nodoInicial
    while Nodo_Actual!=nodoFinal and w<limite:
        columna=np.mod(Nodo_Actual,n)#columnas de la matriz
        fila=int(np.floor(Nodo_Actual/n))#filas de la matriz
        vecinos=([Equivalente[fila-1][columna-1].numero,Equivalente[fila+1][columna+1].numero,Equivalente[fila][columna-1].numero,Equivalente[fila][columna+1].numero,Equivalente[fila-1][columna].numero,Equivalente[fila+1][columna].numero,Equivalente[fila-1][columna+1].numero,Equivalente[fila+1][columna-1].numero])
        rand_nodo = random.choice(vecinos)
        if j!=i and math.sqrt((i%n-j%n)**2 +(i//n-j//n)**2)<=math.sqrt(2) and conexion(Equivalente[i//n][i%n].x, Equivalente[i//n][i%n].y) and conexion(Equivalente[j//n][j%n].x, Equivalente[j//n][j%n].y):
            g.add_edge(i,j)
        w=w+1

    camino=A_estrella(nodoInicial,nodoFinal,g)
    MUEVETE=2

#Metodo que obiene el path hacia final considerando algoritmo a estrella
def A_estrella(nodoInicial,nodoFinal,g):
    path=nx.astar_path(g,nodoInicial,nodoFinal,heuristica)
    return path


def conexion(xCas,yCas):#como variables entran dos numeros relacionados con cada casilla
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





def RRT(xIni, yIni, xFin, yFin):
    global g, distanciaCuadricula,camino,MUEVETE,nodoInicial,nodoFinal
    g.add_node(0)
    step = distanciaCuadricula
    radioError = 5
    casillasRRT.append(Casilla(xIni,yIni))
    contador = 1
    llego = False
    while not llego and contador != 50000:
        xAleatorio = random.randrange(-500,500,1)/100
        yAleatorio = random.randrange(-500,500,1)/100
        posCasillaCercana = posCasillaMasCercana(xAleatorio, yAleatorio)
        xCercana = casillasRRT[posCasillaCercana].x
        yCercana = casillasRRT[posCasillaCercana].y
        teta = math.atan2((yAleatorio-yCercana),(xAleatorio-xCercana))
        xPropuesto = xCercana + step * math.cos(teta)
        yPropuesto = yCercana + step * math.sin(teta)
        if conexion(xPropuesto, yPropuesto):
            g.add_node(contador)
            casillasRRT.append(Casilla(xPropuesto, yPropuesto))
            g.add_edge(posCasillaCercana,contador)
            contador = contador + 1
            if math.sqrt((xFin-xPropuesto)**2+(yFin-yPropuesto)**2)<=radioError:
                llego = True
        if contador%100==0:
            print contador
    camino=A_estrella(0,len(casillasRRT)-1,g)
    MUEVETE=2



def posCasillaMasCercana(x, y):
    global casillasRRT
    distancia = float('Inf')
    numCasilla = -1
    for i in range(0, len(casillasRRT)):
        casillaActual = casillasRRT[i]
        distCasillaActual = math.sqrt((casillaActual.x-x)**2+(casillaActual.y-y)**2)
        if distCasillaActual<distancia:
            numCasilla = i
            distancia = distCasillaActual
    return numCasilla




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
                posFinal_Fija=[pos1,pos2,pos3]
            except ValueError:
                pass
        punto2_a()

    except rospy.ROSInterruptException:
        pass


