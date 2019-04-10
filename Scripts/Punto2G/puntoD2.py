#!/usr/bin/env python
#Se importan las librerias necesarias junto con los mensajes a utilizar
import rospy, math, sys, roslaunch, time, random
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import networkx as nx
import matplotlib.pyplot as plt
# Clase que representa una casilla, tiene ubicacion o punto que la define (mitad) y si un objeto la cubre o no.
class Casilla:
    def __init__(self, xP, yP):
        self.x=float(xP)
        self.y=float(yP)

# Clase que representa una posicion en el mapa
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
distanciaCuadricula = 0.8
# Numero de cuadriculas de la escena seleccionada
n=int(10/distanciaCuadricula)
# Arreglo con la informacion de cada una de las casillas
casillas=[]
# Posicion actual del robot, debe actualizarse por el topico
posicionActual=Posicion(0,0,0)
# Posicion final del robot, inicialmente se toma como la cuadricula superior derecha con angulos
posicionFinal=Posicion(10-distanciaCuadricula/2,10-distanciaCuadricula/2,math.pi/2)
#Es el diametro de la rueda del Pioneer 3dx en metros.
diametroRueda = 195.3/1000#metros
#Es el radio de la rueda del Pioneer 3dx en metros.
radioRueda = diametroRueda/2 #metros
#Es la distancia entre el punto P y el eje de cada rueda.
l = 0.19 # Metros
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
kp = 0.11 # mayor que 0
#Es la constante ka. ka-kp debe ser mayor que 0 para que el sistema sea localmente estable.
ka = 0.5 # ka-kp mayor que 0
#Es la constante kb. Debe ser menor a 0 para que el sistema sea localmente estable.
kb = -0.01 # menor que 0
#Es la variable utilizada para publicar en el topico motorsVel la velocidad de cada motor.
mot = Float32MultiArray()
#En esta se almacenan las velocidades de cada motor.
mot.data = [0, 0]
# Arreglo de casillas generados por RRT, el primer elemento es la posicion inicial del robot siempre
casillasRRT = []
# Arreglo de cordenadas x propuestas por RTT descartadas por presencia de obstaculos
xDescartados = []
# Arreglo de cordenadas y propuestas por RTT descartadas por presencia de obstaculos
yDescartados = []
# Arreglo que permite rastrear la ruta generada por el RRT
track = []
# Senal para saber si ya se esta corriendo la simulacion de ROS
empezar = False
# Boost para moverse mas rapido en camino antes de dirigirse al punto final
pedal = 1 # 0.08

# En este metodo se inicializa el nodo, se suscribe a los topicos necesarios, se crea la variable para publicar al
# topico de motorsVel y tambien se lanza el nodo encargado de graficar. De igual forma ejecuta el metodo para el RRT y
# antes de iniciar la accion de control muestra la ruta generada por el arbol. Es necesario cerrar la grafica de la ruta
# generada para luego empezar a generar la accion de control y graficar la trayectoria real del robot.
def punto2d():
    global posicionActual, g, ruta, pubMot, arrivedP, p, umbralP, kp, posicionActual, empezar, pedal
    rospy.init_node('punto2d', anonymous=True)
    rospy.Subscriber ('InfoObs', Twist, setObst)
    rospy.Subscriber ('pioneerPosition', Twist, setPositionCallback)
    pubMot = rospy.Publisher ('motorsVel', Float32MultiArray, queue_size=10)
    while not empezar:
        empezar = empezar or False
    time.sleep (.1)  # Espera a que se actualice informacion de todos los obstaculos
    RRT(posicionActual.x, posicionActual.y, posicionFinal.x, posicionFinal.y)
    ruta = trackRoute()#  nx.astar_path(g,0, len(casillasRRT)-1 , heuristic=heuristic)
    visualizacionPrevia(ruta)
    iniciarGraficador()
    if len(ruta)>1:
        teta = math.atan2(casillasRRT[ruta[1]].y-casillasRRT[ruta[0]].y, casillasRRT[ruta[1]].x-casillasRRT[ruta[0]].x)
    else:
        teta = math.atan2(posicionFinal.y-casillasRRT[ruta[0]].y, posicionFinal.x-casillasRRT[ruta[0]].x)
    posInter = Posicion(casillasRRT[ruta[0]].x, casillasRRT[ruta[0]].y, teta)
    iRuta = 0
    rate = rospy.Rate (10)
    fin = False
    print len(ruta)
    while (not rospy.is_shutdown()) and (not fin):
        if arrivedP:
            if iRuta == len(ruta)-1:
                iRuta = iRuta + 1
                arrivedP = False
                posInter = posicionFinal
                pedal = 0
            elif iRuta < len(ruta)-1:
                posInter = Posicion(casillasRRT[ruta[iRuta+1]].x, casillasRRT[ruta[iRuta+1]].y, math.atan2(casillasRRT[ruta[iRuta+1]].y-casillasRRT[ruta[iRuta]].y, casillasRRT[ruta[iRuta+1]].x-casillasRRT[ruta[iRuta]].x))
                iRuta = iRuta + 1
                arrivedP = False
                print iRuta
            elif iRuta == len(ruta):
                fin = True
        calcularVelocidades(posInter)
        pubMot.publish(mot)
        rate.sleep()
    mot.data[0] = 0
    mot.data[1] = 0
    pubMot.publish (mot)

# Metodo asociedo a topico para actualizar la posicon del pioneer
def setPositionCallback(pos):
    global posicionActual
    posicionActual.x=pos.linear.x
    posicionActual.y=pos.linear.y
    posicionActual.teta=pos.angular.z


# Metodo asociedo a topico para actualizar la posicion de los obstaculos
def setObst(posicionObstacle):
    global twistInfoPos1, twistInfoPos2, twistInfoPos3, twistInfoPos4, twistInfoPos5, empezar
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
    if not empezar:
        empezar = True

# Metodo que genera el Rapidly Expanding Random Tree. Genera una secuencia de nodos aleatorios con los que saca una
# linea a al mas cercanos del grafo y saca el punto en esta linea a cierta distancia, en caso de que este punto este
# libre genera el vertice y arco con el nodo ya perteneciente y lo anade al grafo
def RRT(xIni, yIni, xFin, yFin):
    global g, distanciaCuadricula, xDescartados, yDescartados, track
    g.add_node(0)
    track.append(-1)
    step = distanciaCuadricula
    radioError = 1 # .25
    casillasRRT.append(Casilla(xIni,yIni))
    contador = 1
    llego = False
    while not llego: # and contador != 20000:
        xAleatorio = random.randrange(-4000,4000,1)/100
        yAleatorio = random.randrange(-4000,4000,1)/100
        posCasillaCercana = posCasillaMasCercana(xAleatorio, yAleatorio)
        xCercana = casillasRRT[posCasillaCercana].x
        yCercana = casillasRRT[posCasillaCercana].y
        teta = math.atan2((yAleatorio-yCercana),(xAleatorio-xCercana))
        xPropuesto = xCercana + step * math.cos(teta)
        yPropuesto = yCercana + step * math.sin(teta)
        if libre(xPropuesto, yPropuesto):
            g.add_node(contador)
            casillasRRT.append(Casilla(xPropuesto, yPropuesto))
            g.add_edge(posCasillaCercana,contador)
            track.append (posCasillaCercana)
            contador = contador + 1
            if math.sqrt((xFin-xPropuesto)**2+(yFin-yPropuesto)**2)<=radioError:
                llego = True
            if contador%100==0:
                print contador
        else:
            xDescartados.append(xPropuesto)
            yDescartados.append(yPropuesto)
    print "Cantidad final de nodos: ",contador


# Retorna la posicicion en el arreglo casillasRTT de la casilla mas cercana a la posicion (x,y) dada por parametro
def posCasillaMasCercana(x, y):
    global casillasRRT
    distancia = float('Inf')
    numCasilla = -1
    for i in range(0, len(casillasRRT)):
        casillaActual = casillasRRT[i]
        distCasillaActual = math.sqrt((casillaActual.x-x)**2+(casillaActual.y-y)**2)
        if distCasillaActual < distancia:
            numCasilla = i
            distancia = distCasillaActual
    return numCasilla


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
    global v,w,p,kp,ka,kb, pedal
    calcularDistancia(pos)
    calcularAngulos(pos)
    v = kp*p
    w = ka*a+kb*b
    mot.data[0] = (v-l*w)/radioRueda + pedal
    mot.data[1] =(v+l*w)/radioRueda + pedal

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
    if b>=math.pi:
        b = b-2*math.pi
    elif b<-math.pi:
        b = 2+math.pi+b

# Metodo que ejecuta nodo graficador usanto herramiento roslaunch, crea un nuevo proceso.
def iniciarGraficador():
    package = 'taller3_4'
    script = 'graficador2.py'
    node = roslaunch.core.Node (package, script)
    launch = roslaunch.scriptapi.ROSLaunch ()
    launch.start ()
    process = launch.launch (node)


# Metodo que grafica el camino generado por el RTT previo a que se realice la accion de control
def visualizacionPrevia(path):
    global casillasRRT, xDescartados, yDescartados, g
    cordX = []
    cordY = []
    xPath = []
    yPath = []
    for i in range(0, len(casillasRRT)):
        casillaActual = casillasRRT[i]
        if i not in path:
            cordX.append(casillaActual.x)
            cordY.append(casillaActual.y)
        else:
            xPath.append (casillaActual.x)
            yPath.append (casillaActual.y)
    plt.plot(cordX, cordY, 'bo')
    plt.plot(xDescartados, yDescartados, 'ro')
    plt.plot(xPath,yPath,'go')
    plt.show()

# Obtener ruta la ruta del arbol generado por el RRT
def trackRoute():
    global track
    resp = []
    i = len(track)-1
    while i!=-1:
        resp.insert(0, i)
        i =  track[i]
    return resp

# Metodo main, cambio la posicion final en caso de que se pasen por parametro una nueva posicion y ejecuta el
# metodo principal
if __name__ == '__main__':
    try:
        if len (sys.argv) > 3:
            try:
                posicionFinal.x = float (sys.argv[1])
                posicionFinal.y = float (sys.argv[2])
                posicionFinal.teta = float (sys.argv[3])
            except ValueError:
                pass
        punto2d()
    except rospy.ROSInterruptException:
        pass