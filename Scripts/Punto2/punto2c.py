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
distanciaCuadricula = 0.5#.8 #1
# Numero de cuadriculas de la escena seleccionada
n = int(80/distanciaCuadricula)
# Arreglo con la informacion de cada una de las casillas
casillas = []
# Posicion actual del robot, debe actualizarse por el topico
posicionActual = Posicion(0,0,0)
# Posicion final del robot, inicialmente se toma como la cuadricula superior derecha con angulo 0
posicionFinal=Posicion(40-distanciaCuadricula/2,40-distanciaCuadricula/2,math.pi/2)
#Es el diametro de la rueda del Pioneer 3dx en metros.
diametroRueda = 195.3/1000#metros
#Es el radio de la rueda del Pioneer 3dx en metros.
radioRueda = diametroRueda/2 #metros
#Es la distancia entre el punto P y el eje de cada rueda.
l = 0.19 # metros
#Es la variable donde se almacena el valor de p (rho) que equivale a la distancia entre el punto actual y el final.
p = 0
#Es un umbral que se define para indicarle al robot cuando llega al punto final.
umbralP = math.sqrt ((0.51 / 2) ** 2 + 0.41 ** 2) # distanciaCuadricula/2
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
kp = 0.4 #0.4 # mayor que 0, antes era 0.1
#Es la constante ka. ka-kp debe ser mayor que 0 para que el sistema sea localmente estable.
ka = 0.5 # 1 # ka-kp mayor que 0, antes era 0.5
#Es la constante kb. Debe ser menor a 0 para que el sistema sea localmente estable.
kb = 0.01 # menor que 0, era negativo -0.1
#Es la variable utilizada para publicar en el topico motorsVel la velocidad de cada motor.
mot = Float32MultiArray()
#En esta se almacenan las velocidades de cada motor.
mot.data = [0, 0]
# Senal para saber si ya se esta corriendo la simulacion de ROS
empezar = False



#En este metodo se inicializa el nodo, se suscribe a los topicos necesarios, se crea la variable para publicar al
# topico de motorsVel y tambien se lanza el nodo encargado de graficar. Ademas es el metodo encargado de realizar
# las acciones de control necesarias segun la ruta dada para llevar el robot a la posicion final.
def punto2c():
    global posicionActual, g, ruta, pubMot, arrivedP, p, umbralP, kp, kb, ka, empezar, pedal
    # Se crean el nodo
    rospy.init_node('punto2c', anonymous=True)

    rospy.Subscriber ('InfoObs', Twist, setObst)
    # Se suscribe a al topico de la informacion de los obstaculos
    rospy.Subscriber ('pioneerPosition', Twist, setPositionCallback)
    # Se suscribe a al topico de la informacion de la posicion del pioneer
    pubMot = rospy.Publisher ('motorsVel', Float32MultiArray, queue_size=10)
    # Se espera a que se publique por primera vez a traves del topico
    while not empezar:
        empezar = empezar or False
    # Como medida de seguridad se espera .1 segundos para asegurarse de que si se actulizaron los valores de los
    # obstaculos
    time.sleep (.1)
    # Ejecuta el metodo para iniciar el nodo graficador con ROSlaunch
    iniciarGraficador()
    # Se crean los vertices y casillas del grafo y arreglo respectivamente
    creadorVerticesCasillas()
    # Se crean los arcos del grafo
    creadorArcos()
    # Se emplea el metodo de la libreria networkx para sacar la ruta de nodos A*, a este metodo es necesario
    # suministrarle el nombre del nodo inicial, final y un parametro que haga referencia a un metodo que calcule la
    # heuristica entre dos nodos
    ruta = nx.astar_path(g,numCasillas(posicionActual.x,posicionActual.y),numCasillas(posicionFinal.x,posicionFinal.y) , heuristic=heuristic)
    # En caso de que la ruta este compuesta por mas de un nodo calcula el teta adecuado para que la primera posicion
    # termine orientada a la siguiente casilla, de lo contrario, la orienta al punto final
    if len(ruta)>1:
        teta = math.atan2(casillas[ruta[1]].y-casillas[ruta[0]].y, casillas[ruta[1]].x-casillas[ruta[0]].x)
    else:
        teta = math.atan2(posicionFinal.y-casillas[ruta[0]].y, posicionFinal.x-casillas[ruta[0]].x)
    # Crea la primera posicion
    posInter = Posicion(casillas[ruta[0]].x, casillas[ruta[0]].y, teta)
    # Inicializa el contador que define el punto de la ruta en la que se encuentra
    iRuta = 0
    # Tasa a la que se debe publicar en el nodo del movimiento del pioneeer
    rate = rospy.Rate (10)
    # Variable booleana que define que se llego al punto final
    fin = False
    while (not rospy.is_shutdown()) and (not fin):
        # Entra al siguiente condicional en caso de que se halla llegado a uno de los puntos intermedios de la ruta
        if arrivedP:
            # Entra al siguiente condicional en caso de que halla llegado al punto final de la ruta, debe encaminarse a
            # posicion final del camino
            if iRuta == len(ruta)-1:
                iRuta = iRuta + 1
                arrivedP = False
                posInter = posicionFinal
                # Se modifica el umbral para que de igual forma llega mas cercano al punto final
                umbralP = 0.1
            # Entra al siguiente condicional en caso de que halla llegado a un punto intermedio de la ruta
            elif iRuta < len(ruta)-1:
                posInter = Posicion(casillas[ruta[iRuta+1]].x, casillas[ruta[iRuta+1]].y, math.atan2(casillas[ruta[iRuta+1]].y-casillas[ruta[iRuta]].y, casillas[ruta[iRuta+1]].x-casillas[ruta[iRuta]].x))
                iRuta = iRuta + 1
                arrivedP = False
            # Entra de que halla llegado a la poscion final
            elif iRuta == len(ruta):
                # Debido a que llega a la posicon final se modifica el Kb para que modifique su orientacion a la final
                kb = -0.06
                if abs( posicionFinal.teta - posicionActual.teta ) < 0.1:
                    # En caso que la orientacion tenga un error menor a los 0.1 radianes en la poscion final termina procedimiento
                    fin = True
        # En cada iteracion calcula las velocidades segun el punto final que se le pase
        calcularVelocidades(posInter)
        # Publica en el topico la velocidad de los motores requerida
        pubMot.publish(mot)
        # Espera a que se cumpla la tasa de tiempo
        rate.sleep()
    # En caso de terminar detiene a el pioneer
    mot.data[0] = 0
    mot.data[1] = 0
    pubMot.publish (mot)


# Metodo que arroja la distancia euclidiana entre dos casillas segun su numeramiento (no posicion exacta)
def heuristic(i, j):
    global n
    # Se retorna la distacnai relativa entre casillas por distancia euclidiana
    return math.sqrt((i%n-j%n)**2+(i//n-j//n)**2)


# Busca la casilla mas cercana para dos coordenadas en la distribucion
def numCasillas(x, y):
    global n
    # Inicializa la distancia minima en infinito y el indice de la casilla en -1
    dist = float ('Inf')
    indice = -1
    # Se empieza a buscar la casilla mas cercana, en caso de que competidor sea mas cercano modifica la distancia minima
    # y el indice de la casilla
    for i in range (0, n ** 2):
        distancia = math.sqrt ((casillas[i].x - x) ** 2 + (casillas[i].y - y) ** 2)
        if distancia < dist:
            indice = i
            dist = distancia
    return indice


# Metodo asociedo a topico para actualizar la posicon del pioneer
def setPositionCallback(pos):
    global posicionActual
    # Se modifica las cordedanadas de el objeto de posicion del pioneer
    posicionActual.x=pos.linear.x
    posicionActual.y=pos.linear.y
    posicionActual.teta=pos.angular.z


# Metodo asociedo a topico para actualizar la posicion de los obstaculos
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


# Metodo que crea los vertices y casillas del arreglo y del grafo no dirigido
def creadorVerticesCasillas():
    global distanciaCuadricula, n
    # Las variables a continuacion definen desde que posicion se empieza a discretizar la escena
    xInic=-40.0+distanciaCuadricula/2
    yInic=40.0-distanciaCuadricula/2
    # El ciclo empieza a generar la matriz de casillas donde se guarda la discretizacion y anade el nodo al grafo.
    for i in range(0,n*n):
        g.add_node(i)
        mod=i%n
        div=i//n
        nC = Casilla(xInic+mod*distanciaCuadricula,yInic-div*distanciaCuadricula, libre(xInic+mod*distanciaCuadricula,yInic-div*distanciaCuadricula) )
        casillas.append(nC)


# Crea los arcos entre los vertices creados en el grafo, solo es necesario revisar los nodos que pueden ser vecinos, en
# de que ambos esten vacios genera el arco.
def creadorArcos():
    global n
    # El ciclo revisa cada uno de los nodos del grafo
    for i in range(0, n**2):
        # Se revisa primero si la casilla actual esta libre de obstaculos, de lo contrario no es necesario generar arcos
        if casillas[i].libre:
            c = i%n
            f = i//n
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
                if cP in range(0, n) and fP in range(0, n) and casillas[fP*n+cP].libre:
                    g.add_edge (i, fP*n+cP)




# Metodo que indica si cierta posicion en la escena esta ocupada por alguno de los obstaculos, devuelve True en caso de
# que no haya obstaculo en ese punto y False de lo contrario
def libre(xCas, yCas):# Si se encuentra un obstaculo en ella
    # Dependiendo de si la discretizacion de las casillas es fina con respecto al tamano del robot se toma como
    # distancia minima la distancia maxima entre el punto P y el borde del Pioneer o si es grande se toma como la
    # distancia de la diagonal entre el el centro una casilla y una de sus esquinas
    if math.sqrt ((0.51 / 2) ** 2 + 0.41 ** 2) > distanciaCuadricula / math.sqrt (2):
        distanciaCarro = math.sqrt ((0.51 / 2) ** 2 + 0.41 ** 2) # 0.27
    else:
        distanciaCarro = distanciaCuadricula / math.sqrt (2)
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



#Este metodo calcula la distancia p (rho) equivalente a la distancia entre el punto final y el actual y tambien calcula
#las distancias en el eje X y Y entre dichos puntos. Ademas, si p (rho) es menor al umbral definido se le indica al resto
#del codigo que se llego al punto final.
def calcularDistancia(pos):
    global p,dx,dy,arrivedP
    # Se define ro
    dx = pos.x-posicionActual.x
    dy = pos.y-posicionActual.y
    p = math.sqrt(dx**2 + dy**2)
    # Si ro es menor que el umbral se asume que ya se llego al punto adecuado y cambio la variable booleana de llegada
    if p <= umbralP:
        arrivedP = True

#Aqui se calculan las velocidades de los motores que mueven al robot.
def calcularVelocidades(pos):
    global v,w,p,kp,ka,kb
    # Se llama al metodo para actualizar ro
    calcularDistancia(pos)
    # se llama al metodo para actualizar alfa y beta
    calcularAngulos(pos)
    # se define la velocidad lineal y angular que se debe tener segun las constantes de sistema de control
    v = kp*p
    w = ka*a+kb*b
    # Se modifican las variables que se deben publicar en el topico
    mot.data[0] = (v-l*w)/radioRueda
    mot.data[1] =(v+l*w)/radioRueda

#En este metodo se calculan los angulos a (alpha) y b (beta). Si el umbral se cumplio, las variables p (rho)
#y a (alpha) se igualan a 0 para permitirle al robot girar y lograr orientarse de manera correcta.
def calcularAngulos(pos):
    global p,a,b,t,arrivedP
    t = math.atan2(dy,dx)
    # Se define si alfa y ro deben ser cero o no dependiendo de si se llego al umbral
    if not arrivedP:
        a = -posicionActual.teta + t
    else:
        p = 0
        a = 0
    # Se restringe el valor de alfa entre -pi y pi
    if a > math.pi:
        while a > math.pi:
            a = a - 2 * math.pi
    elif a <= -math.pi:
        while a <= -math.pi:
            a = a + 2 * math.pi
    # Se restringe el valor de beta entre -pi y pi
    b = posicionActual.teta-pos.teta - a
    if b > math.pi:
        while b > math.pi:
            b = b - 2 * math.pi
    elif b <= -math.pi:
        while b <= -math.pi:
            b = b + 2 * math.pi


# Metodo que ejecuta nodo graficador usanto herramiento roslaunch, crea un nuevo proceso.
def iniciarGraficador():
    # Se define el paquete y nodo que se deben ejecutar
    package = 'taller3_4'
    script = 'graficador.py'
    node = roslaunch.core.Node (package, script)
    launch = roslaunch.scriptapi.ROSLaunch ()
    launch.start ()
    process = launch.launch (node)

# Metodo main, mira si existen parametros para la posicion final deseada y ejecuta el metodo principal
if __name__ == '__main__':
    try:
        # En caso de que se pasen tres parametros tipo numero se ajusta la nueva posicion final deseada
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