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
posicionFinal=Posicion(40-distanciaCuadricula/2,40-distanciaCuadricula/2,math.pi/2)
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
kp = 0.4 # mayor que 0
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

# En este metodo se inicializa el nodo, se suscribe a los topicos necesarios, se crea la variable para publicar al
# topico de motorsVel y tambien se lanza el nodo encargado de graficar. De igual forma ejecuta el metodo para el RRT y
# antes de iniciar la accion de control muestra la ruta generada por el arbol. Es necesario cerrar la grafica de la ruta
# generada para luego empezar a generar la accion de control y graficar la trayectoria real del robot.
def punto2d():
    global posicionActual, g, ruta, pubMot, arrivedP, p, umbralP, kp, kb, posicionActual, empezar, pedal
    rospy.init_node('punto2e', anonymous=True)
    rospy.Subscriber ('pioneerPosition', Twist, setPositionCallback)
    rospy.Subscriber ('InfoObs', Twist, setObst)
    pubMot = rospy.Publisher ('motorsVel', Float32MultiArray, queue_size=10)
    while not empezar:
        empezar = empezar or False
    time.sleep(0.1)
    RRT(posicionActual.x, posicionActual.y, posicionFinal.x, posicionFinal.y)#Se realiza el metodo RRT para expanncion rapida de arboles
    ruta = trackRoute() #SE obtiene la ruta hacia el destino final con el metodo track
    visualizacionPrevia(ruta)#SE grafica la ruta y todos los nodos creados con aquellas casillas con obstaculos
    iniciarGraficador()#SE inicializa grafica de tiempo real
    if len(ruta)>1:
        teta = math.atan2(casillasRRT[ruta[1]].y-casillasRRT[ruta[0]].y, casillasRRT[ruta[1]].x-casillasRRT[ruta[0]].x)
    else:
        teta = math.atan2(posicionFinal.y-casillasRRT[ruta[0]].y, posicionFinal.x-casillasRRT[ruta[0]].x)
    posInter = Posicion(casillasRRT[ruta[0]].x, casillasRRT[ruta[0]].y, teta)#SE almacena primera posicion
    iRuta = 0 #SE define contador para cambiar de ruta una vez llegado a un nodo
    rate = rospy.Rate (10)
    fin = False #SE inicializa en false para frenar nodo
    while (not rospy.is_shutdown()) and (not fin):
        # Entra al siguiente condicional en caso de que se halla llegado a uno de los puntos intermedios de la ruta
        if arrivedP:
            if iRuta == len(ruta)-1:#SE verifica que se este en la ultima posicion de la ruta
                iRuta = iRuta + 1#Se suma en el contador para recorrer el arreglo de la ruta dada
                arrivedP = False#SE pone en false hasta llegar hasta la siguiente posicion
                posInter = posicionFinal#SE modifica posicion intermedia a la posicion final
                umbralP = 0.1#SE redefine el umbral para este ultimo paso
            elif iRuta < len(ruta)-1:#SE verifica que aun no se haya terminado la ruta
                posInter = Posicion(casillasRRT[ruta[iRuta+1]].x, casillasRRT[ruta[iRuta+1]].y, math.atan2(casillasRRT[ruta[iRuta+1]].y-casillasRRT[ruta[iRuta]].y, casillasRRT[ruta[iRuta+1]].x-casillasRRT[ruta[iRuta]].x))
                iRuta = iRuta + 1#Se suma en el contador para recorrer el arreglo de la ruta dada
                arrivedP = False#Se suma en el contador para recorrer el arreglo de la ruta dada
                print iRuta
            elif iRuta == len(ruta):#Si se llega  al nodo final se modifica kb para ser mas exactos
                kb = -0.06
                print abs (posicionFinal.teta - posicionActual.teta),
                if abs (posicionFinal.teta - posicionActual.teta) < 0.1:
                    fin = True#Si se llega a la posicion final el nodo frena
        calcularVelocidades(posInter)#SE ejecuta calcular velocidades
        pubMot.publish(mot)# SE publican velocidades en motor
        rate.sleep()
    mot.data[0] = 0
    mot.data[1] = 0
    pubMot.publish (mot)

# Metodo asociedo a topico para actualizar la posicon del pioneer
def setPositionCallback(pos):
    global posicionActual
    # Se modifica las cordedanadas de el objeto de posicion del pioneer
    posicionActual.x=pos.linear.x
    posicionActual.y=pos.linear.y
    posicionActual.teta=pos.angular.z




# Metodo asociedo a topico para actualizar la posicion de los obstaculos
def setObst(posicionObstacle):
    global twistInfoPos1, twistInfoPos2, twistInfoPos3, twistInfoPos4, twistInfoPos5,  empezar
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
    if not empezar and twistInfoPos1.linear.z!=0 and twistInfoPos2.linear.z!=0 and twistInfoPos3.linear.z!=0 and twistInfoPos4.linear.z!=0:
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
        xAleatorio = random.randrange(-4000,4000,1)/100#Coordenada x aleatoria
        yAleatorio = random.randrange(-4000,4000,1)/100#Coordenada y aleatoria
        posCasillaCercana = posCasillaMasCercana(xAleatorio, yAleatorio)#Se determnina la casilla mas cercana al punto aleatorio creado entre los nodois ya existentes
        xCercana = casillasRRT[posCasillaCercana].x#Se asigna la coordenada x de la casilla mas cercana a una variable
        yCercana = casillasRRT[posCasillaCercana].y#Se asigna la coordenada y de la casilla mas cercana a una variable
        teta = math.atan2((yAleatorio-yCercana),(xAleatorio-xCercana)) #Angulo que permite delimitar coordenada x e y de acuerdo con el step dado
        xPropuesto = xCercana + step * math.cos(teta) #Coordenada x propuesta dada la distancia step y el nodo mas cercano
        yPropuesto = yCercana + step * math.sin(teta)#Coordenada x propuesta dada la distancia step y el nodo mas cercano
        if libre(xPropuesto, yPropuesto): #Se verifica que la distancia entre el nuevo nodo y el centro del obstaculo mas cercano sea mayor que el radio del obstaculo y la diagonal del robot
            g.add_node(contador)#Si se cumple lo anterior se aniaade el nodo
            casillasRRT.append(Casilla(xPropuesto, yPropuesto))#SE crea una casilla en la posicion contador con las coordenadas del nodo
            g.add_edge(posCasillaCercana,contador)#SE agrega un arco entre la casilla mas cercana y el contador que representa la posicion en el vector casillasRRT en donde esta la casilla con las coordenadas delimitadas
            track.append (posCasillaCercana)#Se almacena la posicion de la cual provino el ultiomo nodo
            contador = contador + 1#Se incementa el contador debido a que se quiere un siguiente nodo
            if math.sqrt((xFin-xPropuesto)**2+(yFin-yPropuesto)**2)<=radioError:#Si se esta demadado cerca del punto final se acaba el while
                llego = True#SE acaba el while
        else:
            xDescartados.append(xPropuesto)#SE almacenan los nodos descartados para despues graficarlos
            yDescartados.append(yPropuesto)
    print "Cantidad final de nodos: ",contador#Se imprime el numero de nodos creados


# Retorna la posicicion en el arreglo casillasRTT de la casilla mas cercana a la posicion (x,y) dada por parametro
def posCasillaMasCercana(x, y):
    global casillasRRT
    distancia = float('Inf')
    numCasilla = -1
    for i in range(0, len(casillasRRT)): #Se recorren todos los nodos creados con conexion hasta el momento buscando el mas cercano a la coordenada aleatoria
        casillaActual = casillasRRT[i]
        distCasillaActual = math.sqrt((casillaActual.x-x)**2+(casillaActual.y-y)**2)
        if distCasillaActual < distancia:
            numCasilla = i #
            distancia = distCasillaActual
    return numCasilla #Se retorna la casilla mas cercana al numero aleatorio


# Metodo que dice si hay o no obstaculo para cierta posicion (x,y) del mapa, arroja True si no hay obstaculo y False
# de lo contrario
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
    dx = pos.x-posicionActual.x
    dy = pos.y-posicionActual.y
    p = math.sqrt(dx**2 + dy**2)
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
    while i!=-1:#DEbido a que la proveniencia del punto inicial se define como -1
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
