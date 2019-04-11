#!/usr/bin/env python
import rospy
from pacman.msg import pacmanPos
from pacman.srv import mapService
from pacman.msg import actions
from pacman.msg import cookiesPos
import numpy as np
#Se crea una clase para poder crear colas
class Queue:
    #se inicializa la Queue
    def __init__(self):
        self.items = []
    #revisa si esta vacia
    def isEmpty(self):
        return self.items == []
    #agrega un elemento a la Queue
    def enqueue(self, item):
        self.items.insert(0,item)
    # saca un elementos de la Queue
    def dequeue(self):
        return self.items.pop()
    #Da el tmano de la Queue
    def size(self):
        return len(self.items)
# se crea una clase para poder crear pilas
class Stack:
    #se inicializa el stack
     def __init__(self):
         self.items = []
    #revisa si esta vacio
     def isEmpty(self):
         return self.items == []
    #agrega un elemento al Stack
     def push(self, item):
         self.items.append(item)
    #saca un elemento del Stack
     def pop(self):
         return self.items.pop()
    #da el tamano del Stack
     def size(self):
         return len(self.items)

#variable de tipo actions donde se a guarda la accion de pacman
msg = actions()
#se inicia la accion en -1 para que no se mueva
msg.action = -1

#se crea un arreglo donde se guadaran los obstaculos
obs = []
#se define un valor que sera el rate de spleep del programa
h=(1.0/0.15)
# se inicializan varias variables, stacks, queues para poder realizar bfs
co=cookiesPos()
mapa = None
q=Queue()
ruta=Stack()
pos=[]
posicionPacmanX = 0
posicionPacmanY = 0
posicionCj=0
posicionCi=0
tamanioC=0
pos1=0
pos2=0
var=False
done=False
maxXre=0
maxYre=0
minXre=0
minYre=0
maxMapaY=0
maxMapaX=0
noLlego=999
rutaMin=Stack()
x=0
y=0
posPacman=False
i=0
z=0
# se crean 3 matrices del mismo tamano que el mapa, el primero se iniciliza en false y el resto en 999(un valor grande que representa que no se llego a ese punto)
a = [[False for x in range(maxMapaX+1)] for y in range(maxMapaY+1)]
llegueEnX = [[noLlego for x in range(maxMapaX+1)] for y in range(maxMapaY+1)]
llegueEnY= [[noLlego for x in range(maxMapaX+1)] for y in range(maxMapaY+1)]


#funcion para hacer bfs
def BF (posactx,posacty,posGoalx,posGoalY,j):
    #rospy.loginfo("estoy calculando")
    global q,a,ruta,pos1,pos2,llegueEnY,llegueEnX,posicionPacmanX,posicionPacmanY,retornar,maxMapaY,maxMapaX,noLlego
    a = [[False for x in range(maxMapaY)] for y in range(maxMapaX)]
    llegueEnX = [[noLlego for x in range(maxMapaY)] for y in range(maxMapaX)]
    llegueEnY= [[noLlego for x in range(maxMapaY)] for y in range(maxMapaX)]
    q=Queue()
    ruta=Stack()

    retornar=False
    #se agregan los valores de la posicion actual a la queue
    q.enqueue(posactx)
    q.enqueue(posacty)
    # se expande mientras la queue no este vacia
    while q.isEmpty()==False:
        #saca 2 puntos (x y y)
        pos1=q.dequeue()
        pos2=q.dequeue()
        # ya que se sacaron esos puntos quiere decir que fueron visitados por lo cual se le asigna a la matriz a True en ese punto x,y
        a[pos1+(maxMapaX/2)-1][pos2+(maxMapaY/2)-1]=True
        if pos1==posicionCj[j] and pos2==posicionCi[j]:
            #el while acaba si se llega a la posicion de la galleta actual
            retornar=True
            break

        # se expande hacia todas las direcciones validas
        #verifica si la posicion es valida
        if esValido(pos1+1,pos2):
            #verifica que el punto no haya sido visitado aun
            if (a[pos1+1+(maxMapaX/2)-1][pos2+(maxMapaY/2)-1]==False):
                #lo agrega a la queue
                q.enqueue(pos1+1)
                q.enqueue(pos2)
                # agrega en la matrices LLegueEnX y llegueEnY la posicion desde la cual llego a este punto (el punto desde el cual se expadio)
                llegueEnX[pos1+1+(maxMapaX/2)-1][pos2+(maxMapaY/2)-1]=pos1
                llegueEnY[pos1+1+(maxMapaX/2)-1][pos2+(maxMapaY/2)-1]=pos2
        # hace lo mismo para las demas direcciones
        if esValido(pos1,pos2+1):
            if (a[pos1+(maxMapaX/2)-1][pos2+1+(maxMapaY/2)-1]==False):
                q.enqueue(pos1)
                q.enqueue(pos2+1)
                llegueEnX[pos1+(maxMapaX/2)-1][pos2+1+(maxMapaY/2)-1]=pos1
                llegueEnY[pos1+(maxMapaX/2)-1][pos2+1+(maxMapaY/2)-1]=pos2

        if esValido(pos1-1,pos2):
            if (a[pos1-1+(maxMapaX/2)-1][pos2+(maxMapaY/2)-1]==False):
                q.enqueue(pos1-1)
                q.enqueue(pos2)
                llegueEnX[pos1-1+(maxMapaX/2)-1][pos2+(maxMapaY/2)-1]=pos1
                llegueEnY[pos1-1+(maxMapaX/2)-1][pos2+(maxMapaY/2)-1]=pos2
        if esValido(pos1,pos2-1):
            if (a[pos1+(maxMapaX/2)-1][pos2-1+(maxMapaY/2)-1]==False):
                q.enqueue(pos1)
                q.enqueue(pos2-1)
                llegueEnX[pos1+(maxMapaX/2)-1][pos2-1+(maxMapaY/2)-1]=pos1
                llegueEnY[pos1+(maxMapaX/2)-1][pos2-1+(maxMapaY/2)-1]=pos2

    retornar=True
    return retornar




def definirMovimiento():

    global done, a, posicionPacmanX,posicionPacmanY,i,punto1,punto2,temp,posicionCi,posicionCj,z,rutaMin,ruta
    #se crea un stack con un tamano de 500 (ya que es un numero grande y las rutas nunca llegaran a este tamano)
    w=0
    while w<500:
        rutaMin.push(w)
        w=w+1
    #calculo la ruta hacia todas las galletas
    for j in range (0,tamanioC):

        if BF(posicionPacmanX,posicionPacmanY,posicionCj[j],posicionCi[j],j)==True:
            ruta=Stack()
            punto1=posicionCj[j]
            punto2=posicionCi[j]
            ruta.push(punto1)
            ruta.push(punto2)

            while not (punto1 == posicionPacmanX and punto2 == posicionPacmanY):
                temp=punto1
                #usando las matrices reconstruyo la ruta que me llega hasta la galleta
                punto1= llegueEnX[punto1+(maxMapaX/2)-1][punto2+(maxMapaY/2)-1]
                punto2= llegueEnY[temp+(maxMapaX/2)-1][punto2+(maxMapaY/2)-1]
                #agrego el punto a el Stack
                ruta.push(punto1)
                ruta.push(punto2)
        rospy.loginfo("longitud ruta:{}".format(ruta.size()/2 -1))
        # verifico si la ruta que acabo de calcular es menor a la menor actual, si si lo es calmbio la menor a la nueva
        if ruta.size()<rutaMin.size():
            rutaMin=ruta

            z=j
    rospy.loginfo("longitud ruta minima:{}".format(rutaMin.size()/2 -1))
    done=True


def moverPacman():
    global x,y, posicionPacmanY,posicionPacmanX, msg,ruta,posicionCi,posicionCj
    if not rutaMin.isEmpty():
        #comparo la posicion actual con la posicion sacada del stack para saber cual es mi siguiente movimiento
        y=rutaMin.pop()
        x=rutaMin.pop()
        rX= posicionPacmanX-x
        rY= posicionPacmanY-y
        #cambia msg a la accion dependiendo de la direccion a la que se deberia mover

        if rX==1 and rY==0:
            msg.action=3
        elif rX==-1 and rY==0:
            msg.action=2
        elif rY==1 and rX==0:

            msg.action=1
        elif rY==-1 and rX==0:
            msg.action=0
        else:
            msg.action=-1


    else:
        msg.action=-1


# funcion que actualiza contantemente el numero de galletas
def cookiesCallBack(co):
	global posicionCj,posicionCi,tamanioC

	tamanioC=co.nCookies
	posicionCj=np.arange(tamanioC)
	posicionCi=np.arange(tamanioC)

	for xx in range (0,tamanioC):
		posicionCj[xx]=co.cookiesPos[xx].x
		posicionCi[xx]=co.cookiesPos[xx].y
	pass

#funcion que actualiza constantemente la posicion de pacman
def pacmanPosCallback(msg):
    global posicionPacmanX, posicionPacmanY,var,posPacman

    posicionPacmanX = msg.pacmanPos.x
    posicionPacmanY = msg.pacmanPos.y
    #variable que verifica que ya se haya hecho la primera lectura
    var=True
    #variable para verificar si pacman ya se movio
    posPacman=True

#funcion que revisa si una posicion que entra como parametro es valida
def esValido(actx,acty):
    ret=True
	#recorre y compara la posicion con la posicion de todos los obstaculos del juego
    #revisa si la posicion no supera los limites del mapa
    if actx>=maxXre or acty>=maxYre or actx<minXre or acty<minYre:
        ret=False
    else:
        for w in range (0,len(obs)):
            if actx==obs[w].x and acty==obs[w].y:
                ret=False

    return ret


def pacman_controller_py():
    #inicializo el nodo
    rospy.init_node('pacman_controller_py', anonymous=True)
    #se subscribe a las coordenadas de pacman
    rospy.Subscriber('pacmanCoord0', pacmanPos, pacmanPosCallback)
    #creo variable para publicar en pacmanAction0
    pub = rospy.Publisher('pacmanActions0', actions, queue_size=10)
    #me subscribo a las coordenadas de las galletas
    rospy.Subscriber('cookiesCoord', cookiesPos,cookiesCallBack)


    try:

        mapRequestClient = rospy.ServiceProxy('pacman_world', mapService)
        global msg, mapa, maxMapaX, maxMapaY, maxXre, maxYre, minXre, minYre, h, ruta, q, i , var, done, obs, tamanioC, a, llegueEnX, llegueEnY, noLlego, posicionCi, posicionCj,x,y,posPacman,rutaMin

        mapa = mapRequestClient("Grupo4")
        maxXre=mapa.maxX
        maxYre=mapa.maxY
        minXre= -1*maxXre
        minYre= -1*maxYre
        maxMapaX=(mapa.maxX)*2+1
        maxMapaY=(mapa.maxY)*2+1
        #inicio los obstaculos
        obs = mapa.obs
        #inicio las matrices con los tamanos dados por el mapa
        a = [[False for x in range(maxMapaY)] for y in range(maxMapaX)]
        llegueEnX = [[noLlego for x in range(maxMapaY)] for y in range(maxMapaX)]
        llegueEnY= [[noLlego for x in range(maxMapaY)] for y in range(maxMapaX)]
        rate = rospy.Rate(h)  # 6.666Hz


        while not rospy.is_shutdown():
            # si ya leyo la primera posicion de pacMan y aun no ha calculado la ruta y si hay galletas calcula la ruta
            if (var==True and done==False and tamanioC is not 0):
                definirMovimiento()
                galletaActualX=posicionCj[z]
                galletaActualY=posicionCi[z]


            # si ya calculo la ruta, calcula la ccion que debe realizar y la publica en pacman Actions
            if done:
                if posPacman:
                    moverPacman()

                    pub.publish(msg.action)
                    posPacman=False

                #si ya llego a la posicion de la galleta, detiene a pacman reinicia las variables para que vuelva a calcular
                if posicionPacmanX==galletaActualX and posicionPacmanY==galletaActualY:
                    done=False
                    msg.action=-1
                    pub.publish(msg.action)

                    a = [[False for x in range(maxMapaY)] for y in range(maxMapaX)]
                    llegueEnX = [[noLlego for x in range(maxMapaY)] for y in range(maxMapaX)]
                    llegueEnY= [[noLlego for x in range(maxMapaY)] for y in range(maxMapaX)]
                    while ruta.isEmpty()==False:
                        ruta.pop()
                    while rutaMin.isEmpty()==False:
                        rutaMin.pop()

            rate.sleep()


    except rospy.ServiceException as e:
        print("Error!! Make sure pacman_world node is running ")


if __name__ == '__main__':
    try:
        pacman_controller_py()
    except rospy.ROSInterruptException:
        pass
