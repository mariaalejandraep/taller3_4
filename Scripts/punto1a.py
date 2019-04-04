#!/usr/bin/env python
import rospy
from pacman.msg import pacmanPos
from pacman.srv import mapService
from pacman.msg import actions
from pacman.msg import cookiesPos
import numpy as np
class Queue:
    def __init__(self):
        self.items = []

    def isEmpty(self):
        return self.items == []

    def enqueue(self, item):
        self.items.insert(0,item)

    def dequeue(self):
        return self.items.pop()

    def size(self):
        return len(self.items)
class Stack:
     def __init__(self):
         self.items = []

     def isEmpty(self):
         return self.items == []

     def push(self, item):
         self.items.append(item)

     def pop(self):
         return self.items.pop()

     def peek(self):
         return self.items[len(self.items)-1]

     def size(self):
         return len(self.items)

msg = actions()
msg.action = -1


obs = []

h=1/0.15
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
i=2
a = [[False for x in range(maxMapaX+1)] for y in range(maxMapaY+1)]
llegueEnX = [[noLlego for x in range(maxMapaX+1)] for y in range(maxMapaY+1)]
llegueEnY= [[noLlego for x in range(maxMapaX+1)] for y in range(maxMapaY+1)]
def BF (posactx,posacty,posGoalx,posGoalY):
    rospy.loginfo("estoy calculando")
    global q
    global a
    global ruta
    global pos1
    global pos2,llegueEnY,llegueEnX,posicionPacmanX,posicionPacmanY
    global retornar
    retornar=False
    q.enqueue(posactx)
    q.enqueue(posacty)

    while q.isEmpty()==False:


        pos1=q.dequeue()
        pos2=q.dequeue()
        a[pos1+(maxMapaX/2)-1][pos2+(maxMapaY/2)-1]=True
        if pos1==posicionCj[i] and pos2==posicionCi[i]:
            retornar=True
            break
          
        if esValido(pos1+1,pos2):
            if (a[pos1+1+(maxMapaX/2)-1][pos2+(maxMapaY/2)-1]==False):
                q.enqueue(pos1+1)
                q.enqueue(pos2)
                llegueEnX[pos1+1+(maxMapaX/2)-1][pos2+(maxMapaY/2)-1]=pos1
                llegueEnY[pos1+1+(maxMapaX/2)-1][pos2+(maxMapaY/2)-1]=pos2
        if esValido(pos1,pos2+1):
            if (a[pos1+(maxMapaX/2)-1][pos2+1+(maxMapaY/2)-1]==False):
                q.enqueue(pos1)
                q.enqueue(pos2+1)
                llegueEnX[pos1+(maxMapaX/2)-1][pos2+1+(maxMapaY/2)-1]=pos1
                llegueEnY[pos1+(maxMapaX/2)-1][pos2+1+(maxMapaY/2)-1]=pos2
        if esValido(pos1-1,pos2):
            rospy.loginfo("pos1:{}".format(pos1-1))
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

    global done
    global a
    global posicionPacmanX,posicionPacmanY,i
    if BF(posicionPacmanX,posicionPacmanY,posicionCj[i],posicionCi[i])==True:

        global punto1
        punto1=posicionCj[i]
        global punto2
        punto2=posicionCi[i]
        ruta.push(punto1)
        ruta.push(punto2)

        while punto1 is not posicionPacmanX and punto2 is not posicionPacmanY:
            global punto1,punto2,temp
            temp=punto1
            punto1= llegueEnX[punto1+(maxMapaX/2)-1][punto2+(maxMapaY/2)-1]
            punto2= llegueEnY[temp+(maxMapaX/2)-1][punto2+(maxMapaY/2)-1]
            ruta.push(punto1)
            ruta.push(punto2)

            rospy.loginfo("punto1:{},punto2:{}".format(llegueEnX[punto1+(maxMapaX/2)-1][punto2+(maxMapaY/2)-1],llegueEnY[punto1+(maxMapaX/2)-1][punto2+(maxMapaY/2)-1]))
            rospy.loginfo("puntoactualx:{},puntoactualy:{}".format(posicionPacmanX,posicionPacmanY))



    done=True


def moverPacman():
    global x,y, posicionPacmanY,posicionPacmanX, msg,ruta,posicionCi,posicionCj
    if not ruta.isEmpty():

        rospy.loginfo("hola")
        y=ruta.pop()
        x=ruta.pop()

        rX= posicionPacmanX-x
        rY= posicionPacmanY-y
        rospy.loginfo("rX: {} y rY: {}".format(rX,rY))
        rospy.loginfo("X: {} y Y: {}".format(x,y))
        if rX>0:
            msg.action=3
        elif rX<0:
            msg.action=2
        elif rY>0:
            msg.action=1
        elif rY<0:
            msg.action=0


    else:
        msg.action=-1
        pass

def cookiesCallBack(co):
	global posicionCj
	global posicionCi
	global tamanioC
	tamanioC=co.nCookies
	posicionCj=np.arange(tamanioC)
	posicionCi=np.arange(tamanioC)

	for xx in range (0,tamanioC):
		posicionCj[xx]=co.cookiesPos[xx].x
		posicionCi[xx]=co.cookiesPos[xx].y
	pass


def pacmanPosCallback(msg):
    global posicionPacmanX, posicionPacmanY,var

    posicionPacmanX = msg.pacmanPos.x
    posicionPacmanY = msg.pacmanPos.y
    var=True

#funcion que revisa si una posicion que entra como parametro es valida
def esValido(actx,acty):
    ret=True
	#recorre y compara la posicion con la posicion de todos los obstaculos del juego
    if actx>=maxXre or acty>=maxYre or actx<=minXre or acty<=minYre:
        ret=False
    else:
        for i in range (0,len(obs)):
            if actx==obs[i].x and acty==obs[i].y:
                ret=False

    return ret


def pacman_controller_py():
    rospy.init_node('pacman_controller_py', anonymous=True)
    rospy.Subscriber('pacmanCoord0', pacmanPos, pacmanPosCallback)
    pub = rospy.Publisher('pacmanActions0', actions, queue_size=10)
    rospy.Subscriber('cookiesCoord', cookiesPos,cookiesCallBack)

    try:
        mapRequestClient = rospy.ServiceProxy('pacman_world', mapService)
        global msg
        global mapa
        mapa = mapRequestClient("Grupo4")
        global maxMapaX
        global maxMapaY
        global maxXre
        global maxYre
        global minXre,h
        global minYre, ruta, q
        global i, var, done
        maxXre=mapa.maxX
        maxYre=mapa.maxY
        minXre= -1*maxXre
        minYre= -1*maxYre
        maxMapaX=(mapa.maxX)*2+1
        maxMapaY=(mapa.maxY)*2+1
        global obs
        obs = mapa.obs

        global h,tamanioC,done
        global a, llegueEnX,llegueEnY,noLlego, posicionCi, posicionCj
        a = [[False for x in range(maxMapaY)] for y in range(maxMapaX)]
        llegueEnX = [[noLlego for x in range(maxMapaY)] for y in range(maxMapaX)]
        llegueEnY= [[noLlego for x in range(maxMapaY)] for y in range(maxMapaX)]
        rate = rospy.Rate(h)  # 6hz

        while not rospy.is_shutdown():
            if (var==True  and done==False):
                definirMovimiento()
                galletaActualX=posicionCj[i]
                galletaActualY=posicionCi[i]



            if done:
                moverPacman()
                pub.publish(msg.action)
                if posicionPacmanX==galletaActualX and posicionPacmanY==galletaActualY:
                    done=False
                    rospy.loginfo("aca")
                    msg.action=-1
                    #pub.publish(msg.action)
                    a = [[False for x in range(maxMapaY)] for y in range(maxMapaX)]
                    llegueEnX = [[noLlego for x in range(maxMapaY)] for y in range(maxMapaX)]
                    llegueEnY= [[noLlego for x in range(maxMapaY)] for y in range(maxMapaX)]
                    while ruta.isEmpty==False:
                        ruta.pop()
                    while q.isEmpty==False:
                        q.dequeue
                    var=True



            #rospy.loginfo("posx:{},posy:{},Galletax:{},GalleyaY:{}".format(posicionPacmanX,posicionPacmanY,galletaActualX,galletaActualY))
            rate.sleep()

    except rospy.ServiceException as e:
        print("Error!! Make sure pacman_world node is running ")


if __name__ == '__main__':
    try:
        pacman_controller_py()
    except rospy.ROSInterruptException:
        pass
