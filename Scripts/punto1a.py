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
msg.action = 0


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

def BF (posactx,posacty,posGoalx,posGoalY):
    global q
    global ruta
    global pos1
    global pos2
    q.enqueue(posactx)
    q.enqueue(posacty)
    while q.isEmpty()==False:
        pos1=q.dequeue()
        pos2=q.dequeue()
        if posGoalx==pos1 and pos2==posGoalY:
            ruta.push(pos1)
            ruta.push(pos2)
        if esValido(pos1+1,pos2):
            q.enqueue(pos1+1)
            q.enqueue(pos2)
        if esValido(pos1,pos2+1):
            q.enqueue(pos1)
            q.enqueue(pos2+1)
        if esValido(pos1-1,pos2):
            q.enqueue(pos1-1)
            q.enqueue(pos2)
        if esValido(pos1,pos2-1):
            q.enqueue(pos1)
            q.enqueue(pos2-1)

    return true




def definirMovimiento():
    rospy.loginfo("entro")
    if BF(posicionPacmanX,posicionPacmanY,posicionCj,posicionCi)==true:
        rospy.loginfo("entro2")
        while ruta.isEmpty()==False:
            rospy.loginfo("X : {}  Y : {}".format(ruta.pop(), ruta.pop()))
    done=True
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
    global posicionPacmanX
    global posicionPacmanY



    posicionPacmanX = msg.pacmanPos.x
    posicionPacmanY = msg.pacmanPos.y
    var=True

#funcion que revisa si una posicion que entra como parametro es valida
def esValido(actx,acty):
	ret=True
	#recorre y compara la posicion con la posicon de todos los obstaculos de juego
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

        global mapa
        mapa = mapRequestClient("Grupo4")

        global obs
        obs = mapa.obs

        global h

        rate = rospy.Rate(h)  # 6hz

        while not rospy.is_shutdown():
            if (var==True  and done==False):
                definirMovimiento()
            pub.publish(msg.action)
            rate.sleep()

    except rospy.ServiceException as e:
        print("Error!! Make sure pacman_world node is running ")


if __name__ == '__main__':
    try:
        pacman_controller_py()
    except rospy.ROSInterruptException:
        pass
