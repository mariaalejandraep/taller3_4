#!/usr/bin/env python
import rospy
import threading
from pynput.keyboard import Key, Listener
from pacman.msg import pacmanPos
from pacman.srv import mapService
from pacman.msg import actions

msg0 = actions()
msg1 = actions()
msgAnterior = actions()

msg0.action = 0
msg1.action = -1
msgAnterior.action = -1

noMovio = None
rescatar = False

obstaculos = []
anterior=-1
h=1/0.15

mapa = None

posicionPacman0X = 0
posicionPacman0Y = 0
posicionPacman1X = 0
posicionPacman1Y = 0


def esValido(actx,acty):
	ret=True
	for i in range (0,len(obstaculos)):

		if actx==obstaculos[i].x and acty==obstaculos[i].y:
			ret=False
	return ret

def on_press(key):
    global anterior

    if key == Key.up:

        if esValido(posicionPacman1X,posicionPacman1Y+1):
            msg1.action=0
        else:

            anterior=0
    if key == Key.down:
		if esValido(posicionPacman1X,posicionPacman1Y-1):
			msg1.action=1
		else:
			anterior=1
    if key == Key.right:
        if esValido(posicionPacman1X+1,posicionPacman1Y):

            msg1.action=2
        else:
			anterior=2
    if key == Key.left:
        if esValido(posicionPacman1X-1,posicionPacman1Y):
			msg1.action=3
        else:
            anterior=3

def intentarAnterior():
	global anterior
	if anterior==0 and esValido(posicionPacman1X,posicionPacman1X+1):
		msg1.action=0
		anterior=-1
	if anterior==1 and esValido(posicionPacman1X,posicionPacman1Y-1):
		msg1.action=1
		anterior=-1
	if anterior==2 and esValido(posicionPacman1X+1,posicionPacman1Y):
		msg1.action=2
		anterior=-1
	if anterior==3 and esValido(posicionPacman1X-1,posicionPacman1Y):
		msg1.action=3
		anterior=-1

def on_release(key):
	pass

def ThreadInputs(): #Listener de Pynput en otro thread
	with Listener(on_press=on_press, on_release=on_release) as listener:
		listener.join()

def empezarRescate(msg):
    global posicionPacman0X
    global posicionPacman0Y
    global posicionPacman1X
    global posicionPacman1Y
    rospy.loginfo("posx0:{}".format(posicionPacman0X))
    global rescatar
    if abs(posicionPacman0X - posicionPacman1X)==0 and abs(posicionPacman0Y-posicionPacman1Y) == 0 and not rescatar:
        global msgAnterior
        msgAnterior.action = -1
        rescatar = True

def definirMovimiento():
    global noMovio
    if noMovio:
        noMovio = False
        if msg0.action == 0:
            if not chequearPosicionDerecha():
                msg0.action = 2
            elif not chequearPosicionIzquierda():
                msg0.action = 3
            elif not chequearPosicionAbajo():
                msg0.action = 1
        elif msg0.action == 2:
            if not chequearPosicionAbajo():
                msg0.action = 1
            elif not chequearPosicionArriba():
                msg0.action = 0
            elif not chequearPosicionIzquierda():
                msg0.action = 3
        elif msg0.action == 1:
            if not chequearPosicionIzquierda():
                msg0.action = 3
            elif not chequearPosicionDerecha():
                msg0.action = 2
            elif not chequearPosicionArriba():
                msg0.action = 0
        elif msg0.action == 3:
            if not chequearPosicionArriba():
                msg0.action = 0
            elif not chequearPosicionAbajo():
                msg0.action = 1
            elif not chequearPosicionDerecha():
                msg0.action = 2
    elif not noMovio == None:
        if msg0.action == 0:
            if not chequearPosicionDerecha():
                msg0.action = 2
        elif msg0.action == 1:
            if not chequearPosicionIzquierda():
                msg0.action = 3
        elif msg0.action == 2:
            if not chequearPosicionAbajo():
                msg0.action = 1
        elif msg0.action == 3:
            if not chequearPosicionArriba():
                msg0.action = 0

def pacman0PosCallback(msg):
    global posicionPacman0X
    global posicionPacman0Y

    if posicionPacman0Y == msg.pacmanPos.y and posicionPacman0X == msg.pacmanPos.x:
        global noMovio
        noMovio = True

    posicionPacman0X = msg.pacmanPos.x
    posicionPacman0Y = msg.pacmanPos.y

def pacman1PosCallback(msg):
    global posicionPacman1X
    global posicionPacman1Y

    posicionPacman1X = msg.pacmanPos.x
    posicionPacman1Y = msg.pacmanPos.y
    empezarRescate(msg)

def chequearPosicionArriba():
    i = 0
    while i < mapa.nObs:
        obstaculoChequeo = obstaculos[i]
        if obstaculoChequeo.x == posicionPacman0X:
            if obstaculoChequeo.y - 1 == posicionPacman0Y:
                return True
        i=i+1

def chequearPosicionAbajo():
    i = 0
    while i < mapa.nObs:
        obstaculoChequeo = obstaculos[i]
        if obstaculoChequeo.x == posicionPacman0X:
            if obstaculoChequeo.y + 1 == posicionPacman0Y:
                return True
        i=i+1

def chequearPosicionDerecha():
    i = 0
    while i < mapa.nObs:
        obstaculoChequeo = obstaculos[i]
        if obstaculoChequeo.y == posicionPacman0Y:
            if obstaculoChequeo.x - 1 == posicionPacman0X:
                return True
        i=i+1

def chequearPosicionIzquierda():
    i = 0
    while i < mapa.nObs:
        obstaculoChequeo = obstaculos[i]
        if obstaculoChequeo.y == posicionPacman0Y:
            if obstaculoChequeo.x + 1 == posicionPacman0X:
                return True
        i=i+1

def pacman_controller_py():
    rospy.init_node('pacman_controller_py', anonymous=True)
    rospy.Subscriber('pacmanCoord0', pacmanPos, pacman0PosCallback)
    rospy.Subscriber('pacmanCoord1',pacmanPos, pacman1PosCallback)
    pub0 = rospy.Publisher('pacmanActions0', actions, queue_size=10)
    pub1 = rospy.Publisher('pacmanActions1', actions, queue_size=10)
    threading.Thread(target=ThreadInputs).start()

    try:
        mapRequestClient = rospy.ServiceProxy('pacman_world', mapService)

        global mapa
        mapa = mapRequestClient("Grupo 4")

        global obstaculos
        obstaculos = mapa.obs

        global h

        rate = rospy.Rate(h)  # (1/0.15)Hz (Aproximadamente 6.666Hz)

        while not rospy.is_shutdown():
            intentarAnterior()
            global rescatar
            global msgAnterior
            global msg1
            pub1.publish(msg1.action)
            definirMovimiento()

            if not rescatar:
                pub0.publish(msg0.action)
            else:
                pub0.publish(msgAnterior.action)


            rospy.loginfo("msg:{}".format(msg1.action))

            msgAnterior.action=msg1.action

            rate.sleep()

    except rospy.ServiceException as e:
        print("Error!! Make sure pacman_world node is running ")


if __name__ == '__main__':
    try:
        pacman_controller_py()
    except rospy.ROSInterruptException:
        pass
