#!/usr/bin/env python
import rospy
import threading
import sys
from pynput.keyboard import Key, Listener
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

#Las que se importan para este
import random
import math
import numpy as np
import roslaunch

#Por ahora en 0
vel = 2

msg = Float32MultiArray()
msg.data = [0, 0]
twistInfo = Twist()
fig = None
axs = None
xCord = []
yCord = []
terminate = False

numActualizado = 0
obsActualizado = []
obs = []
num = 0
iteraciones = 40
xActual = 0
yActual = 0
pActual = 0
mp = 0
bp = 0
umbralInlier = 0.00001
umbralPuntos = 4
minimoInliers = 0
maximoIteraciones = 10*iteraciones
rectas = Float32MultiArray()
datosQuitar = []
numQuitar = 0
obsQuitar = []
xmin = 20
xmax = -20
xInlier = list()
yInlier = list()
pInlier = list()
dInlier = list()
tempX = list()
tempY = list()
tempP = list()
tempD = list()
umbralX = 0.1
umbralY = 0.4
dX = 0
dY = 0
teclas = [0,0,0,0]
v0 = [1,1]
h = 15

def on_press(key):
    global msg, vel, teclas, v0

    if key == Key.up:
        teclas[0] = 1
    if key == Key.down:
        vel = -abs(vel)
        teclas[1] = 1
    if key == Key.right:
        teclas[2] = 1
    if key == Key.left:
        teclas[3] = 1


def on_release(key):
    global msg, vel, teclas, v0
    if key == Key.up:
        teclas[0] = 0
    if key == Key.down:
        teclas[1] = 0
    if key == Key.right:
        teclas[2] = 0
    if key == Key.left:
        teclas[3] = 0

def definirMovimiento():
    global vel, v0, teclas, msg

    if teclas[2] == 1:
        v0 = [0.8,0.2]
    elif teclas[3] == 0:
        v0 = [1,1]

    if teclas[3] == 1:
        v0 = [0.2,0.8]
    elif teclas[2] == 0:
        v0 = [1,1]

    if teclas[0] == 1:
        vel = abs(vel)
    elif teclas[1] == 0:
        v0 = [0,0]

    if teclas[1] == 1:
        vel = -abs(vel)
    elif teclas[0] == 0:
        v0 = [0,0]

    if teclas[0] == 1 and teclas[1] == 1:
        v0 = [0,0]


    msg.data = [v0[0]*vel, v0[1]*vel]

def ThreadInputs():
    with Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()


def robot_controller():
    global msg, twistInfo, done, pubRectas, pub, v0, h, rectas, msg, thread
    rospy.init_node('robot_controller', anonymous=True)
    rospy.Subscriber('pioneerPosition', Twist, setPositionCallback)
    rospy.Subscriber('scanner', Float32MultiArray, actualizarObstaculos)
    pub = rospy.Publisher('motorsVel', Float32MultiArray, queue_size=10)
    pubRectas = rospy.Publisher('rectas', Float32MultiArray, queue_size=10)
    thread = threading.Thread(target=ThreadInputs)
    thread.start()
    rate = rospy.Rate(h)

    package = 'taller3_4'
    script = 'graficador3c.py'
    node = roslaunch.core.Node(package,script)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    process = launch.launch(node)

    while not rospy.is_shutdown():
        definirMovimiento()
        calcularLineas()
        pubRectas.publish(rectas)
        pub.publish(msg)
        rate.sleep()

    #thread.stop_event.set()

def actualizarObstaculos(puntos):
    global numActualizado, obsActualizado

    numActualizado = puntos.layout.data_offset
    obsActualizado = list(puntos.data)

def setPositionCallback(pos):
    global twistInfo, xActual, yActual, pActual
    twistInfo = pos
    xActual = pos.linear.x
    yActual = pos.linear.y
    pActual = pos.angular.z

def calcularLineas():
    global num, numActualizado, obs, obsActualizado, xActual, yActual, mp, bp, iteraciones, umbralInlier, rectas, datosQuitar, numQuitar, obsQuitar, minimoInliers,pubRectas
    global xmin, xmax, xInlier, yInlier, pInlier, dInlier, tempX, tempY, tempP, tempD, maximoIteraciones

    obs[:] = obsActualizado[:]
    num = numActualizado

    minimoInliers = num/10
    m = 0

    rectas.data[:] = []

    while num > umbralPuntos*2 and m < maximoIteraciones:

        minimoInliers = num/10

        if minimoInliers > 10:
            minimoInliers = 10

        xInlier = list()
        yInlier = list()
        pInlier = list()
        dInlier = list()

        i = 0

        while i < iteraciones :

            xmin = 20
            xmax = -20

            tempX = list()
            tempY = list()
            tempP = list()
            tempD = list()

            maximoInlier = 0
            cantidadInliers = 0

            punto1 = random.randint(0, num/2-1)*2
            punto2 = random.randint(0, num/2-1)*2

            while punto2 == punto1:
                punto2 = random.randint(0, num/2-1)*2

            p1 = obs[punto1]
            d1 = obs[punto1+1]

            p2 = obs[punto2]
            d2 = obs[punto2+1]

            x1 = xActual + d1*math.cos(p1+pActual)
            y1 = yActual + d1*math.sin(p1+pActual)

            x2 = xActual + d2*math.cos(p2+pActual)
            y2 = yActual + d2*math.sin(p2+pActual)

            m1 = (y1-y2)/(x1-x2)
            b1 = y1-m1*x1

            j = 0

            while j < num:

                pj = obs[j]
                dj = obs[j+1]

                xj = xActual + dj*math.cos(pj+pActual)
                yj = yActual + dj*math.sin(pj+pActual)

                xe = (m1*yj-m1*b1+xj)/(1+m1**2)
                ye = m1*xe+b1

                error = (ye-yj)**2+(xe-xj)**2

                if(error < umbralInlier):

                    tempX.append(xj)
                    tempY.append(yj)
                    tempP.append(pj)
                    tempD.append(dj)

                    cantidadInliers = cantidadInliers+1

                j = j+2

            #rospy.loginfo("Inliers:{} y num:{} y minimo:{}".format(cantidadInliers,num,minimoInliers))


            if (cantidadInliers > maximoInlier and cantidadInliers > minimoInliers):

                xInlier[:] = tempX[:]
                yInlier[:] = tempY[:]
                pInlier[:] = tempP[:]
                dInlier[:] = tempD[:]

            i = i + 1

        if not (xInlier is None  or xInlier == []):

            if(len(xInlier)==0):
                rospy.loginfo("pasa")

            verificar()

            n = len(xInlier)

            if(n > minimoInliers):

                mp = (n*(sum(np.multiply(xInlier,yInlier)))-(sum(xInlier)*sum(yInlier)))/(n*sum(np.multiply(xInlier,xInlier))-(sum(xInlier))**2)
                bp = (1.0/n)*(sum(yInlier)-mp*sum(xInlier))

                k = 0

                while k<len(pInlier):
                    obs.remove(pInlier[k])
                    obs.remove(dInlier[k])
                    k = k+1

                num = len(obs)

                rectas.data.append(mp)
                rectas.data.append(bp)
                rectas.data.append(xmin)
                rectas.data.append(xmax)

            m = m+1

def verificar():
    global xInlier, yInlier, pInlier, dInlier, umbralX, umbralY, xmin, xmax, dX, dY

    dX = 0
    dY = 0
    l = 0

    n = len(xInlier)

    x1 = xInlier[l]
    x2 = xInlier[l+1]

    y1 = yInlier[l]
    y2 = yInlier[l+1]

    dX = abs(x1-x2)
    dY = abs(y1-y2)

    if(dX > umbralX):
        xInlier = xInlier[1:n]
        yInlier = yInlier[1:n]
        pInlier = pInlier[1:n]
        dInlier = dInlier[1:n]

    if(dY > umbralY):
        xInlier = xInlier[1:n]
        yInlier = yInlier[1:n]
        pInlier = pInlier[1:n]
        dInlier = dInlier[1:n]

    l = l+1

    while l < len(xInlier)-1:
        x1 = xInlier[l]
        x2 = xInlier[l+1]

        y1 = yInlier[l]
        y2 = yInlier[l+1]

        dX = abs(x1-x2)
        dY = abs(y1-y2)

        if(dX > umbralX):
            xInlier = xInlier[0:l]
            yInlier = yInlier[0:l]
            pInlier = pInlier[0:l]
            dInlier = dInlier[0:l]
            break

        if(dY > umbralY):
            xInlier = xInlier[0:l]
            yInlier = yInlier[0:l]
            pInlier = pInlier[0:l]
            dInlier = dInlier[0:l]
            break

        l = l+1

    if(len(xInlier) > 0):
        xmin = min(xInlier)
        xmax = max(xInlier)

if __name__ == '__main__':
    try:
        if len(sys.argv) > 1:
            try:
                vel = float(sys.argv[1])
            except ValueError:
                pass

        robot_controller()
    except rospy.ROSInterruptException:
        pass
