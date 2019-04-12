#!/usr/bin/env python
import rospy
import threading
import sys
import time
import os
from pynput.keyboard import Key, Listener
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

#Las que se importan para este
import random
import math
import numpy as np

#Por ahora en 0
vel = 2

msg = Float32MultiArray()
msg.data = [vel, vel]
twistInfo = Twist()
fig = None
axs = None
xCord = []
yCord = []
terminate = False

obs = []
num = 0
iteraciones = 5
xActual = 0
yActual = 0
pActual = 0
mp = 0
bp = 0
umbralInlier = 0.00001
umbralPuntos = 10
minimoInliers = 0
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
umbralX = 0.1
umbralY = 0.4
dX = 0
dY = 0

def on_press(key):
    global msg, vel
    if key == Key.right:
        msg.data = [0.8 * vel, 0.2 * vel]
    elif key == Key.left:
        msg.data = [0.2 * vel, 0.8 * vel]
    elif key == Key.down:
        msg.data = [-vel, -vel]


def on_release(key):
    global msg, vel
    if key == Key.down or key == Key.right or key == Key.left:
        msg.data = [vel, vel]


def ThreadInputs():
    with Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()


def robot_controller():
    global msg, twistInfo, done, pubRectas, pub
    rospy.init_node('robot_controller', anonymous=True)
    rospy.Subscriber('pioneerPosition', Twist, setPositionCallback)
    rospy.Subscriber('scanner', Float32MultiArray, actualizarObstaculos)
    pub = rospy.Publisher('motorsVel', Float32MultiArray, queue_size=10)
    pubRectas = rospy.Publisher('rectas', Float32MultiArray, queue_size=10)
    pubPosicion = rospy.Publisher('topico_Posicion', Twist, queue_size=10)
    threading.Thread(target=ThreadInputs).start()
    rate = rospy.Rate(10)
    #contador = 0

    while not rospy.is_shutdown():
        #calcularLineas()
        #pub.publish(msg)
        #pubRectas.publish(rectas)
        #rospy.loginfo("len:{}".format(len(rectas.data)/2))
        #contador = contador + 1
        #if contador == 3:
        pubPosicion.publish(twistInfo)
        #    contador = 0
        rate.sleep()

def actualizarObstaculos(puntos):
    global num, obs

    num = puntos.layout.data_offset
    obs = list(puntos.data)
    calcularLineas()

def setPositionCallback(pos):
    global twistInfo, xActual, yActual, pActual
    twistInfo = pos
    xActual = pos.linear.x
    yActual = pos.linear.y
    pActual = pos.angular.z

def calcularLineas():
    global num, obs, xActual, yActual, mp, bp, iteraciones, umbralInlier, rectas, datosQuitar, numQuitar, obsQuitar, minimoInliers,pubRectas
    global xmin, xmax, xInlier, yInlier, pInlier, dInlier, tempX, tempY, tempP

    minimoInliers = num/10

    rectas.data[:] = []

    #datosQuitar = obs

    #rospy.loginfo("numObs comienzo:{}".format(num))

    while num > umbralPuntos:

        rospy.loginfo("num:{}, lenObs:{} y umbral:{}".format(num,len(obs),umbralPuntos))

        minimoInliers = num/10
        #obsQuitar[:] = obs[:]
        #obsNumQuitar = num
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

            #datosQuitar[:] = obs[:]
            #numQuitar = num

            maximoInlier = 0
            cantidadInliers = 0

            #rospy.loginfo("lenObs:{}, num:{} y numQuitar:{}".format(len(obs),num,numQuitar))
            punto1 = random.randint(0, num/2-1)*2
            punto2 = random.randint(0, num/2-1)*2

            while punto2 == punto1:
                punto2 = random.randint(0, num/2-1)*2

            p1 = obs[punto1]
            d1 = obs[punto1+1]

            p2 = obs[punto2]
            #rospy.loginfo("punto2:{}".format(punto2))
            d2 = obs[punto2+1]

            x1 = xActual + d1*math.cos(p1+pActual)
            y1 = yActual + d1*math.sin(p1+pActual)

            x2 = xActual + d2*math.cos(p2+pActual)
            y2 = yActual + d2*math.sin(p2+pActual)

            #rospy.loginfo("x1:{} y x2:{}; y1:{} y y2:{}".format(x1,x2,y1,y2))

            m1 = (y1-y2)/(x1-x2)
            b1 = y1-m1*x1

            j = 0

            while j < num:

                pj = obs[j]
                dj = obs[j+1]

                #rospy.loginfo("j:{} y lenObs:{} y num:{} y cantidad:{}".format(j,len(obs),num,cantidadInliers))

                xj = xActual + dj*math.cos(pj+pActual)
                yj = yActual + dj*math.sin(pj+pActual)

                xe = (m1*yj-m1*b1+xj)/(1+m1**2)
                ye = m1*xe+b1

                error = (ye-yj)**2+(xe-xj)**2

                #rospy.loginfo("Error:{}".format(error))

                if(error < umbralInlier):
                    #datosQuitar[j] = 1
                    #datosQuitar[j+1] = 1

                    #datosQuitar[j:j+1] = []

                    tempX.append(xj)
                    tempY.append(yj)
                    tempP.append(pj)
                    tempD.append(dj)
                    #rospy.loginfo(temp)

                    #if (xmin > xj):
                    #    xmin = xj
                    #if (xmax < xj):
                    #    xmax = xj

                    #datosQuitar.remove(pj)
                    #datosQuitar.remove(dj)
                    #rospy.loginfo("lenObs:{} y lenQuitar:{}".format(len(obs), len(datosQuitar)))
                    #datosQuitar.pop(j)

                    #numQuitar = numQuitar-2
                    cantidadInliers = cantidadInliers+1

                j = j+2

            rospy.loginfo("Inliers:{} y num:{} y minimo:{}".format(cantidadInliers,num,minimoInliers))


            if (cantidadInliers > maximoInlier and cantidadInliers > minimoInliers):
                #mp = m1
                #bp = b1

                #rospy.loginfo("Entre!")

                xInlier[:] = tempX[:]
                yInlier[:] = tempY[:]
                pInlier[:] = tempP[:]
                dInlier[:] = tempD[:]

                #rospy.loginfo(xInlier)

                #obsQuitar[:] = datosQuitar[:]
                #obsNumQuitar = numQuitar
                #rospy.loginfo(cantidadInliers)
                #num = numQuitar

                #rospy.loginfo("num:{} y len:{}".format(num,len(obs)))
                #rospy.loginfo("mp:{} y bp:{}".format(mp,bp))

            i = i + 1

            #rospy.loginfo("punto1:{}, punto2:{} y num:{}".format(punto1,punto2,num))


        #rospy.loginfo(xInlier)

        if not (xInlier is None  or xInlier == []):
            #rospy.loginfo("Entrreeeeee")

            if(len(xInlier)==0):
                rospy.loginfo("pasa")

            #rospy.loginfo("lenX:{}, min:{} y num:{}".format(len(xInlier),minimoInliers,num))
            verificar()
            #rospy.loginfo("Sali")

            #obsNumQuitar = len(xInlier)
            n = len(xInlier)

            rospy.loginfo("n:{}".format(n))

            if(n > minimoInliers):
                #mp = m1
                #bp = b1

                mp = (n*(sum(np.multiply(xInlier,yInlier)))-(sum(xInlier)*sum(yInlier)))/(n*sum(np.multiply(xInlier,xInlier))-(sum(xInlier))**2)
                bp = (1.0/n)*(sum(yInlier)-mp*sum(xInlier))

                #mp = np.multiply(xInlier,yInlier)

                #rospy.loginfo("m:{} y b:{}".format(mp,bp))

                # rospy.loginfo("Empiezo")
                # rospy.loginfo(len(mp))
                # rospy.loginfo(len(xInlier))
                # rospy.loginfo(len(yInlier))

                k = 0

                #rospy.loginfo("len antes:{} y lenP:{}".format(len(obs),len(pInlier)))
                #rospy.loginfo("P:{} y D:{}".format(len(pInlier),len(dInlier)))
                while k<len(pInlier):
                    obs.remove(pInlier[k])
                    obs.remove(dInlier[k])
                    k = k+1

                num = len(obs)
                #rospy.loginfo("num:{}".format(num))

                #rospy.loginfo(m1)

                rectas.data.append(mp)
                rectas.data.append(bp)
                rectas.data.append(xmin)
                rectas.data.append(xmax)

        #rospy.loginfo("num:{} y len:{}".format(num,len(rectas.data)))
        #rospy.loginfo("num:{} y lenObs:{}".format(num,len(obs)))
        #obs[:] = obsQuitar[:]

        #num = num-obsNumQuitar

    rospy.loginfo("Publicando")
    pubRectas.publish(rectas)
    pub.publish(msg)
    #rospy.loginfo(len(rectas.data)/2)
    #rospy.loginfo(rectas)
    #rospy.loginfo("xmin:{} y xmax:{}".format(xmin,xmax))

def verificar():
    global xInlier, yInlier, pInlier, dInlier, umbralX, umbralY, xmin, xmax, dX, dY

    #rospy.loginfo(xInlier)

    #xInlier = xInlier.sort()

    #rospy.loginfo(xInlier)

    dX = 0
    dY = 0
    l = 0

    #rospy.loginfo("A")
    #rospy.loginfo(len(xInlier))

    while l < len(xInlier)-1:
        x1 = xInlier[l]
        x2 = xInlier[l+1]

        y1 = yInlier[l]
        y2 = yInlier[l+1]

        dX = abs(x1-x2)
        dY = abs(y1-y2)

        #rospy.loginfo("x1:{}, x2:{}, dX:{} y l:{}".format(x1,x2,dX,l))
        #rospy.loginfo("y1:{}, y2:{} y dY:{}".format(y1,y2,dY))
        rospy.loginfo("dX:{} y dY:{}".format(dX,dY))

        if(dX > umbralX):
            xInlier = xInlier[0:l]
            yInlier = yInlier[0:l]
            pInlier = pInlier[0:l]
            dInlier = dInlier[0:l]
            #rospy.loginfo("DIFXENTRE. lenX:{} y i:{}".format(len(xInlier),i))
            break

        if(dY > umbralY):
            xInlier = xInlier[0:l]
            yInlier = yInlier[0:l]
            pInlier = pInlier[0:l]
            dInlier = dInlier[0:l]
            #rospy.loginfo("DIFXENTRE. lenX:{} y i:{}".format(len(xInlier),i))
            break

        l = l+1

    #rospy.loginfo("Aqui")

    if(len(xInlier) > 0):
        #rospy.loginfo("len:{}, i:{}".format(len(xInlier),i))
        xmin = min(xInlier)
        xmax = max(xInlier)

if __name__ == '__main__':
    try:
        if len(sys.argv) > 1:
            try:
                vel = float(sys.argv[1])
                msg.data = [vel, vel]
            except ValueError:
                pass

        robot_controller()
    except rospy.ROSInterruptException:
        pass
