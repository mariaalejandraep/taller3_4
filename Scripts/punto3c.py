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

#vel en 0 por ahora
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
mp = 0
bp = 0

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
    global msg, twistInfo
    rospy.init_node('robot_controller', anonymous=True)
    rospy.Subscriber('pioneerPosition', Twist, setPositionCallback)
    rospy.Subscriber('scanner', Float32MultiArray, actualizarObstaculos)
    pub = rospy.Publisher('motorsVel', Float32MultiArray, queue_size=10)
    #pubRectas = rospy.Publisher('rectas', )
    pubPosicion = rospy.Publisher('topico_Posicion', Twist, queue_size=10)
    threading.Thread(target=ThreadInputs).start()
    rate = rospy.Rate(10)
    contador = 0

    while not rospy.is_shutdown():
        pub.publish(msg)
        contador = contador + 1
        if contador == 3:
            pubPosicion.publish(twistInfo)
            contador = 0
        rate.sleep()

def actualizarObstaculos(puntos):
    global num, obs

    num = puntos.layout.data_offset
    obs = puntos.data

    calcularLineas()

def setPositionCallback(pos):
    global twistInfo, xActual, yActual
    twistInfo = pos
    xActual = pos.linear.x
    yActual = pos.linear.y

def calcularLineas():
    global num, obs, xActual, yActual, mp, bp, iteraciones


    i = 0
    errorGuardada = 9999

    mp = 0
    bp = 0

    while i < iteraciones:

        punto1 = random.randint(0, num/2-1)*2
        punto2 = random.randint(0, num/2-1)*2

        while punto2 == punto1:
            punto2 = random.randint(0, num/2-1)*2

        p1 = obs[punto1]
        d1 = obs[punto1+1]

        p2 = obs[punto2]
        d2 = obs[punto2+1]

        x1 = xActual + d1*math.cos(p1)
        y1 = yActual + d1*math.sin(p1)

        x2 = xActual + d2*math.cos(p2)
        y2 = yActual + d2*math.sin(p2)

        #rospy.loginfo("x1:{} y x2:{}; y1:{} y y2:{}".format(x1,x2,y1,y2))

        m1 = (y1-y2)/(x1-x2)
        b1 = y1-m1*x1

        j = 0
        error = 0

        while j < num:

            xj = obs[j]
            yj = obs[j+1]

            xe = (m1*yj-m1*b1+xj)/(1+m1**2)
            ye = m1*xe+b1

            error = error + (ye+yj)**2+(xe+xj)**2

            j = j+2

        if (error < errorGuardada):
            mp = m1
            bp = b1
            errorGuardada = error
            #rospy.loginfo("mp:{} y bp:{}".format(mp,bp))

        #rospy.loginfo("punto1:{}, punto2:{} y num:{}".format(punto1,punto2,num))


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
