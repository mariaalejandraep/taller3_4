#!/usr/bin/env python
import rospy
import threading
import sys
import time
import os
from pynput.keyboard import Key, Listener
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

#las que se importan para este
import roslaunch

vel = 10
msg = Float32MultiArray()
msg.data = [0, 0]
twistInfo = Twist()
fig = None
axs = None
xCord = []
yCord = []
terminate = False


teclas = [0,0,0,0]
v0 = [1,1]

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

def ThreadInputs():
    with Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

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

def robot_controller():
    global msg, twistInfo
    rospy.init_node('robot_controller', anonymous=True)
    rospy.Subscriber('pioneerPosition', Twist, setPositionCallback)
    pub = rospy.Publisher('motorsVel', Float32MultiArray, queue_size=10)
    pubPosicion = rospy.Publisher('topico_Posicion', Twist, queue_size=10)
    threading.Thread(target=ThreadInputs).start()
    rate = rospy.Rate(10)
    contador = 0

    package = 'taller3_4'
    script = 'graficador3a.py'
    node = roslaunch.core.Node(package,script)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    process = launch.launch(node)

    while not rospy.is_shutdown():
        definirMovimiento()
        pub.publish(msg)
        contador = contador + 1
        if contador == 3:
            pubPosicion.publish(twistInfo)
            contador = 0
        rate.sleep()


def setPositionCallback(pos):
    global twistInfo
    twistInfo = pos

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
