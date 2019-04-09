#!/usr/bin/env python
import rospy
import threading
import sys
import time
import os
from pynput.keyboard import Key, Listener
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


vel = 10
msg = Float32MultiArray()
msg.data = [vel, vel]
twistInfo = Twist()
fig = None
axs = None
xCord = []
yCord = []
terminate = False


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
    pub = rospy.Publisher('motorsVel', Float32MultiArray, queue_size=10)
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
