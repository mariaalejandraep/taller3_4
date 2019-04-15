#!/usr/bin/env python
#Las librerias que se importan
import rospy
import threading
import sys
from pynput.keyboard import Key, Listener
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import roslaunch

#Es la variable donde se almacena la velocidad de los motores. 10 es el valor por defecto si no se ingresa un parametro.
vel = 10

#Es la variable donde se almacena la velocidad de cada motor que se va a publicar en el topico motorsVel.
msg = Float32MultiArray()
msg.data = [0, 0]

#Es la variable donde se almacenan las teclas presionadas. En orden corresponden a : arriba, abajo, derecha, izquierda
teclas = [0,0,0,0]

#Es la variable que indica la proporcion de la variable vel que se le asigna a cada motor.
v0 = [1,1]

#En este metodo se detecta la tecla presionada y se actualiza la variable teclas.
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

#En este metodo se detecta la tecla que se suelta y se actualiza la varibale teclas.
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

#En este metodo se inicializa el listener del hilo.
def ThreadInputs():
    with Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

#En este metodo se definen las velocidades de cada motor de acuerdo con las teclas presionadas.
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

#En este metodo se inicializa el nodo y se publica en el topico motorsVel la velocidad de cada motor.
def robot_controller():
    global msg, twistInfo
    rospy.init_node('robot_controller', anonymous=True)
    pub = rospy.Publisher('motorsVel', Float32MultiArray, queue_size=10)
    threading.Thread(target=ThreadInputs).start()
    rate = rospy.Rate(18)

    package = 'taller3_4'
    script = 'graficador3a.py'
    node = roslaunch.core.Node(package,script)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    process = launch.launch(node)

    while not rospy.is_shutdown():
        definirMovimiento()
        pub.publish(msg)
        rate.sleep()

#Este metodo llama al metodo que inicializa el nodo y ademas verifica si se ingreso un parametro. Si se ingresa, lo asigna como la velocidad.
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
