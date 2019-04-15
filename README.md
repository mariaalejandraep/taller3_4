# taller3_4
# en caso de encontrar algun problema con el archivo zip el paquete se puede clonar de github con la siguiente direccion: https://github.com/mariaalejandraep/taller3_4.git

Punto 1:
A) comando de ejecución:
Primero se debe iniciar el mapa mediumCorners con el siguiente comando:
rosrun pacman pacman_world --c mediumCorners
despues se debe iniciar el nodo con el siguiente comando:
rosrun taller3_4 punto1a.py

dependencias:
	import rospy
	import numpy as np
	import time

C) comando de ejecución
Primero se debe iniciar el mapacon el siguiente comando:
rosrun pacman pacman_world --c [nombre del mapa]
ej: rosrun pacman pacman_world --c bigCorners
despues se debe iniciar el nodo con el siguiente comando:
rosrun taller3_4 punto1c.py
dependencias:
	import rospy
	import numpy as np
Punto 2:
Primero se debe instalar la herramienta networkx con el siguiente comando:
pip install --user networkx
o siguiendo las instrucciones en esta pagina:
https://networkx.github.io/documentation/stable/install.html
Cabe que vrep debe estar corriendo al igual que roscore. Se debe escoger como escena scene_punto2_1piso.ttt, puede ejecutar el código o la simulación primero.
A)
Comando de ejecución:
 	rosrun taller3_4 punto2a.py
Dependencias:
	import rospy 
	from geometry_msgs.msg import Twist

B) 
Comando de ejecución:
 	rosrun taller3_4 punto2b.py
Dependencias:
	import rospy, time, math
	from geometry_msgs.msg import Twist
	import matplotlib.pyplot as plt
	import networkx as nx

C) 
Comando de ejecución:
 	rosrun taller3_4 punto2c.py [coordenadax] [coordenaday] [theta]
	ejm: rosrun taller3_4 punto2c.py 10 10 0
	ejpm2: rosrun taller3_4 punto2c.py
Dependencias:
	Por punto:
	import rospy, math, roslaunch, time, sys
	from std_msgs.msg import Float32MultiArray
	from geometry_msgs.msg import Twist
	import matplotlib.pyplot as plt
	import networkx as nx
	Por graficador:
	import matplotlib.animation as animation
	from geometry_msgs.msg import Twist

E) 
Cuando inicialice el nodo tal vez se demore un tiempo mientras determina la ruta a la cual quiere llegar, esto es aleatorio, por favor esperar.
Cuando se muestre una gráfica indicando nodos creados, nodos por los cuales no puede pasar y el camino señalado en verde se debe cerrar el mapa para iniciar el movimiento del robot.
Comando de ejecución:
 	rosrun taller3_4 punto2e.py [coordenadax] [coordenaday] [theta]
	ejm: rosrun taller3_4 punto2e.py 10 10 0
	ejpm2: rosrun taller3_4 punto2e.py
Dependencias:
	Por punto:
	import rospy, math, sys, roslaunch, time, random
	from std_msgs.msg import Float32MultiArray
	from geometry_msgs.msg import Twist
	import matplotlib.pyplot as plt
	import networkx as nx
	import matplotlib.pyplot as plt
	Por graficador:
	import matplotlib.animation as animation
	from geometry_msgs.msg import Twist
	Recordar que para que empiece a avanzar el robot se debe cerrar la primera gráfica que aparece

Punto 3:
Primero se debe ejecutar el comando roscore en la linea de comandos y además abrir el simulador vrep. Despue se debe abrir la escena scene_punto3 que se puede encontrar dentro del archivo resources. Se puede iniciar la simulacion primero y despues ejecutar el script o ejecutar el script y despues la simulacion. El envio de la velocidad del robot se puede realizar por parametro, sin embargo no es necesario. Si no se envia una velocidad por parametro se utiliza el valor por defecto del codigo. Para el iteral a esta velocidad es de 10, y para el c es de 4.

Se deben ejecutar los siguientes comandos en una terminal para instalar las librerias que se utilizan en los codigos y asi lograr correr los scripts desarrollados.

Instalacion de pynput: Esta es una libreria para detectar las teclas presionadas.
	Comando: 
	pip install pynput

Instalacion de numpy: Esta es una libreria utilizada para realizar operaciones sobre vectores.
	Comandos:
	sudo apt update
	sudo apt install python-numpy

	Para Python 3.x
	sudo apt update
	sudo apt install python3-numpy

La version de Python se puede verificar con el comando python --version.

A)
Comando de ejecucion:
	rosrun taller3_4 punto3a.py [la velocidad del robot]
	ejm: rosrun taller3_4 punto3a.py 4
	ejm2: rosrun taller3_4 punto3a.py
Dependencias:
	Nodo principal:
	import rospy, threading, sys, roslaunch
	from pynput.keyboard import Key, Listener
	from geometry_msgs.msg import Twist

	Graficador:
	import rospy, math
	import matplotlib.pyplot as plt
	import matplotlib.animation as animation
	from std_msgs.msg import Twist

C)
Comando de ejecucion:
	rosrun taller3_4 punto3c.py [la velocidad del robot]
	ejm: rosrun taller3_4 punto3c.py 2
	ejm2: rosrun taller3_4 punto3c.py
Dependencias:
	Nodo principal:
	import rospy, threading, sys, random, math, numpy, roslaunch
	from pynput.keyboard import Key, Listener
	from std_msgs.msg import Float32MultiArray
	from geometry_msgs.msg import Twist
	
	Graficador:
	import rospy
	import matplotlib.pyplot as plt
	import matplotlib.animation as animation
	from geometry_msgs.msg import Twist
	from std_msgs.msg import Float32MultiArray

Al correr alguno de los comandos anteriores e iniciar la simulacion se abrira una grafica en donde se representa al robot como un punto de color azul y a los obstaculos (ya sea como puntos crudos o lineas, dependiendo del iteral que se corra) de color rojo. El robot se puede mover con las flechas del teclado, presionando hacia adelante o atras para moverlo en dicha direccion y oprimiendo derecha o izquierda para girar mientras se mueve.

