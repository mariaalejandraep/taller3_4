# taller3_4
Punto 1:
A) comando de ejecución:
rosrun taller3_4 punto1a.py
C) comando de ejecución
rosrun taller3_4 punto1c.py
Punto 2:
Cabe que vrep debe estar corrieno al igual que roscore, se debe escoger como escena scene_punto2_1piso.ttt, puede ejecutar el código o la simulación primero.
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
