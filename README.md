# Ball-Balancing-Robot-Python-Simulator

 Here is presented a source code to simulate a ball balancing robot with pyhton and using OpenGL to display the simulated robot.

This simulator has three files:

    bbr.py that provides a class BBR that deals with the mathematical model of the robot and with the resolution of the differencial equations.

    pid.py that provides a PID class to be able to control the robot.

    main.py that uses OpenGL to display the robot and provides the menu to use the simulation

Note that it is based on python 3. It needs numpy and OpenGL.

## bbr.py file

This file provides the BBR class. This class contains a mathematical model for the ball balancing robot. 
The BBR class uses a Runge-Kutta approach to compute the differential equations of the robot's dynamic.

## pid.py file

This file provides a simple discrete PID class that is used to control the robot in the simulation. This implementation was found here. 

## main.py file

This is the main file of the simulator. It uses OpenGL to display the state of the BBR and uses PID to control the robot. It uses 2 PIDs : one for the robot angle in the xz plane and one for the robot angle in the yz plane. The OpenGL part is based on a Jean-Baptiste Fasquel file (http://perso-laris.univ-angers.fr/~fasquel). 
<!--stackedit_data:
eyJoaXN0b3J5IjpbLTE2MTA4MTkyNTFdfQ==
-->
