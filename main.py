#!/usr/bin/env python3

""" main.py uses OpenGL to display a Ball balancing robot and uses the PID class to
    provide a command for the robot.
    This file is based on the work of Jean-Baptiste Fasquel,
    ISTIA - Angers university Jean-Baptiste.Fasquel@univ-angers.fr

    Copyright (C) 2017 
    Remy GUYONNEAU, ISTIA/LARIS, University of Angers (France)
    remy.guyonneau@univ-angers.fr
	
	Azzeddine LEMOUAKNI, ISTIA, University of Angers (France)
	azzeddine.lemouakni@etud.univ-angers.fr

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>."""

import sys as sys;
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from math import cos
from math import sin
from math import pi
from math import sqrt
from bbr import BBR
from pid import PID


print("<-------------------------------------------------------------->")
print("<--            Ball Balancing Robot simulator                -->")
print("<-- developped by Remy GUYONNEAU, ISTIA/LARIS                -->")
print("<--               Azzeddine LEMOUAKNI, ISTIA                 -->")
print("<-- University of Angers                                     -->")
print("<-------------------------------------------------------------->\n")

print(" Page down : activate simulation")
print(" Page up : stop simulation")
print(" F5 : ON/OFF PID correction")
print(" F6 : reset simulation")
print(" F7 : add pertubation (push the robot)")
print(" F8 : add pertubation (push the robot)")

# -- Programme variables --

# variables for the animation of the dynamics
# ref_time : to compute the time between two redisplay$
# FPS : Frame per second, to 
ref_time, FPS = 0, 10

# position of the camera (glulookat parameters)
CameraPosX = 0.0
CameraPosY = 3
CameraPosZ = 10.0
ViewUpX = 0.0
ViewUpY = 1.0
ViewUpZ = 0.0
CenterX = 0.0
CenterY = 0.0
CenterZ = 0.0
follow_robot = False  # so that the camera will follow the robot or not

# to deal with the camera rotation and zoom
Theta,dtheta=0.0,2*pi/100.0
Radius = sqrt( CameraPosX**2+CameraPosZ**2)


# the balancing robot
myBot = BBR()

#PID on the robot's rotation angle in the xz plane
myPIDx_angle = PID()

#PID on the robot's rotation angle in the yz plane
myPIDy_angle = PID()

# to draw the robot at its actual position in the world
dst=myBot.Hbody + myBot.Rball/2
downX = False
downY = False
newXdown = 0
newYdown = 0
TranslationBallx = 0
TranslationBally = 0
angleRobotX = myBot.x_phi+myBot.x_theta
angleRobotY = myBot.y_phi + myBot.y_theta

# the command of the robot's wheels
F = [0, 0]

# activate or distactivate the PIDs correction
use_pid = False

# to deal with the "moving forwars" and "turning" commands
speed = 0
current_speed = 0
turn = 0
current_turn = 0

def initPIDs():
    """ Function that initializes the PIDs parameters (Kp, Ki, Kd)
    """
    global  myPIDx_angle, myPIDy_angle

    myPIDx_angle.setKp(50.0)
    myPIDx_angle.setKi(1)
    myPIDx_angle.setKd(60.0)
    myPIDx_angle.setPoint(0)

    myPIDy_angle.setKp(50.0)
    myPIDy_angle.setKi(1)
    myPIDy_angle.setKd(60.0)
    myPIDy_angle.setPoint(0)




def correction():
    """ Function that uses the PID and the robot state to generate a new 
        command according to the two angles of the robot
    """
    global myBot, F, use_pid, myPIDx_angle, myPIDy_angle

    if (use_pid):

        anglex = myPIDx_angle.update(myBot.x_phi + myBot.x_theta)
        angley = myPIDy_angle.update(myBot.y_phi + myBot.y_theta)

        F = [-anglex, -angley]

    else:
        F = [0, 0]


def animation():
    """ Function to compute the robot state at each time step and to draw it in the world
    """
    global ref_time, FPS, F, myBot, angleRobotX, angleRobotY, downX, newXdown, downY, newYdown
    # FPS expressed in ms between 2 consecutive frame
    delta_t = 0.001 # the time step for the computation of the robot state
    if glutGet(GLUT_ELAPSED_TIME)-ref_time > (1.0/FPS)*1000 :
        # at each redisplay (new display frame)
        dst = 0
        for i in range(0,100):
            # we want the computation of the robot state to be faster than the 
            # display to limit the compution errors
            # display : new frame at each 100ms
            # deltat : 1ms for the differential equation evaluation
            myBot.dynamics(delta_t, F)
            # to calculate the new angles
            angleRobotX = myBot.x_phi + myBot.x_theta
            angleRobotY = myBot.y_phi + myBot.y_theta
            # to check if the robot is on the ball.
            if(angleRobotX >= pi/2 and not downY):
                angleRobotX = pi/2
                downX=True
                newXdown -=0.02
            elif(angleRobotX <= -pi/2 and not downY):
                angleRobotX = -pi/2
                downX = True
                newXdown += 0.02
            if (angleRobotY >= pi / 2 and not downX):
                angleRobotY = pi / 2
                downY= True
                newYdown += 0.02
            elif (angleRobotY <= -pi / 2 and not downX):
                angleRobotY = -pi / 2
                downY = True
                newYdown -= 0.02
        correction()  # calls the PIDs if enable
        glutPostRedisplay()  # refresh the display
        ref_time=glutGet(GLUT_ELAPSED_TIME)

def drawBall(bbr):
    global downX, downY, TranslationBallx, TranslationBally
    glPushMatrix()
    glColor3d(1, 0.5, 0)
    #Translation of the ball
    if(not downX and not downY):
        TranslationBallx = - bbr.Rball*bbr.x_theta
        TranslationBally = - bbr.Rball*bbr.y_theta
    glTranslatef(TranslationBallx,0,0)
    glTranslatef(0, 0, -TranslationBally)
    #Rotation of the ball
    glRotatef(-myBot.x_theta * (180 / pi), 0, 1, 0)
    glRotatef(-myBot.y_theta * (180 / pi), 1, 0, 0)

    glutSolidSphere(bbr.Rball,20,20)
    glPopMatrix()

def drawCylindre(bbr):
    global  downX, downY,newXdown, TranslationBallx, TranslationBally
    glPushMatrix()
    glColor3d(0, 0, 1)
    glRotatef(90,1,0,0)

    #Translation of the robot
    glTranslatef(TranslationBallx, 0, 0)
    glTranslatef(0, -TranslationBally, 0)

    #Rotation of the robot
    glRotatef(angleRobotX*(180/pi),0,1,0)
    glRotatef(angleRobotY * (180 / pi), 1, 0, 0)

    # put the robot on the ball
    glTranslatef(0,0,-(bbr.Hbody + bbr.Rball))

    #If the robot falls down
    if(downX):
        glTranslatef(newXdown,0,0)
    if(downY):
        glTranslatef(0, newYdown, 0)

    glutSolidCylinder(bbr.Rbody,bbr.Hbody,20,20)
    glPopMatrix()


def Displayfct():
    """ Function to draw the world
    """

    scaleCoeff = 10  # to scale the robot display

    glClearColor(0,0,0,0)  # background color
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    # Projection
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    glFrustum(-1,1,-1,1,1,80.)

    # Camera position and orientation
    global CameraPosX, CameraPosY, CameraPosZ, CenterX, CenterY, CenterZ, ViewUpX, ViewUpY, ViewUpZ
    global myBot, follow_robot



    gluLookAt(CameraPosX, CameraPosY , CameraPosZ, CenterX, CenterY, CenterZ, ViewUpX, ViewUpY, ViewUpZ)

    # Draw the objects and geometrical transformations
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    drawBall(myBot)
    drawCylindre(myBot)


    # To efficient display
    glutSwapBuffers()


def ReshapeFunc(w,h):
    """ Function calls when reshaping the window
    """
    glViewport(0,0,w,h)

def rotate_camera(angle):
    """ Function to rotate the camera according to an angle
        angle : step angle for the camera rotation
    """
    global CameraPosX,CameraPosZ,Radius,Theta
    Theta+=angle
    CameraPosZ=Radius*cos(Theta)
    CameraPosX=Radius*sin(Theta)
    return 0

def zoom_camera(factor):
    """ Function to zoom the camera according to a factor
        factor : zoom factor
    """
    global CameraPosX,CameraPosY,CameraPosZ,Radius
    # Update camera center
    CameraPosX, CameraPosY, CameraPosZ = factor*CameraPosX, factor*CameraPosY, factor*CameraPosZ
    # Update radius (for next rotations)
    Radius = sqrt( CameraPosX**2 + CameraPosZ**2 )

def SpecialFunc(skey,x,y):
    """ Function to handle the keybord keys
    """
    global CameraPosY, Theta, dtheta
    global myBot, speed, use_pid, turn
    global angleRobotX, angleRobotY, follow_robot
    global downX, downY, newXdown, newYdown, TranslationBallx, TranslationBally

    if glutGetModifiers() == GLUT_ACTIVE_SHIFT:
        # SHIFT pressed
        if skey == GLUT_KEY_UP :
            CameraPosY+=0.3  # put the camera higher
        if skey == GLUT_KEY_DOWN :
            CameraPosY-=0.3  # put the camera lower
    else:
        # standard
        if skey == GLUT_KEY_LEFT :
            rotate_camera(-dtheta)
        elif skey == GLUT_KEY_RIGHT :
            rotate_camera(dtheta)
        elif skey == GLUT_KEY_UP :
            zoom_camera(0.9)
        elif skey == GLUT_KEY_DOWN :
            zoom_camera(1.1)
        elif skey == GLUT_KEY_PAGE_DOWN : 
            print("\tSimulation ON")
            glutIdleFunc(animation)
        elif skey == GLUT_KEY_PAGE_UP :
            print("\tSimulation PAUSE")
            glutIdleFunc(None)
        elif skey == GLUT_KEY_F5 : 
            use_pid = not(use_pid)
            print("\tUsing PID : " + str(use_pid))
            if(use_pid == True):
                initPIDs()
        elif skey == GLUT_KEY_F6 :
            downX = False
            downY = False
            newXdown = 0
            newYdown = 0
            TranslationBallx = 0
            TranslationBally = 0
            myBot.initRobot()
            angleRobotX = myBot.x_phi + myBot.x_theta
            angleRobotY = myBot.y_phi + myBot.y_theta
            initPIDs()
        elif skey == GLUT_KEY_F7 :
            myBot.x_phi += (10 * pi / 180)
            myBot.y_phi += (10 * pi / 180)
        elif skey == GLUT_KEY_F8 :
            myBot.x_phi += (-20 * pi / 180)
            myBot.y_phi += (-20 * pi / 180)

    glutPostRedisplay()
    return 0

# Initialization of the Window stuff
glutInit(sys.argv)
glutInitWindowPosition(100,100)
glutInitWindowSize(250,250)
glutInitDisplayMode(GLUT_RGBA |GLUT_DOUBLE | GLUT_DEPTH)
glutCreateWindow(b"BBR simulator")

# Opengl Initialization      
glClearColor(0.0,0.0,0.0,0.0)
glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
glEnable(GL_DEPTH_TEST) # Hidden objects are not drawn

# Display are reshape function
glutDisplayFunc(Displayfct)
glutReshapeFunc(ReshapeFunc)
glutSpecialFunc(SpecialFunc)

# Infinite loop
glutMainLoop()
