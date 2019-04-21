#!/usr/bin/env python3

from math import cos
from math import sin
from math import pi
from math import atan
import numpy as np
from numpy.linalg import inv

class BBR:
    """ A class to simulate the behavior of a BBR
    """

    def __init__(self):
        """ The constructor of the class """

        # variables for the model        
        self.g = None   # m/s^2   Acceleration due to gravity
        self.L = None   # Distance from the center of the ball to the COM of the robot
        self.Mball = None   # Mass of the ball
        self.Mbody = None   # mass of the robot
        self.Iball = None   # moment of inertia of the ball
        self.Rball = None   # Radius of th ball
        self.Rbody = None   # Radius of the robot
        self.Hbody = None   # Height of the robot
        self.rhoBody = None # Density of the robot

        self.x_Ibody = None   # moment of inertia of the robot (xz plane)
        self.y_Ibody = None   # moment of inertia of the robot (yz plane)


        # variables for dynamic evaluation
        self.x_phi = None      # angle of the robot xz plane
        self.x_phip = None     # angular speed of the robot xz plane
        self.x_phipp = None    # angular acceleration of the robot xz plane
        self.x_theta = None    # rotation of the ball xz plane
        self.x_thetap = None   # rotation angular speed of the ball xz plane
        self.x_thetapp = None  # rotation angular acceleration of the ball xz plane

        self.y_phi = None      # angle of the robot yz plane
        self.y_phip = None     # angular speed of the robot yz plane
        self.y_phipp = None    # angular acceleration of the robot yz plane
        self.y_theta = None    # rotation of the ball yz plane
        self.y_thetap = None   # rotation angular speed of the ball yz plane
        self.y_thetapp = None  # rotation angular acceleration of the ball yz plane

        self.initRobot()  # to initialize all the parameters of the system

    def initRobot(self):
        """ Function to initialize all the parameters of the system
            The value presented here are the value from the paper where the 
            model come from.
        """
        # variables for the model
        self.g = 9.81     # m/s^2   Acceleration due to gravity
        self.Mball = 0.6   # Mass of the ball
        self.Mbody = 5   # mass of the robot
        self.Rball = 0.24   # Radius of th ball
        self.Rbody = 0.30   # Radius of the robot
        self.Hbody = 0.40   # Height of the robot
        self.L = self.Rball + self.Hbody/2  # Distance from the center of the ball to the COM of the robot

        self.x_phi = pi/6  # the BBR is initially stable if phi and theta == 0
        self.x_phip = 0
        self.y_phi = pi/12
        self.y_phip = 0

        self.x_theta = 0
        self.y_theta = 0
        self.x_thetap = 0
        self.y_thetap = 0

        self.rhoBody = self.Mbody / (pi*self.Rbody*self.Hbody)

        self.x_Ibody = self.rhoBody*((self.Rbody**3)*pi/3 + ((self.L-self.Hbody/2 + self.Hbody)**3 - (self.L-self.Hbody/2)**3)/3)
        self.y_Ibody = self.rhoBody*((self.Rbody**3)*pi/3 + ((self.L-self.Hbody/2 + self.Hbody)**3 - (self.L-self.Hbody/2)**3)/3)

        self.Iball = 2/5*self.Mball*(self.Rball**2)



    def f(self, x_theta, x_thetap, x_phi, x_phip, y_theta, y_thetap, y_phi, y_phip, deltat, F):
        """ Function to evaluate the new state of the system according to:
            - the mathematical model
            - the current state
            - the time step deltat
            - the command F=[tau1, tau2]
        """

        tau1 = F[0]
        tau2 = F[1]

        #the new state is calculated for both planes, xz and yz

        # the mass matrix M
        m = self.Mbody*self.L**2
        Mx = np.array([[self.Iball + (self.Mball+self.Mbody)*self.Rball**2 + m + 2*self.Mbody*self.Rball*self.L*cos(x_theta + x_phi), m + self.Mbody*self.Rball*self.L*cos(x_theta + x_phi)],
                       [m + self.Mbody*self.Rball*self.L*cos(x_theta + x_phi), m + self.x_Ibody]])
        My = np.array([[self.Iball + (self.Mball+self.Mbody)*self.Rball**2 + m + 2*self.Mbody*self.Rball*self.L*cos(y_theta + y_phi), m + self.Mbody*self.Rball*self.L*cos(y_theta + y_phi)],
                       [m + self.Mbody*self.Rball*self.L*cos(y_theta + y_phi), m + self.y_Ibody]])

        # the vector of coriolis and centrifugal forces
        Cx = np.array([[-self.Mball * self.Rball * self.L * sin(x_theta + x_phi) * ((x_thetap + x_phip) ** 2)],
                       [0]])
        Cy = np.array([[-self.Mball * self.Rball * self.L * sin(y_theta + y_phi) * ((y_thetap + y_phip) ** 2)],
                       [0]])

        # the vector of gravitational forces
        Gx = np.array([[-self.Mbody * self.g * self.L * sin(x_theta + x_phi)],
                       [-self.Mbody * self.g * self.L * sin(x_theta + x_phi)]])
        Gy = np.array([[-self.Mbody * self.g * self.L * sin(y_theta + y_phi)],
                       [-self.Mbody * self.g * self.L * sin(y_theta + y_phi)]])


        x_xp = inv(Mx) * (np.array([[0], [tau1]]) - Cx - Gx)
        y_xp = inv(My) * (np.array([[0], [tau2]]) - Cy - Gy)

        #the new state
        x_thetapp = x_xp[0][0]
        x_phipp = x_xp[1][0]
        y_thetapp = y_xp[0][0]
        y_phipp = y_xp[1][0]

        return x_thetap*deltat, x_thetapp*deltat, x_phip*deltat, x_phipp*deltat, y_thetap*deltat, y_thetapp*deltat, y_phip*deltat, y_phipp*deltat

    def runge_kutta(self, deltat, F):
        """ Function that call the f() function defined above, and uses a 
            runge-kutta approach to deal with the differential equations
        """
        k1x_thetap, k1x_thetapp, k1x_phip, k1x_phipp, k1y_thetap, k1y_thetapp, k1y_phip, k1y_phipp = \
            self.f(self.x_theta, self.x_thetap, self.x_phi, self.x_phip, self.y_theta, self.y_thetap, self.y_phi, self.y_phip, deltat, F)
        k2x_thetap, k2x_thetapp, k2x_phip, k2x_phipp, k2y_thetap, k2y_thetapp, k2y_phip, k2y_phipp = \
            self.f(self.x_theta + k1x_thetap/2, self.x_thetap + k1x_thetapp/2, self.x_phi + k1x_phip/2, self.x_phip + k1x_phipp/2, self.y_theta + k1y_thetap/2, self.y_thetap + k1y_thetapp/2, self.y_phi + k1y_phip/2, self.y_phip + k1y_phipp/2, deltat, F)
        k3x_thetap, k3x_thetapp, k3x_phip, k3x_phipp, k3y_thetap, k3y_thetapp, k3y_phip, k3y_phipp = \
            self.f(self.x_theta + k2x_thetap/2, self.x_thetap + k2x_thetapp/2, self.x_phi + k2x_phip/2, self.x_phip + k2x_phipp/2, self.y_theta + k2y_thetap/2, self.y_thetap + k2y_thetapp/2, self.y_phi + k2y_phip/2, self.y_phip + k2y_phipp/2, deltat, F)
        k4x_thetap, k4x_thetapp, k4x_phip, k4x_phipp, k4y_thetap, k4y_thetapp, k4y_phip, k4y_phipp = \
            self.f(self.x_theta + k3x_thetap, self.x_thetap + k3x_thetapp, self.x_phi + k3x_phip, self.x_phip + k3x_phipp, self.y_theta + k3y_thetap, self.y_thetap + k3y_thetapp, self.y_phi + k3y_phip, self.y_phip + k3y_phipp, deltat, F)

        self.x_theta = self.x_theta + (k1x_thetap+2*k2x_thetap+2*k3x_thetap+k4x_thetap)/6
        self.x_thetap = self.x_thetap + (k1x_thetapp+2*k2x_thetapp+2*k3x_thetapp+k4x_thetapp)/6
        self.x_phi = self.x_phi + (k1x_phip+2*k2x_phip+2*k3x_phip+k4x_phip)/6
        self.x_phip = self.x_phip + (k1x_phipp+2*k2x_phipp+2*k3x_phipp+k4x_phipp)/6
        self.y_theta = self.y_theta + (k1y_thetap+2*k2y_thetap+2*k3y_thetap+k4y_thetap)/6
        self.y_thetap = self.y_thetap + (k1y_thetapp+2*k2y_thetapp+2*k3y_thetapp+k4y_thetapp)/6
        self.y_phi = self.y_phi + (k1y_phip+2*k2y_phip+2*k3y_phip+k4y_phip)/6
        self.y_phip = self.y_phip + (k1y_phipp+2*k2y_phipp+2*k3y_phipp+k4y_phipp)/6

    def dynamics(self, deltat, F):
        """ Function to provide an interface between the model and the display
        """
        self.runge_kutta(deltat, F)