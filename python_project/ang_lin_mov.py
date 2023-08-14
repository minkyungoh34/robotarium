import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

import numpy as np
import time
'''
This script makes the robot go straight then spin around in place.
Written by Minkyung Oh
'''
#Number of robots
N = 1

#Initialize the Robotarium object
r = robotarium.Robotarium(number_of_robots = N, show_figure = True, sim_in_real_time = True)

#Set the number of linear iterations
lin_iterations = 500

#Set the number of angular iterations
ang_iterations = 1000

#Linear velocity in m/s
lin_vel = 0.05

#Angular velocity in rad/s
angular_vel = 0.2

#Get the most recent poses of the robot
x = r.get_poses()

#Iterator
r.step()

#Algorithm for moving straight (linear)
#For the number of iterations
for i in range (lin_iterations):
    #Get poses of the robot
    x = r.get_poses()
    #Set the velocity of the robot
    r.set_velocities(np.arange(N),np.array([[lin_vel],[0]]))
    #Iterator
    r.step()

#Algorithm for spinning around (angular)
#For the number of iterations
for i in range (ang_iterations):
    #Get poses of the robot
    x = r.get_poses()
    #Set the velocity of the robot
    r.set_velocities(np.arange(N),np.array([[0],[angular_vel]]))
    #Iterator
    r.step()
    
#Call at the end to debug
r.call_at_scripts_end()

