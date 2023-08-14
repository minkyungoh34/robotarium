import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

import numpy as np
import time

'''
This script drives the robot straight (linear movement).
Written by Minkyung Oh
'''
#Number of robots
N = 1

#Initialize the Robotarium object
r = robotarium.Robotarium(number_of_robots = N, show_figure = True, sim_in_real_time = True)

#Set the number of iterations
iterations = 1000

#Linear velocity in m/s
velocity = 0.05

#Get the most recent poses of the robot
x = r.get_poses()

#Iterator
r.step()

#Algorithm
#For the number of iterations
for i in range (iterations):
    #Get poses of the robot
    x = r.get_poses()
    #Set the velocity of the robot
    r.set_velocities(np.arange(N),np.array([[velocity],[0]]))
    #Iterator
    r.step()
#Call at the end to debug
r.call_at_scripts_end()
