import numpy as np
import scipy as sp 
import random 
import time
from PySwarm.drone_class import drone 
from PySwarm.functions_sensors import initial_position_generator, read_sam, YAML_gen, run
from pycrazyswarm import Crazyswarm
from main import move_random


#PARAMETERS-----------------------------

N_drones = 4 # Number of drones in the simulation
separation = 1 # Separation between the drones
initial_position = [1,1] # Initial position. Others spawned around it
pattern = 'lineNE' # Choose between: 'lineNE', 'triangle3', 'triangle4', 'triangle9'


# Simulation Settings ------------------------------------

path = 'PySwarm/state_action_matrices/state_action_matrix_{}.txt'.format(pattern)
action_states = read_sam(path) #Reads file with phat state action matrix


initial_positions = initial_position_generator(initial_position, N_drones, separation)
YAML = YAML_gen(initial_positions)

# Create Swarm---------------------------------------------------------------------

# Create swarm object
swarm = Crazyswarm(YAML)

#Create Timer
timeHelper = swarm.timeHelper

# Create an object for when you want to control all the crazyflies at once
allcf = swarm.allcfs
if __name__ == '__main__':

    TAKEOFF_DURATION = 5

    allcf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)

#Create individual controllers
drones = [drone(cf,initial_positions[x],separation,action_states) for x,cf in enumerate(allcf.crazyflies)] # Assign CFs to their ontrollers

#RUN---------------------------------------------

run(drones, timeHelper)
