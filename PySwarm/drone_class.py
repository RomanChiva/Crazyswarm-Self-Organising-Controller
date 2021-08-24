import numpy as np
import scipy as sp 
import random 
import time
from PySwarm.functions_sensors import  position_change, PID, process_sensor_data, angle_to_bearing, bearing_to_state, sensor_input
import pandas as pd




class drone(object):

    def __init__(self,cf,initial_position,dist,action_states):

        self.cf = cf
        self.dist = dist
        self.action_states = action_states
        

        #Regarding Position:
        self.target_position = initial_position
        self.neighborhood = None

        #Regarding actions:
        self.cf.ledRGB = 'orange'
        self.neighbors_static = None # Are any of my neighbors moving?
        self.stationary = True # Am I moving?
        self.currently_doing = 'still' # Current action
        self.threshold = 1 # Trigger probablility to take action
        self.k_move = 0.7 # PID proportionality gain
        self.k_adjust = 1.5 # Adjustment gain
        self.k_repel = 1.5 # Repulsion gain
        self.k_attract = 1.5 # Attraction gain
        
        # Trigger simulation
        



    # Function updates position of drone while moving:

    def move(self):
       
        PID(self.cf,self.target_position, self.k_move, self.dist)

                
        

    # Function updates position of drone while still: (placeholder)

    def still(self):

        self.cf.cmdVelocityWorld([0,0,0],0)



    # Function updaes position of drone while adjusting

    def adjust(self):
        
        v = np.array([0,0])

        relative_positions = self.neighborhood - np.array(self.cf.position()[:2])
        distances = np.array([np.linalg.norm(i) for i in relative_positions])
        
        
        # Attraction

        if np.all(distances > self.dist*np.sqrt(2)*1.01):
            print('attraction')
            for x, pos in enumerate(relative_positions):

                unit_vector = pos/distances[x]
                
                speed = self.k_attract*(distances[x]- self.dist*np.sqrt(2))**2
                vector = unit_vector*speed
                v = v + vector

            self.cf.cmdVelocityWorld([v[0], v[1],0],0)
        

        # Repulsion

        elif np.any(distances < self.dist*0.97):
            print('repulsion')
            direction = relative_positions[np.where(distances == np.amin(distances))][0]
            dist = np.amin(distances)
            unit_vector = direction/dist
    
            speed = self.k_repel*(np.amin(distances)- self.dist)**2

            vector = -1*unit_vector*speed
            v = v + vector

            self.cf.cmdVelocityWorld([v[0], v[1],0],0)


         # Adjustment   
        
        else:
            print('adjustment')
            for x,pos in enumerate(relative_positions):

                angle = np.arctan2(pos[1], pos[0])*(180/np.pi)
                bearing = angle_to_bearing(angle)
                state = bearing_to_state(bearing)
                change = np.array(position_change(state, self.dist))
                target_position = np.array(self.neighborhood[x]) - change

                vector = target_position - np.array(self.cf.position()[:2])
                velocity = vector*self.k_adjust
                v = v + velocity

                self.cf.cmdVelocityWorld([v[0], v[1],0],0)

        
        




    # Controller of drone. Recieves sensor input and decides what action to take

    def action_manager(self, drones):
        
        sensor_input(self.dist, drones)
        if self.currently_doing == 'still':

            if self.neighbors_static.all():
                self.currently_doing = 'adjust'
                self.cf.ledRGB = 'orange'
                self.adjust()

            else:
                self.still()
        
        elif self.currently_doing == 'adjust':
            
            # Figure out where you wanna go
            target_state = process_sensor_data(self.cf.position()[:2], self.neighborhood, self.action_states)
            
            if self.neighbors_static.all() and random.randint(0,100) < self.threshold and target_state >=0:
                
                self.target_position = np.array(self.cf.position()[:2]) + np.array(position_change(target_state, self.dist))
                self.stationary = False
                self.currently_doing = 'move'
                self.cf.ledRGB = 'green'
                self.move()
                
            elif self.neighbors_static.all() != True:
                self.currently_doing = 'still'
                self.cf.ledRGB = 'red'
                self.stationary = True
                self.still()
            else:
                self.adjust()
                
                
        elif self.currently_doing == 'move':

            if self.neighbors_static.all() != True:
                self.stationary = True
                self.currently_doing = 'still'
                self.cf.ledRGB = 'red'
                self.still()
            
            elif np.linalg.norm(np.array(self.target_position) -np.array(self.cf.position()[:2])) < 0.1:
                self.stationary = True
                self.currently_doing = 'adjust'
                self.cf.ledRGB = 'orange'
                self.adjust()
                
            else:
                self.move()
                
            



            
 



       


    

