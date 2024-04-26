"""
Author: Kelechi Agommuo, David Olufade, Rajjya Rhan Paudyal
ID Numbers: XM33737, HP56857, SR3095
Created: 9 April 2024



"""
import numpy as np
from enum import Enum


# Varibles that can be change by the user such as the object mass, enviroment (air resistance ,graviattional force, friction)
MASS = 0
V_INIT = 0
SIZE = 
RADIUS = 0.5 * SIZE

# Air density of the planet
EARTH_AIR_DENSITY = 1.293 #kg m^-3
MOON_AIR_DENSITY = 0
MARS_AIR_DENSITY = 0.020 #kg m^ -3
VENUS_AIR_DENSITY = 65 # kg m^-3
JUPITER_AIR_DENSITY = 0.16 # kg m^-3
all_densities = np.array([1.293, 0, 0.020, 64, 0.16]) # Putting all gravities from current densities into list

#Gravity (#m/s^2) from  https://www.smartconversion.com/factsheet/solar-system-gravity-of-planets
EARTH_GRAVITY = 9.81    #m/s^2
MOON_GRAVITY = 1.6      #m/s^2
MARS_GRAVITY = 3.71     #m/s^2
VENUS_GRAVITY = 8.9     #m/s^2
JUPITER_GRAVITY = 23.1  #m/s^2
all_gravities = np.array([9.81, 1.6, 3.71, 8.9, 23.1]) # Putting all gravities from current planets into list

#Drag coeffiencinnt from https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/shape-effects-on-drag-2/
SPHERE_DRAG_COE = .5
AIRFOIL_DRAG_COE = .045
BULLET_DRAG_COE = .295
FLAT_PLATE_DRAG_COE = 1.28
PRISM_DRAG_COE = 1.14

#
PRISM_BASE = SIZE * SIZE
PRISM_HEIGHT = 1.5 * SIZE


SA_Sphere = 4 * np.pi * SIZE**2

SA_BULLET_CONE = np.pi * RADIUS * np.sqrt(SIZE**2 + RADIUS**2)
SA_BULLET_CYL = 2 * np * RADIUS * SIZE + np.pi * RADIUS**2
SA_BULLET = SA_BULLET_CONE + SA_BULLET_CYL



SA_PRISM =  



#
curr_velo = 0.0



# List to hold selected planet information
selected_planet = []
selected_shape = []

# need to add function to account for
    #   DRAG
    def drag_at_time(curr_velo):
        D = selected_shape * selected_planet * ((curr_velo**2)/2) * 


    #   WEIGHT
    def net_force(obj_weight):
        return (obj_weight * selected_planet[1])
    #   VELOCITY AT A CERTAIN TIME
    def velocity_at_time(time):
        pass

    #   POSITION AT A CERTAIN TIME
    def position_at_time(time):
        pass
    #   ACCELERATION AT A  CERTAIN TIME
    def accel_at_time(time):
        pass

# DEVELOPED USER INTERACTION TO CHANGE THE VARAIBLE
# User Input
user_inp = input("What planet would you like to select? (1-5)")

# Selecting planet info to append into list based on user choice
selected_planet.append(all_densities[int(user_inp) - 1])
selected_planet.append(all_gravities[int(user_inp) - 1])

MASS = float(input("Enter the object's mass (in kg): "))
V_INIT = float(input("Enter the initial velocity (in m/s): "))

# PLOT THE GIVEN INFORMATION

# SOME HOW GET THE WEBSITE THE INFORMATION
