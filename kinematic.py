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

# Air density of the planet 
EARTH_AIR_DENSITY = 1.293 #kg m^-3
MOON_AIR_DENSITY = 0
MARS_AIR_DENSITY = 0.020 #kg m^ -3
VENUS_AIR_DENSITY = 65 # kg m^-3
JUPITER_AIR_DENSITY = 0.16 # kg m^-3

#Gravity (#m/s^2) from  https://www.smartconversion.com/factsheet/solar-system-gravity-of-planets
EARTH_GRAVITY = 9.81    #m/s^2
MOON_GRAVITY = 1.6      #m/s^2
MARS_GRAVITY = 3.71     #m/s^2
VENUS_GRAVITY = 8.9     #m/s^2
JUPITER_GRAVITY = 23.1  #m/s^2


# need to add function to account for
    #   DRAG
    #   WEIGHT
    #   VELOCITY AT A CERTAIN TIME
    #   POSITION AT A CERTAIN TIME
    #   ACCELERATION AT A  CERTAIN TIME

# DEVELOPED USER INTERACTION TO CHANGE THE VARAIBLE 

# PLOT THE GIVEN INFORMATION

# SOME HOW GET THE WEBSITE THE INFORMATION