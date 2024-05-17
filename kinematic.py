"""
Author: Kelechi Agommuo, David Olufade, Rajjya Rhan Paudyal
ID Numbers: XM33737, HP56857, SR3095
Created: 9 April 2024
Description: This program compute the projectile motion of an object in different enviroment accounting for drag


"""
import numpy as np
import matplotlib.pyplot as plt


# Varibles that can be change by the user such as the object mass, enviroment (air resistance ,graviattional force, friction)
Mass = 5
V_Init = 30
Size = 1
Radius = 0.5 * Size
theta = 0

# Air density of the planet
NULL_DENSITY = 0
EARTH_AIR_DENSITY = 1.293 #kg m^-3
MOON_AIR_DENSITY = 0
MARS_AIR_DENSITY = 0.020 #kg m^ -3
VENUS_AIR_DENSITY = 65 # kg m^-3
JUPITER_AIR_DENSITY = 0.16 # kg m^-3

#Gravity (#m/s^2) from  https://www.smartconversion.com/factsheet/solar-system-gravity-of-planets
NULL_GRAVITY = 0
EARTH_GRAVITY = 9.81    #m/s^2
MOON_GRAVITY = 1.6      #m/s^2
MARS_GRAVITY = 3.71     #m/s^2
VENUS_GRAVITY = 8.9     #m/s^2
JUPITER_GRAVITY = 23.1  #m/s^2

#Drag coeffiencinnt from https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/shape-effects-on-drag-2/
NULL_DRAG_COE = 0
SPHERE_DRAG_COE = .5
AIRFOIL_DRAG_COE = .045
BULLET_DRAG_COE = .295
FLAT_PLATE_DRAG_COE = 1.28
PRISM_DRAG_COE = 1.14

#Cross section area
global Shape_List
global Airfoil_CS
global Bullet_CS
global Flat_PLATE_CS
global Prism_DRAG_CS

Sphere_CS = np.pi * Radius**2       #m^2
Airfoil_CS = Size * 0.25 * Size     #m^2
Bullet_CS = np.pi * Radius**2       #m^2
Flat_PLATE_CS = Size * Size         #m^2
Prism_DRAG_CS = Size * Size         #m^2


# List to hold selected planet information
AIR_DENSITY_LIST = np.array([NULL_DENSITY, EARTH_AIR_DENSITY, MOON_AIR_DENSITY, MARS_AIR_DENSITY, VENUS_AIR_DENSITY, JUPITER_AIR_DENSITY], float) # Putting all gravities from current densities into list
GRAVITY_LIST = np.array([NULL_GRAVITY, EARTH_GRAVITY, MOON_GRAVITY, MARS_GRAVITY, VENUS_GRAVITY, JUPITER_GRAVITY], float) # Putting all gravities from current planets into list
DRAG_LIST = np.array([SPHERE_DRAG_COE,AIRFOIL_DRAG_COE, BULLET_DRAG_COE, FLAT_PLATE_DRAG_COE, PRISM_DRAG_COE ])
Shape_List = np.array([Sphere_CS, Airfoil_CS, Bullet_CS, Flat_PLATE_CS, Prism_DRAG_CS],  float)


def positionDrag(r, t, density_index, gravity_index, shape_index):
    #change in time
    dt = t - r[6] 

    velocity = np.sqrt(r[2] **2 + r[3]**2)

    #F_air = -1/2 * p(air density) * Area * drag_Coeffeient * | v |^2 * unit_vector
    F_air = -  0.5 * AIR_DENSITY_LIST[density_index] * Shape_List[shape_index] * DRAG_LIST[shape_index] * velocity**2

    #calculate the gravity
    F_gravity =  - GRAVITY_LIST[gravity_index]

    acel_x =  F_air /Mass * np.cos(theta)
    acel_y =  F_gravity + (F_air /Mass * np.sin(theta))

    velo_x = r[2] + acel_x * dt
    velo_y = r[3] + acel_y * dt

    # compute the position of the ball
    pos_x = r[0] + velo_x * dt
    pos_y = r[1] + velo_y * dt

    t += dt

    return np.array([pos_x, pos_y, velo_x, velo_y, acel_x, acel_y, t], float)

def rk4(r_init, t, h):
    r = r_init

    k1 = h * positionDrag(r, t)
    k2 = h * positionDrag((r + 0.5 * k1), (t + 0.5 * h))
    k3 = h * positionDrag((r + 0.5 * k2), (t + 0.5 * h))
    k4 = h * positionDrag((r+k3), (t+h))
 
    r = r + ((k1+ 2*k2 + 2*k3 + k4) /6)
 
    return r

def updateSize():
    global Shape_List
    global Airfoil_CS
    global Bullet_CS
    global Flat_PLATE_CS
    global Prism_DRAG_CS
    global Shape_List
    
    Radius = 0.5 * Size

    # Compue Cross section area of a new shape
    Sphere_CS = np.pi * Radius**2
    Airfoil_CS = Size * 0.25 * Size
    Bullet_CS = np.pi * Radius**2
    Flat_PLATE_CS = Size * Size 
    Prism_DRAG_CS = Size * Size 

    Shape_List = np.array([Sphere_CS, Airfoil_CS, Bullet_CS, Flat_PLATE_CS, Prism_DRAG_CS],  float)

def changeShape(r, h, density_index, gravity_index, shape_index):
    r_curr = r
    x_points = []
    y_points = []
    t = 0
    isbeing = True

    # Compute until the onject reaches the floor
    while (r_curr[1] >= 0 or isbeing == True):
    
        x_points.append(r_curr[0])
        y_points.append(r_curr[1])

        r_curr = positionDrag(r_curr, t, density_index, gravity_index, shape_index)

        t += h
        isbeing = False

    return x_points, y_points
   
def calcTrajectory(r, h, density_index, gravity_index, shape_index):
    t = 0
    r_curr = r
    
    x_points = []
    y_points = []
    
    # calculate until the object reaches the floor
    while (r_curr[1] >= 0 or isbeing == True):

        x_points.append(r_curr[0])
        y_points.append(r_curr[1])
    
        r_curr = positionDrag(r_curr, t, density_index, gravity_index, shape_index)

        t += h
        isbeing = False
        
    return x_points, y_points
      
if __name__ == '__main__':
    
    x_init = 0
    y_init = 0
    velo_x_init = 0
    velo_y_init = 0
    theta = np.radians(45)  # Initial angle
    V_Init =  58.33         # fastest  soccer ball is kick
    Mass = .45              # mass of a soccer ball
    Size = 0.113 * 2        # diamater of a soccer ball

    #array of all the data
    x_ball_earth = []
    y_ball_earth = []

    x_ball_moon = []
    y_ball_moon = []

    x_ball_mars = []
    y_ball_mars = []

    x_ball_venus = []
    y_ball_venus = []

    x_ball_jup = []
    y_ball_jup = []

    x_airfoil = []
    y_airfoil = []

    x_bullet = []
    y_bullet = []

    x_flat = []
    y_flat = []

    x_prism = []
    y_prism = []
    
    x_airfoil_moon = []
    y_airfoil_moon = []
    
    x_nodrag_earth = []
    y_nodrag_earth = []


    updateSize()


    user_planet_index = 1
    user_shape_index = 0

    # calculate the velocity into x and y components
    velo_x_init = V_Init * np.cos(theta)
    velo_y_init = V_Init * np.sin(theta)

    r = np.array([0, y_init, velo_x_init, velo_y_init, 0, 0, 0], float)

    t = 0
    h = 0.001


    # density and gravity on earth using a sphere   
    density_index = 1
    gravity_index = 1
    shape_index = 0
    x_ball_earth, y_ball_earth = calcTrajectory(r, h, density_index, gravity_index, shape_index)
    
    
    # density and gravity on moon using a sphere  
    density_index = 2
    gravity_index = 2
    shape_index = 0
    x_ball_moon, y_ball_moon = calcTrajectory(r, h, density_index, gravity_index, shape_index)
    
    # density and gravity on mars using a sphere   
    density_index = 3
    gravity_index = 3
    shape_index = 0
    x_ball_mars, y_ball_mars = calcTrajectory(r, h, density_index, gravity_index, shape_index)
    
    # density and gravity on venus using a sphere    
    density_index = 4
    gravity_index = 4
    shape_index = 0
    x_ball_venus, y_ball_venus = calcTrajectory(r, h, density_index, gravity_index, shape_index)
    
    # density and gravity on jupiter using a sphere    
    density_index = 5
    gravity_index = 5
    shape_index = 0
    x_ball_jup, y_ball_jup = calcTrajectory(r, h, density_index, gravity_index, shape_index)
    
    
    #SHAPES ON EARTH
    #using earth gravity and air density the shape will be change
    density_index = 1
    gravity_index = 1

    # using a airfoil
    shape_index = 1
    x_airfoil, y_airfoil = changeShape(r, h, density_index, gravity_index, shape_index)

    # using a bullet
    shape_index = 2
    x_bullet, y_bullet = changeShape(r, h, density_index, gravity_index, shape_index)

    # using flat plate
    shape_index = 3
    x_flat, y_flat = changeShape(r, h, density_index, gravity_index, shape_index)

    # using a flat rectangular prism
    shape_index = 4
    x_prism, y_prism = changeShape(r, h, density_index, gravity_index, shape_index)
    
    
    #change in atomshere
    density_index = 2
    gravity_index = 2
    shape_index = 2
    x_airfoil_moon, y_airfoil_moon = changeShape(r, h, density_index, gravity_index, shape_index)
    
    #earth with no drag
    density_index = 0
    gravity_index = 1
    shape_index = 0
    x_nodrag_earth, y_nodrag_earth = changeShape(r, h, density_index, gravity_index, shape_index)


    # PLOT THE GIVEN INFORMATION
    
    #ALL PLANET SAME SAHPE
    plt.plot(x_ball_earth, y_ball_earth, 'b', label="EARTH",)
    plt.plot(x_ball_moon, y_ball_moon, 'k', label="MOON")
    plt.plot(x_ball_mars, y_ball_mars, 'r', label="MARS")
    plt.plot(x_ball_venus, y_ball_venus, 'm', label="VENUS")
    plt.plot(x_ball_jup, y_ball_jup, 'y', label="JUPITER")
    plt.xlabel("Distance in x-axis (m)")
    plt.ylabel("Distance in y-axis (m)")
    plt.title("Soccer Ball Kicked on Different Planets")
    plt.legend(loc='upper right')
    plt.grid()

    # JUST VENUS
    plt.figure()
    plt.plot(x_ball_venus, y_ball_venus, 'm', label="VENUS")
    plt.xlabel("Distance in x-axis (m)")
    plt.ylabel("Distance in y-axis (m)")
    plt.title("Soccer Ball Kicked on Venus")
    plt.legend(loc='upper right')
    plt.grid()

    #DIFFERNET SHAPE ON EARTH
    plt.figure()
    plt.plot(x_ball_earth, y_ball_earth, 'b', label="SHPERE",)
    plt.plot(x_airfoil, y_airfoil, label="AIRFOIL")
    plt.plot(x_bullet, y_bullet, label="BULLET")
    plt.plot(x_flat, y_flat, label="FLAT_PLATE")
    plt.plot(x_prism, y_prism, label="REC_PRISM")
    plt.xlabel("Distance in x-axis (m)")
    plt.ylabel("Distance in y-axis (m)")
    plt.title("Differnt Shape Object Launched from Earth")
    plt.legend(loc='upper right')
    plt.grid()

    #Airfoil on moon
    plt.figure()
    plt.plot(x_airfoil_moon, y_airfoil_moon, label="AIRFOIL")
    plt.plot(x_ball_moon, y_ball_moon, 'k', label="MOON")
    plt.xlabel("Distance in x-axis (m)")
    plt.ylabel("Distance in y-axis (m)")
    plt.title("Trajectory of a Sphere and Airfoil on the Moon")
    plt.legend(loc='upper right')
    plt.grid()
    
    # COMPARE EFFECT OF DRAG ON EARTH
    plt.figure()
    plt.plot(x_ball_earth, y_ball_earth, 'b', label="With Drag",)
    plt.plot(x_nodrag_earth, y_nodrag_earth, 'k', label="Without Drag")
    plt.xlabel("Distance in x-axis (m)")
    plt.ylabel("Distance in y-axis (m)")
    plt.title("The Effect of Drag on a Soccer Ball ")
    plt.legend(loc='upper right')
    plt.grid()
    
    plt.show()