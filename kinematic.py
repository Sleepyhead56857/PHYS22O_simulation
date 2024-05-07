"""
Author: Kelechi Agommuo, David Olufade, Rajjya Rhan Paudyal
ID Numbers: XM33737, HP56857, SR3095
Created: 9 April 2024



"""
import numpy as np
import matplotlib.pyplot as plt


# Varibles that can be change by the user such as the object mass, enviroment (air resistance ,graviattional force, friction)
Mass = 5
V_Init = 100
Size = 1
RADIUS = 0.5 * Size

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
SPHERE_CS = np.pi * Size**2

SA_BULLET_CONE = np.pi * RADIUS * np.sqrt(Size**2 + RADIUS**2)
SA_BULLET_CYL = 2 * np.pi * RADIUS * Size + np.pi * RADIUS**2
SA_BULLET = SA_BULLET_CONE + SA_BULLET_CYL



# List to hold selected planet information
AIR_DENSITY_LIST = np.array([NULL_DENSITY, EARTH_AIR_DENSITY, MOON_AIR_DENSITY, MARS_AIR_DENSITY, VENUS_AIR_DENSITY, JUPITER_AIR_DENSITY], float) # Putting all gravities from current densities into list
GRAVITY_LIST = np.array([NULL_GRAVITY, EARTH_GRAVITY, MOON_GRAVITY, MARS_GRAVITY, VENUS_GRAVITY, JUPITER_GRAVITY], float) # Putting all gravities from current planets into list
DRAG_LIST = np.array([SPHERE_DRAG_COE,AIRFOIL_DRAG_COE, BULLET_DRAG_COE, FLAT_PLATE_DRAG_COE, PRISM_DRAG_COE ])
SHAPE_LIST = np.array([SPHERE_CS], float)

# The index of the user choices 
user_planet_index = 1
user_shape_index = 0


x_points = []
y_points = []
t_points = []




# need to add function to account for
    #   DRAG
def positionDrag(r, t):

    #change in time
    dt = t - r[6] 

    # compute the velocity
    velo_x = r[4]/Mass
    velo_y = r[5]/Mass


    magnitude_velocity = np.sqrt(velo_x **2 + velo_y**2)

    #calculate the unit vector of the x ad y axis
    unit_vector_x = velo_x / magnitude_velocity
    unit_vector_y = velo_y / magnitude_velocity

    #F_air = -1/2 * p(air density) * Area * drag_Coeffeient * | v |^2 * unit_vector
    F_air = - AIR_DENSITY_LIST[user_planet_index] * SHAPE_LIST[user_shape_index] * DRAG_LIST[user_shape_index] *(magnitude_velocity**2)

    # comute the air resistance for x and y
    F_air_x = F_air * (velo_x/ magnitude_velocity)
    F_air_y = F_air * (velo_y/ magnitude_velocity)

    #calculate the gravity
    F_gravity = Mass * GRAVITY_LIST[user_planet_index]

    F_net_y = F_gravity + F_air_y

    #compute the momentum of the object
    rho_x = r[4] + F_air_x * dt
    rho_y = r[5] + F_net_y * dt

    # compute the position of the ball
    pos_x = r[0] + rho_x * dt / Mass
    pos_y = r[1] + rho_y * dt / Mass

    t += dt

    return np.array([pos_x, pos_y, velo_x, velo_y, rho_x, rho_y, t], float)

    #When Weight == Drag it has reah ter,inal velocty https://www.grc.nasa.gov/www/k-12/VirtualAero/BottleRocket/airplane/termv.html

def rk4(r_init, t, h):
    r = r_init

    k1 = h * positionDrag(r, t)
    k2 = h * positionDrag((r + 0.5 * k1), (t + 0.5 * h))
    k3 = h * positionDrag((r + 0.5 * k2), (t + 0.5 * h))
    k4 = h * positionDrag((r+k3), (t+h))
 
    r = ((k1+ 2*k2 + 2*k3 + k4) /6)
 
    return r

def adapative_size(r_init, a_init, b_init, change = 1):
    #set up
    r = r_init
    h = (b_init - a_init) / 1
    t = a_init

    x_points.append(r[0])
    y_points.append(r[1])
    t_points.append(t)

    rho = 1

    while (r[1] >= 0):

        if rho < 2:
            h *= rho**(1/4)
        else:
            h *= 2

   
        #calclute rho
        r_half = r + rk4(r, t, h)
        r_half += rk4(r_half, t+h, h)
        r_full = r + rk4(r, t, 2*h)
        
        R_cmp = r_half - r_full
        error = np.sqrt(R_cmp[0]**2 + R_cmp[1]**2)

        if (error == 0):
            rho = 3
        else:
            rho = 30 * h * change/ np.sqrt(R_cmp[0]**2 + R_cmp[1]**2)
        
        
        if rho > 1:
            t += 2*h
            r = r_half
            x_points.append(r[0])
            y_points.append(r[1])
            t_points.append(t)
        
def calcMaxHeightTime(theta):
    #https://phys.libretexts.org/Bookshelves/University_Physics/Physics_(Boundless)/3%3A_Two-Dimensional_Kinematics/3.3%3A_Projectile_Motion#:~:text=The%20range%20of%20an%20object,2%CE%B8i2g.
    time = (V_Init * np.sin(theta)) /GRAVITY_LIST[user_planet_index] 

    return time


if __name__ == '__main__':

    theta = 30 * np.pi / 180
    x_init = 0
    y_init = 0
    velo_x_init = 0
    velo_y_init = 0
    rho_x_init = 0
    rho_y_init = 0
    MaxTime = 0
    accuracy = .01


    """
    # DEVELOPED USER INTERACTION TO CHANGE THE VARAIBLE
    # User Input
    user_inp = input("What planet would you like to select? (1-5)")

    # Selecting planet info to append into list based on user choice
    selected_planet.append(all_densities[int(user_inp) - 1])
    selected_planet.append(all_gravities[int(user_inp) - 1])

    MASS = float(input("Enter the object's mass (in kg): "))
    V_INIT = float(input("Enter the initial velocity (in m/s): "))
    

    # Get the y position that user wants 
    y_init = input("What height do you wan to start the ball?")

    while(y_init.isnumeric() == False or y_init < 0):
        print("Please enter a number equal or grater than 0")
        y_init  = input("What height do you wan to start the ball?")

    
    theta = input("What angle do you want to lauch te object")
    """

    #ininital momentum
    rho_x_init = Mass * V_Init * np.cos(theta)
    rho_y_init = Mass * V_Init * np.sin(theta)

    #inital velocity 
    velo_x_init = rho_x_init / Mass
    velo_y_init = rho_y_init / Mass

    #get the max time that 
    MaxTime = calcMaxHeightTime(theta)

    r = np.array([0, y_init, velo_x_init, velo_y_init, rho_x_init, rho_y_init, 0], float64)
    a = 0
    b = MaxTime

    adapative_size(r, a, b, change=accuracy)



    # PLOT THE GIVEN INFORMATION
    plt.plot(t_points, y_points, 'ro')
    plt.show()
    # SOME HOW GET THE WEBSITE THE INFORMATION
