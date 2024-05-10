import numpy as np
import matplotlib.pyplot as plt

# Given parameters
r = 0.5  # radius of the sphere (m)
Cd = 0.5  # drag coefficient
rho = 1.293  # air density (kg/m^3)
m = 5  # mass of the sphere (kg)
v0 = 30  # initial velocity (m/s)
theta = np.radians(65)  # launch angle in radians
g = 9.81  # gravity (m/s^2)
dt = 0.01  # time step (s)

# Cross-sectional area of the sphere
A = np.pi * r**2

# Function to calculate acceleration components
def acceleration(vx, vy):
    v = np.sqrt(vx**2 + vy**2)
    ax = -0.5 * Cd * A * rho * v * vx / m
    ay = -g - 0.5 * Cd * A * rho * v * vy / m
    return ax, ay

# Initialize lists to store position coordinates
x_values = [0]  # initial x position
y_values = [0]  # initial y position

# Initialize initial velocity components
vx = v0 * np.cos(theta)
vy = v0 * np.sin(theta)

# Runge-Kutta integration loop
t = 0  # initial time
while y_values[-1] >= 0:
    # Calculate k1 values
    ax1, ay1 = acceleration(vx, vy)
    k1x = vx
    k1y = vy
    
    # Calculate k2 values
    ax2, ay2 = acceleration(vx + ax1 * dt / 2, vy + ay1 * dt / 2)
    k2x = vx + ax1 * dt / 2
    k2y = vy + ay1 * dt / 2
    
    # Calculate k3 values
    ax3, ay3 = acceleration(vx + ax2 * dt / 2, vy + ay2 * dt / 2)
    k3x = vx + ax2 * dt / 2
    k3y = vy + ay2 * dt / 2
    
    # Calculate k4 values
    ax4, ay4 = acceleration(vx + ax3 * dt, vy + ay3 * dt)
    k4x = vx + ax3 * dt
    k4y = vy + ay3 * dt
    
    # Update velocity components
    vx += (dt / 6) * (ax1 + 2 * ax2 + 2 * ax3 + ax4)
    vy += (dt / 6) * (ay1 + 2 * ay2 + 2 * ay3 + ay4)
    
    # Update position
    x_values.append(x_values[-1] + vx * dt)
    y_values.append(y_values[-1] + vy * dt)
    
    # Update time
    t += dt

# Plot the trajectory
plt.plot(x_values, y_values)
plt.title("Trajectory of the sphere (RK4 method)")
plt.xlabel("Horizontal Distance (m)")
plt.ylabel("Vertical Distance (m)")
plt.grid(True)
plt.show()