# Contributors: Jacob Webb, Yassine Fouchal
# Last edited: March 29, 2023
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
#####the code is not complete####
# these values are not set on stone 
D = 4.0  # Diameter of vehicle (m)
L = 10.0  # Length of vehicle (m)
h = 10.0  # Convective heat transfer coefficient (W/m^2.K) it can range from 0.5-1000w/m^2.k)
k_alumninum = 170.0  # Thermal conductivity of aluminum (W/m.K)
k_wall = .04  # Thermal conductivity of fiberglass (W/m.K)
#k_insulation = 0.1  # Thermal conductivity of insulation material (W/m.K)
k_insulation = 0.013  # Thermal conductivity of aerogel insulation material (W/m.K)
L_servo = .5 # diameter of servo housing (m)
t_insulation = .00087 # insulation thickness (m)
t_fiberglass = .0625 # fiberglass wall thickness (m)
kwallOvertwall = (k_insulation*k_wall)/(k_insulation*t_insulation+k_wall*t_fiberglass)
T_RTG = 1273  # Temperature of RTG (K)
T_surroundings = 90.0  # Temperature of surroundings (K)
T_amb = 268 # Temperature of operating range minimum (K)
epsilon = 0.05  # Emissivity of aluminum
#epsilon = 0.75  # Emissivity of fiberglass
epsilon_aerogel = .9
sigma = 0.8 * 5.67e-8  # Stefan-Boltzmann constant for aluminum (W/m^2.K^4)
servo_housing_wall_height = 1 #m
A_wall = 4 * math.pi * (D / 2) ** 2 + math.pi * D * (L - D)# Calculating the surface area of the vehicle
A_wall_servo = (4/3)*math.pi * (.5 / 2) ** 2# Calculating the surface area of the FC-servo housing
A_wall_insulation = (4/3)*math.pi * (.5 / 2) ** 2 - (4/3)*math.pi * (.25 / 2) ** 2 # area of insulation for servo
# Generating a range of x and y values for the temperature profile
x_values = np.linspace(0, L, 101)
y_values = np.linspace(-D/2, D/2, 101)
X, Y = np.meshgrid(x_values, y_values)
Q_conv = h * A_wall * (T_RTG - T_surroundings)# Calculating the heat transfer through convection
Q_conv_servo = h * A_wall_servo * (T_surroundings-T_amb)
delta_T = T_RTG - T_surroundings# Calculating the heat transfer through conduction
Q_cond = k_wall * A_wall * delta_T / L
Q_cond_comb = kwallOvertwall*A_wall*(T_surroundings-T_RTG)
Q_cond_servo = k_insulation * A_wall_servo * (T_surroundings-T_amb) / L_servo
Q_rad = epsilon * sigma * A_wall * T_RTG ** 4# Calculating the heat transfer through radiation
Q_rad_servo = epsilon_aerogel * sigma * A_wall_servo * T_amb ** 4 # servo radiation heat transfer
Q_insulation = k_insulation * A_wall * delta_T / L# Calculating the heat transfer through insulation
Q_insulation_servo = k_insulation * A_wall_insulation * (T_surroundings-T_amb)/ t_insulation# Calculating the heat transfer through insulation for servo
Q_net = Q_conv + Q_cond_comb - Q_rad - Q_insulation# Calculating the net heat transfer
Q_needed_for_heaters = Q_cond_servo + Q_conv_servo - Q_rad_servo - Q_insulation_servo
# Calculating the temperature profile
T_values = T_RTG - (h * (D / 2) / k_wall) * (T_RTG - T_surroundings) * np.exp(-h * X / k_wall) + ((Q_insulation / k_insulation) * (L - X) / (D / 2))
# Adjusting the z-axis values to display heat transfer
T_values_heat_transfer = T_values - T_surroundings
z_values = np.zeros_like(T_values_heat_transfer)
z_values.fill(Q_net)
for i in range(len(x_values)):
    for j in range(len(y_values)):
        if X[j, i] >= 3:
            z_values[j, i] = Q_net + Q_rad + Q_conv + Q_insulation + Q_needed_for_heaters + Q_conv_servo + Q_rad_servo + Q_insulation_servo

# 3D plot of the temperature distribution and heat transfer
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(X, Y, T_values_heat_transfer, cmap='coolwarm', linewidth=0, antialiased=False)
ax.plot_surface(X, Y, z_values, cmap='coolwarm', linewidth=0, antialiased=False)
ax.set_xlabel('Position Along Fuselage (m)')
ax.set_ylabel('Radius (m)')
ax.set_zlabel('Heat Transfer (W)')
ax.set_title('Heat Transfer Distribution')
plt.show()

print(f"Convective heat transfer: {Q_conv:.2f} W")
print(f"Conductive heat transfer: {Q_cond:.2f} W")
print(f"Conductive heat transfer from wall and insulation combined: {Q_cond_comb:.2f} W")
print(f"Radiative heat transfer: {Q_rad:.2f} W")
print(f"Insulation heat transfer: {Q_insulation:.2f} W")
print(f"Net heat transfer: {Q_net:.2f} W")
print(f"Net heat transfer for servos on wings: {Q_needed_for_heaters:.2f} W")
#  3D plot of the temperature and heat transfer distribution to include Q_net
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(X, Y, T_values, cmap='coolwarm')
ax.set_xlabel('Position Along Fuselage (m)')
ax.set_ylabel('Radius (m)')
#ax.set_zlabel('Heat transfer (W)')
ax.set_title('Heat Transfer Distribution')
ax.plot([4], [0], [Q_net], color="Orange")
plt.show()
