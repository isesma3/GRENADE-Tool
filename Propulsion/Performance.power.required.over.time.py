# -*- coding: utf-8 -*-
"""
Created on Tue Jan 31 23:49:51 2023

@author: yassine
"""

import numpy as np
import matplotlib.pyplot as plt

def calculate_power(speed, altitude, mass, C_L, C_D, wing_area, power_source):
    rho = 1880 # kg/m^3
    g = 1.325 # m/s^2
    S = wing_area # m^2
    V = speed # m/s
    h = altitude # meters
    m = mass # kg
    power_required = 0.5 * rho * V**3 * S * (C_D + C_L**2 / (np.pi * C_D))
    if power_source == 'MMRTG':
      power_propulsion = 100
    elif power_source == 'ASRG':
      power_propulsion = 200
    else:
      raise ValueError('Invalid power source')
    return power_required, power_propulsion

# Define the various parameters for the airplane
speeds = [50, 100, 200, 300, 400, 500, 600, 800, 900, 1000, 1500, 2000, 2500] # m/s
altitudes = [1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 6000, 6500, 7000, 7500, 8000, 10000] # m
mass = 300 # kg
C_L_array = [0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7]
C_D_array = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
wing_area_array = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100] # m^2
power_sources = ['MMRTG', 'ASRG']

# Calculate power required and propulsion for each set of parameters
years = np.arange(14)
for power_source in power_sources:
    power_required_total = []
    power_propulsion_total = []
    for wing_area in wing_area_array:
        for C_L in C_L_array:
            for C_D in C_D_array:
                for speed in speeds:
                    for altitude in altitudes:
                        power_required, power_propulsion = calculate_power(speed, altitude, mass, C_L, C_D, wing_area, power_source)
                        power_required_total.append(power_required)
                        power_propulsion_total.append(power_propulsion)
    # Plot the results for each power source
    plt.plot(years, power_required_total, label=power_source)

# Add labels and legend to the plot
plt.xlabel('Years')
plt.ylabel('Power Required (W)')
plt.legend()
plt
