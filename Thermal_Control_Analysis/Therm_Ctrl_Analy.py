# -*- coding: utf-8 -*-
"""
Created on Sat Feb  4 23:18:43 2023

@author: yassine Fouchal
"""

import numpy as np
import matplotlib.pyplot as plt

# those are not fixed values, they cann be changed :)
atmosphere_temperature = 90 # Kelvin
ambient_temperature = 85 # Kelvin
surface_area = 2 # m^2
heat_generated = 10 # Watts

# Here I am just defining arrays for variuos values of heat transfer coefficients and heat generated
heat_transfer_coefficient_array = [10, 50, 100, 200, 300, 400] # W/m^2K
heat_generated_array = [5, 10, 20, 50, 100, 200] # Watts

# Define the time interval for the simulation
#time_interval = 14 * 365 * 24 * 60 * 60 # 14 years in seconds

# arrays to store the temperature changes
temp_change_htc = []
temp_change_hg = []
temp_change_sa = []

# Loop over different heat transfer coefficients
for htc in heat_transfer_coefficient_array:
    # Calculate the temperature change based on the heat transfer coefficient
    temp_change = heat_generated / (htc * surface_area)
    temp_change_htc.append(temp_change)

# Loop over different values of heat generated
for hg in heat_generated_array:
    # Calculate the temperature change based on the heat generated
    temp_change = hg / (heat_transfer_coefficient_array[0] * surface_area)
    temp_change_hg.append(temp_change)

# Loop over different surface areas
surface_area_array = [0.5, 1, 2, 5, 10, 20] # m^2
for sa in surface_area_array:
    # Calculate the temperature change based on the surface area
    temp_change = heat_generated / (heat_transfer_coefficient_array[0] * sa)
    temp_change_sa.append(temp_change)

#  temperature changes for each heat transfer coefficient
plt.figure()
plt.plot(heat_transfer_coefficient_array, temp_change_htc, '-', label='Heat Transfer Coefficient')
plt.xlabel('Heat Transfer Coefficient (W/m^2K)')
plt.ylabel('Temperature Change (K)')
plt.legend()
params = {"ytick.color" : "w",
          "xtick.color" : "w",
          "axes.labelcolor" : "w",
          "axes.edgecolor" : "w"}
plt.rcParams.update(params)
plt.savefig('heatTransferCoefficientTemperatureChange.png', transparent=True)
# temperature changes for each heat generated
plt.figure()
plt.plot(heat_generated_array, temp_change_hg, '-', label='Heat Generated')
plt.xlabel('Heat Generated (W)')
plt.ylabel('Temperature Change (K)')
plt.legend()
params = {"ytick.color" : "w",
          "xtick.color" : "w",
          "axes.labelcolor" : "w",
          "axes.edgecolor" : "w"}
plt.rcParams.update(params)
plt.savefig('heatGeneratedTemperatureChange.png', transparent=True)
# temperature changes for each surface area
plt.figure()
plt.plot(surface_area_array, temp_change_sa, '-', label='Surface Area')
plt.xlabel('Surface Area (m^2)')
plt.ylabel('Temperature Change (K)')
plt.legend()
params = {"ytick.color" : "w",
          "xtick.color" : "w",
          "axes.labelcolor" : "w",
          "axes.edgecolor" : "w"}
plt.rcParams.update(params)
plt.savefig('surfaceAreaTemperatureChange.png', transparent=True)
plt.show()
