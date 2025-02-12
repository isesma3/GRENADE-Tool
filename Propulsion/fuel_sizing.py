import numpy as np
import math
import matplotlib.pyplot as plt


def track_fuel(m0, c, t):
    return m0 - c * t  

def size_fuel_tank(fuel_needed, fuel_density=0.81): ##LOX and CH4
    return fuel_needed / fuel_density 

def compute_burn_rate(power, isp, g0=3.7):
    return power / (isp * g0)

reserve_factor=0.1
def mission_fuel_sizing(power_requirements, durations, isp, fuel_density=0.81, reserve_factor=0.1):
    burn_rates = {phase: compute_burn_rate(power_requirements[phase], isp) for phase in power_requirements}
    total_fuel_needed = sum(burn_rates[phase] * durations[phase] for phase in burn_rates)


    reserve_margin = reserve_factor * total_fuel_needed
    initial_fuel_mass = total_fuel_needed + reserve_margin

    fuel_remaining = initial_fuel_mass

    print("\n--- Fuel Consumption Breakdown ---")
    for phase in power_requirements.keys():
        fuel_used = burn_rates[phase] * durations[phase]
        fuel_remaining = track_fuel(fuel_remaining, burn_rates[phase], durations[phase])
        print(f"{phase}: {fuel_used:.2f} kg used, Fuel remaining: {fuel_remaining:.2f} kg")

    fuel_tank_volume = size_fuel_tank(initial_fuel_mass, fuel_density)
    
    print(f"\nTotal Fuel Needed: {total_fuel_needed:.2f} kg")
    print(f"Fuel Reserve (10%): {reserve_margin:.2f} kg")
    print(f"Initial Fuel Mass (With Reserve): {initial_fuel_mass:.2f} kg")
    print(f"Required Fuel Tank Volume: {fuel_tank_volume:.2f} liters\n")
    
    return initial_fuel_mass, fuel_tank_volume


