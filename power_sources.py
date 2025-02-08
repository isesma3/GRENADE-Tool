import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

################################################################
############USER INPUTS############
planet = "Titan"
################################################################

powers = pd.read_excel("power sources copy.xlsx")
solarphoto_specpwr = powers.iloc[1,1].split('-')
solartherm_specpwr = powers.iloc[1,2].split('-')
rtg_specpwr = powers.iloc[1,3].split('-')
nuclear_react_specpwr = powers.iloc[1,4].split('-')
fuelcell_specpwr = powers.iloc[1,5]

solarphoto_speccost = powers.iloc[2,1].split('-')
solartherm_speccost = powers.iloc[2,2].split('-')
rtg_speccost = powers.iloc[2,3].split('-')
nuclear_react_speccost = powers.iloc[2,4].split('-')
fuelcell_speccost = powers.iloc[2,5].split('-')

#should use solar_photo if earth, rtg if beyond mars
if planet == "Earth" or  planet == "Mars":
    power_source = "solar_photo"
    print("your power source should be", power_source)
    print("your specific power range needs to be between:", solarphoto_specpwr, "W/kg")
    print("your specific cost range needs to be between:", solarphoto_speccost, "$/W")
else:
    power_source = "RTG"
    print("your power source should be", power_source)
    print("your specific power range needs to be between:", rtg_specpwr, "W/kg")
    print("your specific cost range needs to be between:", rtg_speccost, "$/W")








