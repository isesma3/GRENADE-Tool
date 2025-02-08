   # Yassine
                  
                  #sources: https://eaglepubs.erau.edu/introductiontoaerospaceflightvehicles/chapter/reciprocating-engine-propeller/#chapter-493-section-7
                 # https://www.sciencedirect.com/topics/engineering/propeller-diameter
                          #https://www.sciencedirect.com/topics/engineering/propeller-diameter
                          #https://reader.elsevier.com/reader/sd/pii/B978012818465300015X?token=32834D4E5647222F83BA8F779A7C3F08E87085B18E4AB3FA3AE39E922E11361F04B396EE03AD2079AE0920CAB82B2B6C&originRegion=us-east-1&originCreation=20230322211124
                 # three-bladed metal propellers: (BHP stands for Brake Horsepower)
                 # D_p = 18*sqrt(P_BHP) # for three metal-bladed propellers
                 # P_BHP = (thrust*V_stream)/n_p
                 #propeller pitch (P_G) =1251(V_ktas/RPM). This only applies to fixed-pitch propellers rotating at RPM at some desired cruising speed (V_ktas)
import numpy as np
import math
import pandas as pd
from scipy.interpolate import interp1d


#these values can be changed, they are not set on stone. 
#V_stream=100 
#thrust=1000 
#n_p=0.85
#rho=1.225
#V_ktas=100
#area_disc=0.5

def calculate_propeller_diameter(Data):
        '''
    Inputs:
        Sw  - Wetted Area [m^2]
        AR  - Aspect Ratio
        SWP - Sweep Angle [deg]
        q   - Dynamic Pressure [Pa]
        tpr - Taper Ratio
        tc  - Thickness-Chord Ratio
        Nz  - Ultimate Load Factor (1.5x Limit Load)
        mdg - Flight Design Gross Mass [kg]

    Outputs:
        mwing - Wing Mass [kg]

    Assumptions
        - Mlim ~ 0.85
        - RPM ~ 2700 rev/min
    '''
        
    P_BHP = Data.P[0]           # Brake Power [kW]
    a = Data.a_min[0]           # Speed of Sound [m/s]
    v_stream = Data.v[0]        # Free-stream velocity [m/s]
    nBlade = Data.PropBlade[0]  # Number of blades on propeller

    Mlim = 0.85
    RPM = 2700
    if nBlade == 2:
        K = 0.56
    elif nBlade == 3:
        K = 0.52
    else:
        K = 0.49

    D1 = np.sqrt((np.square(a*Mlim) - np.square(v_stream))/(math.pi*RPM))
    D2 = K*np.sqrt(P_BHP)
    return min([D1,D2])

def calculate_propeller_pitch(Data):
    propeller_pitch = 1251*(V_ktas/RPM) #needs to be converted
    return propeller_pitch



    


