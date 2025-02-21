# Code to conceptually (with some preliminary) size an aircraft for flight on another planet :) <3 UwU
# TODO list
# 2. Integrate Cristobal's packaging code to get a more accurate estimate for the fuselage length and diameter
# 4. Add VSP files for each config 
# 3. Add in deployment pull out code. Use number of wing folds to indicate (nod) the downsides of wing folding 
# 5. For 2024 VGC team, we need you guys to add a propellor sizing process with xROTOR that sizes the 
#    diameter of the propeller to obtain the best efficiency while meeting thrust requirements
# 6. For 2024 VGC team, if time permits, add parallel processing to make the code run faster :)

import os
ROOT_DIR = os.path.dirname(__file__)

import numpy as np
import pandas as pd
import sys
import matplotlib.pyplot as plt
from timeit import default_timer
from datetime import datetime

sys.path.insert(0, ROOT_DIR + "/Payload")
sys.path.insert(0, ROOT_DIR + "/Dim_Constraint")
sys.path.insert(0, ROOT_DIR + "/Flight_Requirements")
sys.path.insert(0, ROOT_DIR + "/Environment")
sys.path.insert(0, ROOT_DIR + "/Aero")
sys.path.insert(0, ROOT_DIR + "/Propulsion")
sys.path.insert(0, ROOT_DIR + "/Power")
sys.path.insert(0, ROOT_DIR + "/Sizing")
sys.path.insert(0, ROOT_DIR + "/Weight")
sys.path.insert(0, ROOT_DIR + "/Mission_Performance")
sys.path.insert(0, ROOT_DIR + "/VSP_tests")

from Propulsion.Prop_Power_V2 import *
from Propulsion.PropellerDnP import *
from Propulsion.thrustRequired import *
from Weight.TitanWeight import *
from Weight.Structure import *
from Environment.MarsAtm import *
from Aero.AnS import *
from Aero.parasitic_drag_func import *
from Payload.Power_Systems import *
from Flight_Requirements.Mission import *
from Flight_Requirements.Flight_Performance import SegRequirements as ReqIN
from Flight_Requirements.Performance_Metrics import *
from VSP_tests.BWB_sections import *
from TOPSIS import *
from AeroshellSelector import *
from Config_generation import *
from Wing_folding import *
from Velocity_Area_Calc import *
from Case_generator import *

# Read input file from Input.csv
#! WHAT DO WE INPUT? FULL DOE?
input = pd.read_csv(os.path.join(ROOT_DIR, "Inp/Input.csv"))

#TODO Environment
planet = input.Planet[0]
atm_table = planet
planetConstants = Planet_Const(planet)
g0 = planetConstants[0]
DIAMETER = planetConstants[1]

#! CHECK SOME OF THE INPUTS: SENSORWIDTH, TMISSION, ...
# Define variables from Input.csv
ncases = int(input.n_Cases[0])
n_cases = np.linspace(1,ncases,ncases)
W_inp = input.Launch_Veh_Mass_Const[0]
CruiseAlt = input.Cruise_Alt[0]
CruiseV = calculate_required_velocity(input.Sensor_swath_width[0],input.Area_coverage[0],input.tmission[0])
if CruiseV < 12:
    CruiseV = 12
DShell = input.D_shell[0]
MinAlt = input.Min_Alt[0]
RoCminAlt = input.RoC_min_alt[0]
RoCmaxAlt = input.RoC_max_alt[0]
Accel = input.Accel[0]
num_Folds = input.Num_Folds[0]

# Constants
fge = 9.80665  # Earth gravitational constant [m/s^2]
lbf2N = 4.44822  # [lbf/N]
N2lbf = 0.224809  # [N/lbf]
m2ft = 3.28084  # [m/ft]
psf2Pa = 0.020885434273039  # [psf/Pa]

#! THIS WE SHOULD CHECK
custom = 1 # Change this to 1 or 0 if you want to use a custom DoE or not
if custom == 1:
    DOE = pd.read_csv(os.path.join(ROOT_DIR, "Inp/DOE_Custom_8900_Final.csv"))
elif custom == 0:
    DOE = Case_generator(input)

## Design Loop

# Constraints (NEED TO MAKE AN INPUT)
tol = 0.9999999
CLmax = 1.5

# CASE input
CASE_START = 1 # If you want to start from a certain case #
CASE_END = DOE.shape[0]

if CASE_END == "end":
    loop_end = DOE.shape[0]
else:
    loop_end = CASE_END
Len1 = loop_end - (CASE_START - 1)
Sref = DOE.Wing_S.to_numpy()

wingx0 = 0
CGx = 0
rootfoil = DOE.Foil0[0]
tipfoil = DOE.Foil1[0]
S_Vtail = DOE.Ht_S[0] + DOE.Vt_S[0]
Lf = DOE.Fuse_L[0]
df = DOE.Fuse_d[0]
Len2 = 1

# Propulsion Inputs
etap1 = 0.9
etap2 = 0.9

#! CHECK IF SOME OF THESE NEED TO BE CHANGED/REMOVED
# Output Arrays

Out_S = np.zeros((Len1, Len2))  # Surface
Out_b = np.zeros((Len1, Len2))  # Wingspan
Out_P = np.zeros((Len1, Len2))  # 
Out_m = np.zeros((Len1, 16))    # Mass?
Out_v = np.zeros((Len1, Len2))  # Speed (cruise?)
Out_CD = np.zeros((Len1, Len2)) # CD
Out_CL = np.zeros((Len1, Len2)) # CL
Out_AoA = np.zeros((Len1, Len2))    # AoA
# Out_PropN = np.zeros((Len1, Len2))  # Number of props
# Out_PropD = np.zeros((Len1, Len2))  # Diameter of props
# Out_PropBlade = np.zeros((Len1, Len2))  # Something of the propeller blade
Out_Sweep = np.zeros((Len1, Len2))  # Wing sweep
Out_root_C = np.zeros((Len1, Len2)) # Root cord
Out_tip_C = np.zeros((Len1, Len2))  # Tip cord
Out_Wing_TR = np.zeros((Len1, Len2))    # Tapper ratio
Out_Wing_AR = np.zeros((Len1, Len2))    # Aspect ratio
Out_Wing_dhl = np.zeros((Len1, Len2))   # Wing dihedral
Out_twist = np.zeros((Len1, Len2))  # Wing twist
Out_fuse_L = np.zeros((Len1, Len2)) # Fuselage length
Out_fuse_D = np.zeros((Len1,Len2))  # Fuselage diameter
Out_fuse_x_CG = np.zeros((Len1, Len2)) # Fuselage CoG
Out_LD_ratio = np.zeros((Len1, Len2))   # Efficiency
Out_S_Vtail = np.zeros((Len1, Len2))    # V tail surface
Out_S_htail = np.zeros((Len1, Len2))    # H tail surface
Out_S_vtail = np.zeros((Len1, Len2))    # Vertical tail surface
Out_SM = np.zeros((Len1, Len2)) # Static margin
Out_cgx = np.zeros((Len1, Len2))    # CoG
Out_RTG_h = np.zeros((Len1, Len2))  #! needs change for rocket
Out_nRTG = np.zeros((Len1, Len2))   #! needs change for rocket
Out_Bat_h = np.zeros((Len1, Len2))  #* Battery stuff, do we need?
Out_check = np.zeros((Len1, Len2), dtype=object)    # Error check
Out_tail_check = np.zeros((Len1, Len2), dtype=object) # Tail error check
Out_lb = np.zeros((Len1, Len2)) # Boom length/Wing span for boom config
Out_Planet = np.zeros((Len1, Len2), dtype=object)   # Planet
Out_Limiting = np.array(np.zeros((Len1, Len2)), dtype=object)   # Power limitation
Out_Area_cov = np.zeros((Len1, Len2))   #* Sensor area cov
Out_Case = np.zeros((Len1, Len2))   # Case
Out_Tail_type = np.zeros((Len1, Len2), dtype=object)    # Tail type
Out_Winglet = np.zeros((Len1, Len2), dtype=object)  # Winglet
Out_Wing_fold = np.zeros((Len1, Len2), dtype=object)    # Wing folding
Out_Num_wing_fold = np.zeros((Len1, Len2))  # Number of wing folds
Out_Season = np.zeros((Len1, Len2), dtype=object)   # Season
Out_mission_time = np.zeros((Len1, Len2))   # Mission
Out_swath_width = np.zeros((Len1, Len2))    # Sensor width
Out_aeroshell = np.zeros((Len1,Len2))   # Aeroshel diameter
loop = 0
j = 0

#! SEE WHAT NEEDS TO BE CHANGED, KEEP THE REST THE SAME
start1 = default_timer()
for loop in range(CASE_START - 1, loop_end):    
    start = default_timer()
    I = 0
    print(f"Case {loop + 1} of {loop_end}")
    CASE = DOE.iloc[[loop]]
    CASE.index = [0]

    max_alt = CASE.Cruise_Alt[0]
    min_alt = CASE.Min_Alt[0]
    cruise_alt = (
        CASE.Cruise_Alt[0]
        if "Cruise_Alt" in CASE.columns
        else 0.5 * (max_alt + min_alt)
    )
    cruise_v = CASE.Cruise_V[0]
    v = CASE.v[0] if "v" in CASE.columns else cruise_v
    if custom == 1:
        #! This is a function of sensors, need to check
        v = calculate_required_velocity(CASE.Sensor_swath_width[0],CASE.Area_coverage[0],CASE.tmission[0])
        CASE.loc[0, 'Cruise_V'] = v
    # if v < 12.0:
    #     v = 12.0
    #     CASE.loc[0, 'Cruise_V'] = v
        #! This needs to be changed by me an Louis
    if CASE.Season[0] == 'summer': # Can choose just one season b/c Titan seasons last several earth years and mission time is usually alwasy <= earth 1 yr
        v_wind = Wind_Summer(0, (ATM_Table(cruise_alt, planet)[1]/100)) # Currently only have wind data for Titan
    if CASE.Season[0]== 'fall':
        v_wind = Wind_Fall(0, (ATM_Table(cruise_alt, planet)[1]/100)) # latitude hard coded to 0 for north pole of Titan but should make it an input
    if CASE.Season[0]== 'spring':
        v_wind = Wind_Spring(0, (ATM_Table(cruise_alt, planet)[1]/100))
    if CASE.Season[0]== 'winter':
        v_wind = Wind_Winter(0, (ATM_Table(cruise_alt, planet)[1]/100))
    if abs(v_wind) > v:
        v = 1.15*abs(v_wind)
    min_v = CASE.Min_V[0] if "Min_V" in CASE.columns else cruise_v
    max_v = CASE.Max_V[0] if "Max_V" in CASE.columns else cruise_v
    roc_min_alt = CASE.RoC_min_alt[0]
    roc_max_alt = CASE.RoC_max_alt[0]
    accel = CASE.Accel[0]

    if not ("v" in CASE.columns):
        CASE = pd.concat([CASE, pd.DataFrame({"v": [v]})], axis=1)

    alt_vec = np.linspace(min_alt, max_alt, 3)
    rho_vec = np.zeros(len(alt_vec))
    mu_vec = np.zeros(len(alt_vec))
    a_vec = np.zeros(len(alt_vec))
    T_vec = np.zeros(len(alt_vec))
    for i, h in enumerate(alt_vec):
        a = ATM_Table(h, atm_table)
        rho_vec[i] = a[0]
        mu_vec[i] = a[5]
        a_vec[i] = a[3]
        T_vec[i] = a[2]

    rho_low = rho_vec[0]
    rho_cruise = ATM_Table(cruise_alt, atm_table)[0]
    rho_high = rho_vec[2]
    mu = mu_vec[0]
    sos = a_vec[0]

    T_pay = Temp_Payload()
    CASE = pd.concat(
        [
            CASE,
            pd.DataFrame(
                {"Tp_min": [T_pay[0]], "Tp_max": [T_pay[1]], "Tp_mid": [T_pay[2]]}
            ),
        ], 
        axis=1,
    )

    CASE = pd.concat(
        [
            CASE,
            pd.DataFrame(
                {
                    "a_min": [min(a_vec)],
                    "g": [g0],
                    "P": [0],
                    "PHeat": [0],
                    "q": [0],
                    "Tmin": [np.min(T_vec)],
                    "Tmax": [np.max(T_vec)],
                }
            ),
        ],
        axis=1,
    )
    root = Airfoil_Properties(DOE.Foil0[loop], DOE.NACA0[loop])
    tip = Airfoil_Properties(DOE.Foil1[loop], DOE.NACA1[loop])
    CASE = pd.concat(
        [
            CASE,
            pd.DataFrame(
                {
                    "Wing_r_tc": [root[0]],
                    "Wing_t_tc": [tip[0]],
                    "Wing_r_xtc": [root[1]],
                    "Wing_t_xtc": [tip[1]],
                    "Wing_r_cam": [root[2]],
                    "Wing_t_cam": [tip[2]],
                    "Wing_r_xc": [root[3]],
                    "Wing_t_xc": [tip[3]],
                }
            ),
        ],
        axis=1,
    )
    CASE = CASE.astype('object')
    if "Wing_TC" in CASE.columns:
        CASE.at[0, "Wing_TC"] = (root[0] + tip[0]) / 2
    else:
        CASE = pd.concat(
            [CASE, pd.DataFrame({"Wing_TC": [(root[0] + tip[0]) / 2]})], axis=1
        )

    AR = CASE.Wing_AR[0]
    S = CASE.Wing_S[0]
    Static_M = 0.15
    b = np.sqrt(AR * S)
    k = 1 / np.pi / 0.85 / (AR)
    P0 = 0.5
    convergence = 1
    count = 1
    struc = 0
    counter = 0
    z = 1 # Learining parameter
    Num_Folds = 0
    fold_loc = 0
    while convergence > tol:
        print('Tail type:',DOE.Tail_type[loop])
        print('Wing Aspect Ratio: {:.2f}'.format(AR))
        print('Wingspan: {:.2f} m'.format(b))
        print('Flying in the Martian',CASE.Season[0])
        #! This will change
        print('Number of propulsors: {:.2f}'.format(CASE.PropulsionN[0]))
        D_shell = CASE.D_shell[0]
        print('Aeroshell Diameter Constraint: {:.2f} m'.format(D_shell))
        if DOE.Wing_folding[loop] == 'yes':
            print('Wing folding allowed. Current number of folds per wing:',Num_Folds/2)
        else:
            print('Wing folding not allowed')

        if CASE.Tail_type[0] == 'No_tail':
            CASE.at[0, 'Winglet'] = 'no'

        if CASE.Winglet[0] == 'yes':
            print('Winglets being used')
        q = q_calc(rho_cruise, v)
        CASE.at[0, "q"] = q
        
        # Weight
        if not (hasattr(struc, "__len__")):
            CASE.at[0, "EQ_W_Wing"] = 0
        else:
            CASE.at[0, "EQ_W_Wing"] = DOE.iloc[loop].EQ_W_Wing
        #! This here also (need to change when we change weights)
        CASE.at[0, "PropD"] = calculate_propeller_diameter(CASE)
        m, fuse_dim, cg_set, Pinfo, Check = Weights(CASE, struc, Num_Folds, fold_loc)
        print('Fuselage length: {:.2f} m'.format(fuse_dim['Fuse_L']))
        print('Fuselage diameter: {:.2f} m'.format(fuse_dim['Fuse_d']))
        
        if fuse_dim['Fuse_L'] > 0.9*CASE.D_shell[0]:
            print()
            print('Failure: Fuselage length greater than aeroshell diameter')
            Check = 'Fuselage greater than aeroshell diameter'
            break

        if hasattr(fuse_dim, "__len__") and CASE.Type[0] == "Wing":
            CASE.at[0, "Fuse_L"] = fuse_dim["Fuse_L"]
        CASE.at[0, "P"] = Pinfo[0]
        CASE.at[0, "PHeat"] = Pinfo[1]
        mtot = m.to_numpy()[0][0]

        # Aero
        mach = np.divide(v, sos)

        [dp, S_Vtail, Vtailpnt, SM, npt, struc, Yaw_check, Pitch_check,b_tail_check,S_tail_check, cd0] = AnS(
            CASE, mach, rho_cruise, mu, v, g0, CGx, D_shell, input.lb[0], 
            DOE.Tail_type[loop], DOE.Winglet[loop],fuse_dim['Fuse_L'],fuse_dim['Fuse_d']
        )
        tail_check = 1
        if b_tail_check == 1:
            tail_check = 'Tail span too big'

        ##WE SHOULD CHANGE THIS TO CALCULATE THE MAXIMUM ML/D, WE CARE ABOUT
        ##RANGE PRIMARILY
        # Calculate the airspeed required to achieve the maximum L/D
        if CASE.Wing_Sweep[0] == 0:
            e = 1.75*(1 - 0.045*AR**0.68)-0.64
        elif 0 < CASE.Wing_Sweep[0] < 30:
            etheo1 = 1.75*(1 - 0.045*AR**0.68)-0.64
            etheo2 = 4.61 * (1 - 0.045 * AR ** 0.68) * (np.cos(CASE.Wing_Sweep[0] * (math.pi/180))) ** 0.15 - 3.1
            e = (etheo1 + etheo2)/2
        else:
            e = 4.61 * (1 - 0.045 * AR ** 0.68) * (np.cos(CASE.Wing_Sweep[0] * (math.pi/180))) ** 0.15 - 3.1
        k = 1/(math.pi*e*AR)
        v = np.sqrt((2/rho_cruise)*((mtot*g0)/S)*(k/cd0))
        v = v.item()
        CASE.v = v
        print('Adjusting airspeed to achieve (L/D)max: {:.2f} m/s'.format(v))

        # Determine angle of attack needed to achieve L/D max
        cl = (mtot*g0)/((0.5*rho_cruise*v**2)*S)
        fa = interp1d(dp["CL"], dp["AOA"], fill_value="extrapolate")
        aoa_LDmax = fa(cl)
        AoA = aoa_LDmax[0]
        print('To achieve (L/D)max, the aircraft needs to fly at an AoA = {:.2f} degrees'.format(AoA))
        if round(AoA) < -5 or round(AoA) > 19:
            print('Failure: Required AoA out of range')
            Check = 'AoA out of range'
            AoA = 2.2 # dummy value to keep the loop going
        if AoA < -1:
            CASE.loc[0, 'Foil0'] = 'n2415'
            CASE.loc[0, 'Foil1'] = 'n2415'
            print('Negative angle of attack needed to achieve (L/D)max, changing airfoil to NACA 2415 to achieve lower lift coefficient')

        # Determine if the required angle of attack will stall the aircraft
        sweep_aoa_array = [15, 16, 17, 18, 19, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]
        cl_index = [index for index, value in enumerate(sweep_aoa_array) if value == int(round(AoA))] # Finds what index the aoa for LDmax is 
        cl_aoa_LDmax = dp['cl'][cl_index[0]]
        cl_aoa_LDmax_avg = sum(cl_aoa_LDmax)/len(cl_aoa_LDmax)

        # Calculate static margin at new AoA
        SM = -dp['Cma'][cl_index[0]]/dp['CLa'][cl_index[0]]
        
        # If the AoA will stall the aircraft, then increase wingspan, cruise speed, or both in order to meet lift requirements
        if cl_aoa_LDmax_avg > 1.42:
            print('AoA needed to fly at (L/D)max may cause the aircraft to stall. The wingspan and cruise speed will be adjusted to meet lift requirements')
            cl = []
            cl.append(dp['CL'][12])# Use CL at a 2 degree angle of attack
            AoA = 2
            cl_index[0] = 2
            SM = -dp['Cma'][12]/dp['CLa'][12]
            L_generated = (0.5*rho_cruise*v**2)*S*cl[0]
            if L_generated < mtot*g0 and DOE.Wing_folding[loop] == 'no': # If wing folding isn't allowed
                b = 0.95*CASE.D_shell[0]
                CASE.b = b
                print('Lift coefficient too low, adjusting wingspan to 95 percent of the aeroshell diameter: {:.2f}'.format(b))
                S = (b**2)/AR
                CASE.S = S
                print('New wing area: {:.2f} m^2'.format(S))
                v = np.sqrt((2*mtot*g0)/(rho_cruise*S*cl[0]))
                v = v.item() # Converts v to float
                CASE.v = v
                print('Adjusting cruise speed to {:.2f} m/s to meet lift requirements'.format(v))
            if L_generated < mtot*g0 and DOE.Wing_folding[loop] == 'yes': # If wing folding is allowed
                S = (mtot*g0)/((0.5*rho_cruise*v**2)*cl[0])
                S = S.item()
                CASE.S = S
                b = np.sqrt(AR*S)
                CASE.b = b
                print('Lift coefficient too low, adjusting wingspan to {:.2f} m to meet lift requirements'.format(b))
                print('New wing area: {:.2f} m^2'.format(S))

        # Calculate new mission time to achieve target area coverage
        tmission = calculate_mission_t(CASE.Sensor_swath_width[0],CASE.Area_coverage[0],v)
        CASE.loc[0, 'tmission'] = tmission
        print('Mission time adjusted to {:.4f} years due to airspeed change'.format(tmission))

        # Wing folding
        if DOE.Wing_folding[loop] == 'no':
            if b > CASE.D_shell[0]:
                print()
                print('Failure: Required wingspan is greater than aeroshell diameter. Breaking out of loop')
                Check = 'Wingspan greater than aeroshell diameter, folding wings needed'
                break
        
        if DOE.Wing_folding[loop] == 'yes':
            [Num_Folds,fold_loc] = num_wing_folds(b, CASE.D_shell[0])
            print('New number of wing folds per wing:',Num_Folds/2)
        
        fcd = interp1d(dp["CL"], dp["CD"], kind = 'quadratic', fill_value="extrapolate")
        print('CL: ',cl[0])
        print('CD: ', fcd(cl)[0])
        print('L/D: ', cl/fcd(cl)[0])
        cl = cl[0]
        cd = fcd(cl)

        if cl < 0 or cd < 0:
            print()
            print('Failure: Negative aerodynamic coefficients, breaking out of loop')
            Check = 'Negative aerodynamic coefficient'
            break
        
        # Structures
        # Calculate the limit load factor due to gusts
        #! This should take care of itself once the wind model is upadted
        rho_sl = ATM_Table(0, atm_table)[0]
        #TODO 
        U = continuous_gust_model(v, abs(v_wind), CASE.tmission[0],planet,cruise_alt)# Finds gust as a percentage of the total wind speed (z-component of wind vector)
        n_gust = 1 + (1/(2*mtot*g0))*rho_sl*(v*np.sqrt(rho_cruise/rho_sl))*U*S*dp['CLa'][cl_index[0]]
        print('Max gust speed in Titanian {:}: {:.2f} m/s'.format(CASE.Season[0],U))
        print('Gust load factor: {:.2f}'.format(n_gust[0]))
        if n_gust > CASE.Nz[0]: # Adjusts the limit load factor to the gust load factor if the gust load factor is greater
            CASE.loc[0, 'Nz'] = n_gust
        
        # Structural Analysis
        station = np.array(struc["station"][0])
        Fshear_data = np.array(struc["ShearF"])
        Mbend_data = np.array(struc["BendM"])
        Fshear = np.zeros(len(station))
        Mbend = np.zeros(len(station))
        for k, st in enumerate(station):
            fF = interp1d(dp["CL"], Fshear_data[:, k], fill_value="extrapolate")
            fM = interp1d(dp["CL"], Mbend_data[:, k], fill_value="extrapolate")
            Fshear[k] = fF(np.isscalar(cl))
            Mbend[k] = fM(np.isscalar(cl))
        struc = pd.DataFrame({"struc_st": station, "struc_F": Fshear, "struc_M": Mbend})

        #! THIS NEEDS TO BE CHANGED TO ROCKET ENGINE
        # Propulsion
        if v > min_v:
            min_v = v
        if v > max_v:
            max_v = v
        inp_low = [rho_low, min_v, roc_min_alt, accel]
        inp_high = [rho_high, max_v, roc_max_alt, 0]
        inp_cruise = [rho_cruise, v, (roc_max_alt + roc_min_alt) / 2, accel / 2]
        # Fitted a linear regression equation to determine the efficiency when using different numbers of propulsors
        etap1 = 0.91 - 0.01*CASE.PropulsionN[0]
        etap2 = etap1
        Preq, PLimiting = Prequired(
            inp_low, inp_cruise, inp_high, cd, S, mtot, g0, etap1, etap2
        )

        ##BEGIN DEVIN EDITS 02/16
        #maximum thrust in any mission segment sizes the engine: 
        Treq, Tlimiting = Trequired(inp_low, inp_cruise, inp_high, cd, S, mtot, g0)
        #integrate over the cruise to determine fuel consumption:
        #need to add some inputs; SANTA was seemingly not concerned with cruise range
        #adding commented framework for the mission analysis here:
        #cruiseIteration = np.linspace(0, cruiseRange, 100)
        #totalCruiseBurn = 0
        #m_active = mtot
        #for step in cruiseIteration:
        #    dt = cruiseRange/100 * 1e3 / v
        #    CL_cruise = m_active * g0 / (0.5 * rho * v**2 * s)
        #    Treq_cruise = cruise_thrust(rho, v, cd, s)
        #    dWdt = rocketPropulsionPerf(Treq_cruise, Isp, g, M)
        #    decrement weight:
        #    m_active = m_active - dWdt / g0 * dt
        #    totalCruiseBurn = totalCruiseBurn + dWdt * dt

        #m_eng = rocketPropulsionPerf(T_req, Isp, g, M)

        # Power
        ##END DEVIN EDITS 02/16


        Ppay = Power_Payload()
        Preq += Ppay
        if "PPay" in CASE.columns:
            CASE.at[0, "PPay"] = Ppay
        else:
            CASE = pd.concat([CASE, pd.DataFrame({"PPay": [Ppay]})], axis=1)
        CASE.at[0, "P"] = Preq / 1000 + CASE.PHeat[0]
        # Sizing & Dimensioning
        cg_UPD = Position_Long_Stable(
            CASE, m, cg_set, Static_M, npt
        )  # Need updated netural point position and replace value of 1 here
        CGx = cg_UPD.iloc[0]["Net"]

        #Fuel tank sizing / mission analysis
        #while cruiseRange < 

        # Testing Convergence
        #! This tests every output variable to achieve convergence on all of them
        #TODO change conversion criteria to fit our propulsion system (e.g. mass)
        print(f"Total Mass [kg]: {mtot}")
        # print(f"Syst. Power: {Ppay}")
        print(f"Preq: {Preq}, Pguess: {P0 * 1000}")
        convergence = abs((Preq - P0 * 1000))
        diff = (Preq - P0 * 1000)      
        z += diff/1000
        #print('Learning Parameter: ',z)
        P0 = z*(Preq - P0 * 1000) / 1000 + P0
        if counter == 0:
            counter += 1
            P0 = (Preq - 1)/1000  
        print('New Power Guess: ',P0*1000)
        count += 1
        Check = 1
        if convergence > 1e5:
            convergence = tol / 2
            # Preq = Preq
            Check = "Unconverged"
        if cl >= CLmax:
            if I > 10:
                convergence = tol / 2
            else:
                I += 1
            # cl = float("nan")
            Check = "CL > CLmax"
        if fuse_dim["Fuse_L"] > D_shell:
            if I > 10:
                convergence = tol / 2
            else:
                I += 1
            # fuse_dim[-1] = float("nan")
            Check = "Fuselage too big"
        if DOE.Wing_folding[loop] == 'no':
            bmax = CASE.D_shell[0]
            if b > bmax:
                if I > 10:
                    convergence = tol / 2
                else:
                    I += 1
                Check = "Span too big"
        if Yaw_check < 1:
            if I > 10:
                convergence = tol / 2
            else:
                I += 1
            # swl = float("nan")
            if Yaw_check == 0:
                Check = "Tail too big"
            elif Yaw_check == -1:
                Check = "Tail sizing failure"
            else:
                Check = "Missin Cnb in results"
        if Pitch_check < 1:
            if I > 10:
                convergence = tol / 2
            else:
                I += 1
            Check = "Missing Cma in results"
        if abs(diff) < 25:
            print('Power Converged')
            break
        if count > 15:
            print('Power did not converge within 15 iterations, Breaking out of loop')
            Check  = "Power Unconverged"
            break
        if S_tail_check == -1:
            print()
            print('Failure: Negative tail area. Breaking out of loop')
            break
        if v/sos > 0.8:
            print()
            print('Failure: Higher than transonic speeds required, Breaking out of loop')
            Check = "Required airspeed too high"
            break
        print()
    duration = default_timer() - start
    print()
    print(f"Case {loop + 1} took {duration} seconds")
    print()
    # Performance Metrics
    # PerfA = aero_perf(
    #     CASE.PassN[0], v, DIAMETER * math.pi, CASE.tmission[0], min_alt, max_alt
    # )
    # PerfG = grnd_perf(v, PerfA, DIAMETER)

    ## Mission Performance
    #!*Assigns values to the initialized outputs at the beggining
    Out_Case[j] = loop + 1
    Out_S[j] = S
    Out_b[j] = b
    Out_Wing_AR[j] = CASE.Wing_AR[0]
    Out_Wing_TR[j] = CASE.Wing_TR[0]
    Out_m[j] = m.iloc[0]
    Out_P[j] = Preq
    Out_v[j] = v
    Out_CD[j] = cd
    Out_CL[j] = cl
    Out_AoA[j] = AoA
    Out_Sweep[j] = CASE.Wing_Sweep[0]
    Out_Wing_dhl[j] = CASE.Wing_dhl[0]
    Out_twist[j] = 0 # Not calculated
    Out_fuse_L[j] = fuse_dim["Fuse_L"]
    Out_fuse_D[j] = fuse_dim["Fuse_d"]
    Out_fuse_x_CG[j] = cg_UPD.Other[0]
    Out_S_Vtail[j] = S_Vtail
    Out_S_htail[j] = CASE.Ht_S[0]
    Out_S_vtail[j] = CASE.Vt_S[0]
    Out_cgx[j] = cg_UPD.Net[0]
    Out_RTG_h[j] = fuse_dim["h_RTG"]
    Out_Bat_h[j] = fuse_dim["h_bat"]
    Out_nRTG[j] = fuse_dim["n_RTG"]
    Out_SM[j] = SM
    Out_LD_ratio[j] = cl/cd
    Out_check[j] = Check
    Out_tail_check[j] = tail_check
    Out_lb[j] = input.lb[0]
    Out_Planet[j] = planet
    Out_Limiting[j] = PLimiting
    Out_Area_cov[j] =  CASE.Area_coverage[0]
   # Out_PropN[j] = CASE.PropulsionN[0]
    # Out_PropD[j] = CASE.PropD[0]
   # Out_PropBlade[j] = CASE.PropBlade[0]
    Out_Tail_type[j] = CASE["Tail_type"]
    Out_Winglet[j] = CASE["Winglet"]
    Out_Wing_fold[j] = CASE["Wing_folding"]
    Out_Num_wing_fold[j] = Num_Folds
    Out_Season[j] = CASE["Season"]
    Out_mission_time[j] = CASE.tmission[0]
    Out_swath_width[j] = CASE.Sensor_swath_width[0]
    Out_aeroshell[j] = CASE.D_shell[0]
    j += 1

    #* Put all in a data frame to use in VSP
    dF = pd.DataFrame(
        data={
            "Case": Out_Case[0:, 0],
            "Planform": Out_S[0:, 0],
            "Span": Out_b[0:, 0],
            "Wing_Sweep": Out_Sweep[0:, 0],
            'Wing_AR':  Out_Wing_AR[0:, 0],
            'Wing_dhl': Out_Wing_dhl[0:, 0],
            'Wing_TR': Out_Wing_TR[0:, 0],
            'Wing_Twist': Out_twist[0:, 0],
            "Mass": Out_m[0:, 0],
            "Power": Out_P[0:, 0],
            "PowerConstraint": Out_Limiting[0:, 0],
            "AreaCoverage": Out_Area_cov[0:, 0],
            "Airspeed": Out_v[0:, 0],
            "CD": Out_CD[0:, 0],
            "CL": Out_CL[0:, 0],
            "L/D": Out_LD_ratio[0:, 0],
            "AoA": Out_AoA[0:, 0],
            "Fuse_L": Out_fuse_L[0:, 0],
            "Fuse_D": Out_fuse_D[0:, 0],
            "Fuse_x": Out_fuse_x_CG[0:, 0],
            "S_VTail": Out_S_Vtail[0:, 0],
            "S_HorTail": Out_S_htail[0:, 0],
            "S_VerTail": Out_S_vtail[0:, 0],
            "SM": Out_SM[0:, 0],
            "cgx": Out_cgx[0:, 0],
            "RTG_h": Out_RTG_h[0:, 0],
            "n_RTG": Out_nRTG[0:, 0],
            "Batt_h": Out_Bat_h[0:, 0],
           # "PropulsionN": Out_PropN[0:, 0],
            #"PropD": Out_PropD[0:, 0],
            # "PropBlade": Out_PropBlade[0:, 0],
            "Check": Out_check[0:, 0],
            "TailCheck": Out_tail_check[0:, 0],
            "Planet": Out_Planet[0:, 0],
            "mTotal": Out_m[0:, 0],
            "mRTG": Out_m[0:, 1],
            "mBattery": Out_m[0:, 2],
            "mPayload": Out_m[0:, 3],
            "mEmpty": Out_m[0:, 4],
            "mWing": Out_m[0:, 5],
            "mFuselage": Out_m[0:, 6],
            "mFC": Out_m[0:, 7],
            "mStabilizer": Out_m[0:, 8],
            "mProp": Out_m[0:, 9],
            "mAvionic": Out_m[0:, 10],
            "mInsulation": Out_m[0:, 11],
            "mRadiator": Out_m[0:, 12],
            "mHeater": Out_m[0:, 13],
            "mSpanels": Out_m[0:,14],
            "mWingFold": Out_m[0:,15],
            "Tail_type": Out_Tail_type[0:, 0],
            "Winglet": Out_Winglet[0:, 0],
            "Wing_folding": Out_Wing_fold[0:, 0],
            "Num_wing_fold": Out_Num_wing_fold[0:, 0],
            "Season": Out_Season[0:, 0],
            "t_mission": Out_mission_time[0:, 0],
            "Sensor_Swath_width": Out_swath_width[0:, 0],
            "Aeroshell_Diameter": Out_aeroshell[0:, 0]
        },
        index=[i for i in range(Out_S.shape[0])],
    )

    # Get current date
    current_date = datetime.now().strftime("%m-%d")

    # Assuming dF is your DataFrame
    dF.to_csv(
        os.path.join(
            ROOT_DIR, "Out/LOG_" + str(CASE_START) + "-" + str(CASE_END) + "_" + str(planet) + "_" + current_date + ".csv"
        )
    )

    # Eliminate rows with failed cases
    TdF = dF
    TdF = TdF[(dF['Check'] == 1) & (TdF['TailCheck'] == 1)].reset_index(drop=True)

    # Create output with eliminated cases
    TdF.to_csv(
        os.path.join(
            ROOT_DIR, "Out/TOPSIS_LOG_" + str(CASE_START) + "-" + str(CASE_END) + "_" + str(planet) + "_" + current_date + ".csv"
        )
    )
total_duration = default_timer() - start1
print('Runtime:',total_duration,'seconds')

dF.to_csv(
        os.path.join(
            ROOT_DIR, "Out/Compiled_" + str(CASE_START) + "-" + str(CASE_END) + "_" + str(planet) + "_" + current_date + ".csv"
        )
    )

TdF.to_csv(
        os.path.join(
            ROOT_DIR, "Out/TOPSIS_Compiled_" + str(CASE_START) + "-" + str(TdF.shape[0]) + "_" + str(planet) + "_" + current_date + ".csv"
        )
    )

# Case selection
if TdF.shape[0] == 1:
    print('Only case', TdF.Case[0], 'met requirements')
elif TdF.empty:
    print('No cases met requirements')
else:
    # Perform TOPSIS
    TdF,Case_num = TOPSIS(input,TdF,TdF.shape[0])
    # Print the case number selected
    print('Case number',TdF.Case[0], 'was selected')

# Find Aeroshell and Launch vehicle 
file_name = ROOT_DIR + "\Aeroshells.csv"  # Replace with the actual path to your CSV file
if TdF.Wing_folding[0] == 'yes':
    min_diameter = 0.97*TdF.Aeroshell_Diameter[0]  # If wing folding is allowed, the folded wingspan will be about 97% the aeroshell diameter
else:
    min_diameter = TdF.Span[0]  # minimum diameter in meters
min_mass = TdF.Mass[0]  # minimum mass in kilograms

aeroshell_data, keys = read_aeroshell_data(file_name)
selected_aeroshell, aeroshell_name_key = find_aeroshell(min_diameter, min_mass, aeroshell_data, keys)

if selected_aeroshell:
    # Using the correct key for accessing aeroshell name
    print(f"Selected Aeroshell: {selected_aeroshell[aeroshell_name_key]}")
    print(f"Launch Vehicle: {selected_aeroshell['Launch Vehicle']}")
    print(f"Mission Cost ($ billion): {selected_aeroshell['Cost ($ billion)']}")
else:
    print("No suitable aeroshell found for the given criteria.")

#* Geometry generation
# Generate a .vsp3 file for the chosen case
print('Generating VSP file...')
if (TdF.Fuse_L[0] > 0.4*TdF.Span[0]):
    generate_Beluga_VSPfile(TdF)
else:
    generate_BWB_VSPfile(TdF)
print('VSP file generation done')