 # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# File Name: Flight Performance.py                                 
# Creat Date: January 2023                        
# Contributors (In-Order): Alex Kehler, Daniel J. Moore                    
# Last Edited: February 28, 2023                    
# Last Edited By: Daniel Moore                          
# For: Aerospace Systems Design Lab @ Georgia Tech     
# Description: 
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from math import pi as PI
import os
LOCAL_DIR = os.path.dirname(__file__)
ROOT_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))
import sys
sys.path.append(ROOT_DIR)
from Mission import Power2Weight as P2W
from Environment.MarsAtm import ATM_Table as ATM


df = pd.read_csv(os.path.join(LOCAL_DIR,'Flight_Requirements.csv'))
max_alt = df.loc[0, 'Value']
min_alt = df.loc[1, 'Value']
vcruisemax = df.loc[2, 'Value']
vcruisemin = df.loc[3, 'Value']
rocmax = df.loc[4, 'Value']
rocmin = df.loc[5, 'Value']
accel = df.loc[6, 'Value']

def SegRequirements():
    '''
    Inputs:
        NA
    Outputs:
        out - dataframe read of Mission_Profile.csv
    '''
    df = pd.read_csv(os.path.join(ROOT_DIR,'Inp/Mission_Profile.csv'))
    out = df
    return out
def Constraint(m,g0,cd0,Sref,e,AR):
    '''
    Inputs:
        m       - vehicle mass
        g0      - gravitational acceleration
        cd0     - Parasitic Drag Coefficient
        Sref    - planform area
        e       - Oswald's Efficiency
        AR      - Aspect Ratio
    Outputs:
        PW  - array of power-weight/mass ratios (# constraints X len(WS))
        WS  - array of wing-loadings
    Assumptions:
        Residual Drag = 0
        K2 = 0
    '''
    #Import Parameters
    R = 0
    req = SegRequirements() # Read in Mission Profile Parameters
    req_n = req.to_numpy()  # Convert to array for data processing purposes
    Seg = req['Segment'].to_numpy()
    WS = np.divide(m,Sref)  # define wing-loading(s)

    if hasattr(WS,"__len__"):
        PW = np.zeros((req_n.shape[0],len(WS))) #array of thrust2mass for each constraint (rows) and wing-loading (columns)
    else:
        PW = np.zeros((req_n.shape[0],1))
    K = [1/(PI*e*AR),0] # Coefficients for drag polar
    # Adjust m and Sref if only one is an array
    L1 = hasattr(m,"__len__")
    L2 = hasattr(Sref,"__len__")
    if not(L1) and L2:
        m *=np.ones(len(Sref))
    elif not(L2) and L1:
        Sref *= Sref*np.ones(len(m))
    if not(L1) and not(L2):
        L1 = 1
        L2 = 1
        m = [m]
        Sref = [Sref]
    else:
        L1 = len(m)
        L2 = len(Sref)
    for j in range(0,max([L1,L2])): # For each wing-loading
        mi = m[j]
        S = Sref[j]
        for i, Typ in enumerate(req_n[:,1]): # For line in the mission profile
            dhdt = 0
            R = 0
            dvdt = 0
            # Check constraint type
            if Typ == 'Cruise':
                CASE = 1
            elif Typ =='Climb':
                CASE = 2
                dhdt = req_n[i][6]
            elif Typ =='Turn':
                CASE = 3
                R = req_n[i][8]
            elif Typ == 'Acceleration':
                CASE = 4
                dvdt = req_n[i][7]
            else:    # Absolute Ceiling
                CASE = 5
            for idx in range(4,6): # for each altitude
                alt = req_n[i][idx]
                prop = ATM(alt,'')
                a = prop[3]
                rho = prop[0]
                for idx in range(2,4): # for each velocity
                    v = req_n[i][idx]
                    Pm = P2W(CASE,a,rho,v,cd0,mi,S,R,K,dhdt,R,g0,dvdt)
                    if Pm > PW[i][j]:   # for each constraint, only keep most constraining
                        PW[i][j] = Pm
    return PW, WS

###### TEST CODE ######
# Test Constraint Diagram
#PW,WS = Constraint([10,50,100,150,200,250,300,350,400,500,600,700,800,900],1.3,0.05,5,0.9,8)
#print(PW)
#print(WS)
#plt.figure()
#for i in range(0,PW.shape[0]):
#    plt.plot(WS,PW[i])
#plt.show()