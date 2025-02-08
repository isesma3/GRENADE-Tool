# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# File Name: TitanMission.py                                 
# Creat Date: January 3, 2023                        
# Contributors (In-Order): Daniel J. Moore                        
# Last Edited: January 3, 2023                    
# Last Edited By: Daniel J. Moore                                
# For: Aerospace Systems Design Lab @ Georgia Tech     
# Description: 
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Functions:
#   Power2Weight()                        
#   Stall_WS():
#
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Version Sumamry
#   1 - Initial Commit (Current Version)
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Additional Notes
#
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Import Libraries
import os
LOCAL_DIR = os.path.dirname(__file__)
ROOT_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))

import pandas as pd
from scipy.interpolate import interp1d
import numpy as np
import math

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

def Power2Weight(CASE,a,rho,v,cd0,mi,S,R,K,dhdt,Rc,g,dvdt):
    '''
    Inputs:
        CASE    - Designation of the flight condition for evaluation
        a       - Ratio motor thrust at altitude to "sea-level"
        h       - Altitude
        rho     - Density
        v       - Airspeed
        cd0     - Parasitic Drag
        mi      - Initial mass
        S       - Wing Area
        R       - Residual drag from other sources
        K       - [K1,K2], where K1 is CL^2 coefficient and K2 is CL coefficient for assumption of parabolic drag model (CD = CD0 + K2CL + K1CL^2)
        dhdt    - Rate of Climb
        Rc      - Turn Radius
        g       - Gravitational accleration
        dvdt    - Acceleration


    Ouptuts:
        P2m -   Power Loading

    Flight Conditions:
        1 - Straight and Level Flight (constant altitude & speed cruise)
        2 - Constant Speed Climb
        3 - Constant Altitude & Speed Turn
        4 - Horizontal Acceleration
        5 - Absolute Ceiling

    '''

    # Assumed Constants
    b = 1   # Beta value (fraction of initial weight)
    a = 1
    K1 = K[0]
    K2 = K[1]

    # General Calculations
    q = 0.5*np.multiply(rho,np.square(v))
    if CASE == 1:
        # Const. Alt. & Speed Cruise
        ePs = 0 #Excess Specific Power
        R = 0   #Assumed no extra resistance
        n = 1
    elif CASE == 2:
        # Const. Speed Climb
        ePs = dhdt
        R = 0
        n = 1
    elif CASE == 3:
        # Const. Alt. & Speed Turn
        ePs = 0
        R = 0
        n = np.sqrt(1+np.divide(np.square(v),np.multiply(g,Rc)))
    elif CASE == 4:
        # Horiz. Accel
        ePs = v*dvdt
        R = 0
        n = 1
    elif CASE == 5:
        # Service Ceiling
        ePs = 0
        R = 0
        n = 1

    
    C0 = np.divide(b,a)
    C1 = np.multiply(np.multiply(C0,v),np.multiply(n,K1))
    C2 = np.multiply(np.multiply(C0,v),np.multiply(n,K2))
    C3 = np.multiply(np.divide(v,a),np.multiply(q,cd0) + np.divide(R,S))
    C4 = C0
    WS = np.divide(mi*g,S)

    P2W = np.multiply(C1,WS) + C2 + np.divide(C3,WS) + np.multiply(C4,ePs)
    P2m = np.divide(P2W,g)


    return P2m

def Stall_m_S(rho,v,mi,g,S,CLmax):
    '''
    Inputs:
        CASE    - Designation of the flight condition for evaluation
        h       - Altitude
        rho     - Density
        v       - Airspeed
        g       - Gravitational Constant
        CLmax   - Maximum Lift Coefficient


    Ouptuts:
        WS -   Wing-Loading

    '''
    mS = 0.5*np.multiply(CLmax,np.multiply(rho,np.square(v)))/g
    return mS

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Testing


#alpha = Power2Weight(1,1,1,2000,0.002377,100,0.05,30,10,0,[1/(math.pi*1*8),0],0)
#print(alpha)
