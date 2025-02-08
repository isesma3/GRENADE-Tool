# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# File Name: TitanWeight.py                                 
# Creat Date: January 3, 2023                        
# Contributors (In-Order): Daniel J. Moore, Jacob Webb                      
# Last Edited: January 23, 2023                    
# Last Edited By: Daniel Moore                            
# For: Aerospace Systems Design Lab @ Georgia Tech     
# Description: 
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Functions:                                          
#   Power_Payload()
#   Temp_Payload()
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Version Sumamry
#   1 - Initial Commit (Current Version)
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Additional Notes
#
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


# Function Definition
def Temp_Payload():
    '''
    Inputs:
        If called, reads in "included" payload and sensors power from ./Payload/Payload_and_Sensors.csv
    Outputs:
        Tmin - the maximum of the minimum temperature required for payload systems
        Tmax - the minimum of the maximum temperature required for payload systems
        Tmid - mean of Tmin and Tmax

    Notes:
    - Will not read in "Battery" or "RTG" components
    - Components require "TRUE" in the Include col. to be included
    
    '''
    df = pd.read_csv(os.path.join(ROOT_DIR,'Payload','Payload_and_Sensors.csv'))
    Nnan = df['Component'].isna().sum()
    Comp = df['Component'].to_numpy()
    Tmn = df['TempMin'].to_numpy()
    Tmx = df['TempMax'].to_numpy()
    In = df['Include'].to_numpy()
    Tmin = np.max(Tmn[np.argwhere(np.isfinite(Tmx))])
    Tmax = np.min(Tmx[np.argwhere(np.isfinite(Tmx))])
    Tmid = np.mean([Tmax,Tmin])
    Len = Comp.size
    idx = []
    i = 0
    while i < (Len-Nnan):
        if Comp[i] != 'RTG' and Comp[i] != 'Battery' and In[i] == True:
            idx.append(i)
        i += 1
    return [Tmin,Tmax,Tmid]

def Power_Payload():
    '''
    Inputs:
        If called, reads in "included" payload and sensors power from ./Payload/Payload_and_Sensors.csv
    Outputs:
        Ppay - Payload Power [W]

    Notes:
    - Will not read in "Battery" or "RTG" components
    - Components require "TRUE" in the Include col. to be included in the summation
    - Assumes maxPower used
    
    '''
    df = pd.read_csv(os.path.join(ROOT_DIR,'Payload/Payload_and_Sensors.csv'))
    Ppay = 0
    Nnan = df['Component'].isna().sum()
    Comp = df['Component'].to_numpy()
    Pmx = df['maxPower'].to_numpy()
    CNT = df['Count'].to_numpy()
    In = df['Include'].to_numpy()
    Len = Comp.size
    i=0
    while i < (Len-Nnan):
        if Comp[i] != 'RTG' and Comp[i] != 'Battery' and In[i] == True:
            Ppay += Pmx[i]*CNT[i]
        i += 1
    return Ppay