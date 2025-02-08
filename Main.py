import os
ROOT_DIR = os.path.dirname(__file__)

import numpy as np
import pandas as pd
import sys
import scipy

sys.path.insert(0, ROOT_DIR+'/Payload')
sys.path.insert(0, ROOT_DIR+'/Dim_Constraint')
sys.path.insert(0, ROOT_DIR+'/Flight_Requirements')
sys.path.insert(0, ROOT_DIR+'/Environment')
sys.path.insert(0, ROOT_DIR+'/Aero')
sys.path.insert(0, ROOT_DIR+'/Propulsion')
sys.path.insert(0, ROOT_DIR+'/Power')
sys.path.insert(0, ROOT_DIR+'/Sizing')
sys.path.insert(0, ROOT_DIR+'/Weight')
sys.path.insert(0, ROOT_DIR+'/Mission_Performance')

from Propulsion.Prop_Power_V1 import *
from Weight.TitanWeight import *
from Environment.TitanAtm import *
from Aero.Aero_V1 import *


# Constants
ge = 9.80665 # Earth gravitational constant [m/s^2]
lbf2N = 4.44822 # [lbf/N]
N2lbf = 0.224809    # [N/lbf]
m2ft = 3.28084  # [m/ft]
psf2Pa = 0.020885434273039  # [psf/Pa]

## Load inputs

# Payload


# Dimension Constraint


# Flight Perfromance
FR = pd.read_csv("Flight_Requirements/Flight_Requirements.csv")
max_alt = FR.loc[0, 'Value']
min_alt = FR.loc[1, 'Value']
vcruisemax = FR.loc[2, 'Value']
vcruisemin = FR.loc[3, 'Value']
rocmax = FR.loc[4, 'Value']
rocmin = FR.loc[5, 'Value']
accel = FR.loc[6, 'Value']

# Environemnt
a = TitanATM_Table(2000, 'recommended')
rho = a[0]
g0 = 1.352
sos = a[3]
print(a)
## Design Loop

# Aero inputs
b = 5
S = 3.125
tr = 1
sweep = 0
dihedral = 0
wingx0 = 0
mach = vcruisemax/sos
# Propulsion Inputs
cd0 = 0.02
k = 1/np.pi/0.8/(b**2/s)
#cl = 0.5



convergence = 1
while convergence > 0.1:
    # Aero
    dp = aero(b,S,tr,sweep,dihedral,wingx0,mach,rho,g0) # {'AOA': alphas, 'CL': CL, 'CD': CD}

    # Weight
    d = {'Batt': [0],'Pay': [100], 'P': [0.12], 't': [10], 'g':[1.352]}
    set = pd.DataFrame(data=d)
    m = Prelim_Weights(set)

    # Propulsion
    q = 0.5*rho*np.square(vcruisemax)
    cl = (m*g0)/(q*S)
    Preq = cruise(rho, vcruisemax, cd0, k, cl, 0.8)

    # Stability (Might be incorperated in Aero)

    # Power

    # Sizing

    # Testing Convergence
    convergence = 0
## Mission Performance
