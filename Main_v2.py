# Reformat using Black Format Extenstion
import os

ROOT_DIR = os.path.dirname(__file__)
import numpy as np
import pandas as pd
import sys
import scipy
import matplotlib.pyplot as plt

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

from Propulsion.Prop_Power_V2 import *
from Weight.TitanWeight import *
from Environment.TitanAtm import *
from Aero.AnS_FlyingWing import *
from Aero.parasitic_drag_func import *
from Payload.Power_Systems import *
from Flight_Requirements.Mission import *

# Constants
ge = 9.80665  # Earth gravitational constant [m/s^2]
lbf2N = 4.44822  # [lbf/N]
N2lbf = 0.224809  # [N/lbf]
m2ft = 3.28084  # [m/ft]
psf2Pa = 0.020885434273039  # [psf/Pa]

## Load inputs

# Payload


# Dimension Constraint


# Flight Perfromance
FR = pd.read_csv("Flight_Requirements/Flight_Requirements.csv")
max_alt = FR.loc[0, "Value"]
min_alt = FR.loc[1, "Value"]
vcruisemax = FR.loc[2, "Value"]
vcruisemin = FR.loc[3, "Value"]
rocmax = FR.loc[4, "Value"]
rocmin = FR.loc[5, "Value"]
accel = FR.loc[6, "Value"]

# Environemnt
a = TitanATM_Table(2000, "recommended")
rho = a[0]
mu = a[8]
g0 = 1.352
sos = a[3]
print(a)
## Design Loop


# Constraints
bmax = 4 * 2
tol = 0.1
CLmax = 1.5

# Aero inputs
Len1 = 4
Len2 = 4
v_array = np.linspace(vcruisemin, vcruisemax, num=Len1)
Sref = np.linspace(1, 5, num=Len2)
tr = 0.5
sweep = 40
dihedral = 0
wingx0 = 0
swl = 0.03
arwl = 3
trwl = 0.25
tcr = 0.15

# Propulsion Inputs
etap1 = 0.8
etap2 = 0.9

# Output Arrays
Out_S = np.zeros((Len1, Len2))
Out_b = np.zeros((Len1, Len2))
Out_P = np.zeros((Len1, Len2))
Out_m = np.zeros((Len1, Len2))
Out_CD = np.zeros((Len1, Len2))
Out_CL = np.zeros((Len1, Len2))
j = 0
for v in v_array:
    i = 0
    for S in Sref:
        AR = 10
        b = np.sqrt(AR * S)
        if b > bmax:
            b = bmax
            AR = b**2 / S
        k = 1 / np.pi / 0.85 / (AR)
        P0 = 0.12
        convergence = 1
        while convergence > tol:
            # Weight
            d = {
                "Batt": [0],
                "Pay": [0],
                "P": [P0],
                "t": [10],
                "g": [1.352],
                "RTGfuelfrac": [0.05],
                "RTGfueltype": [1],
            }
            set = pd.DataFrame(data=d)
            m = Prelim_Weights(set)
            mtot = m.to_numpy()[0][0]

            # Aero
            q = q_calc(rho, v)
            cl = cl_calc(mtot, g0, q, S)

            mach = np.divide(v, sos)
            # [dp,stab,wl,struc] = flywingaero(b,S,tr,mach,rho,v,g0,sweep,dihedral,bwl,swl,trwl,sweepwl)
            [dp, swl, SM, struc] = flywingaero(
                b,
                S,
                tr,
                sweep,
                tcr,
                dihedral,
                swl,
                arwl,
                trwl,
                mach,
                rho,
                mu,
                v,
                g0,
                CGx=0,
            )
            fcd = interp1d(dp["CL"], dp["CD"], fill_value="extrapolate")
            cd = fcd(cl)

            # Propulsion
            Preq = Prequired(rho, v, cd, S, mtot, g0, rocmin, accel, etap1, etap2)
            # Pcruise = cruise_power(rho, v, cd, S, etap1)
            # Paccelerate = accelerate(rho, v, cd, S, mtot, accel, etap2)
            # Pclimb = climb(rho, vcruisemin, cd, S, mtot, g0, rocmin, etap2)
            # Preq = max(Pclimb, Pcruise, Paccelerate)

            # Power
            Ppay = Power_Payload()
            Preq += Ppay
            # Sizing

            # Testing Convergence
            print(f"Total Mass [kg]: {mtot}")
            print(f"Syst. Power: {Ppay}")
            print(f"Preq: {Preq}, Pguess: {P0*1000}")
            convergence = abs(Preq - P0 * 1000)
            P0 = Preq / 1000
            if convergence > 1e6:
                convergence = tol / 2
                Preq = float("nan")
            if cl > CLmax:
                convergence = tol / 2
                Preq = float("nan")

        ## Mission Performance
        Out_S[j][i] = S
        Out_b[j][i] = b
        Out_m[j][i] = mtot
        Out_P[j][i] = Preq
        Out_CD[j][i] = cd
        Out_CL[j][i] = cl
        i += 1
    j += 1

print(Out_P)


df1 = pd.DataFrame(
    data=Out_S[0:, 0:],
    index=[i for i in range(Out_S.shape[0])],
    columns=["f" + str(i) for i in range(Out_S.shape[1])],
)
df2 = pd.DataFrame(
    data=Out_b[0:, 0:],
    index=[i for i in range(Out_b.shape[0])],
    columns=["f" + str(i) for i in range(Out_b.shape[1])],
)
df3 = pd.DataFrame(
    data=Out_m[0:, 0:],
    index=[i for i in range(Out_m.shape[0])],
    columns=["f" + str(i) for i in range(Out_m.shape[1])],
)
df4 = pd.DataFrame(
    data=Out_P[0:, 0:],
    index=[i for i in range(Out_P.shape[0])],
    columns=["f" + str(i) for i in range(Out_P.shape[1])],
)
df5 = pd.DataFrame(
    data=Out_CD[0:, 0:],
    index=[i for i in range(Out_CD.shape[0])],
    columns=["f" + str(i) for i in range(Out_CD.shape[1])],
)
df6 = pd.DataFrame(
    data=Out_CL[0:, 0:],
    index=[i for i in range(Out_CL.shape[0])],
    columns=["f" + str(i) for i in range(Out_CL.shape[1])],
)

df1.to_csv(os.path.join(ROOT_DIR, "Out/S_PlanformArea.csv"))
df2.to_csv(os.path.join(ROOT_DIR, "Out/b_Span.csv"))
df3.to_csv(os.path.join(ROOT_DIR, "Out/m_Mass.csv"))
df4.to_csv(os.path.join(ROOT_DIR, "Out/P_Power.csv"))
df5.to_csv(os.path.join(ROOT_DIR, "Out/CD_DragCoeff.csv"))
df6.to_csv(os.path.join(ROOT_DIR, "Out/CL_LiftCoeff.csv"))

# Plot

fig, ax = plt.subplots()
x = np.divide(Out_m, Out_S)
y = np.divide(Out_P, Out_m)
j = 0
for v in v_array:
    ax.plot(x[j], y[j], linewidth=2.0, marker="o", label=str(v))
    j += 1
plt.legend()
plt.ylabel("Power-to-Mass Ratio, P/m [W/kg]")
plt.xlabel("Wing-Loading, m/S [kg/m^2]")

# fig, ax = plt.subplots()
# x = Out[0]
# y = np.divide(Out[2],Out[1])
# ax.plot(x, y, linewidth=2.0, marker ='o')
# plt.ylabel("Power-to-Mass Ratio, P/m [W/kg]")
# plt.xlabel("S [m^2]")


plt.show()
