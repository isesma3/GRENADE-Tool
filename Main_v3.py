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
from Weight.Structure import *
from Environment.TitanAtm import *
from Aero.AnS_FlyingWing import *
from Aero.parasitic_drag_func import *
from Payload.Power_Systems import *
from Flight_Requirements.Mission import *
from Flight_Requirements.Flight_Performance import SegRequirements as ReqIN

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
MP = ReqIN()
MP_n = MP.to_numpy()
DOE = pd.read_csv(os.path.join(ROOT_DIR, "Inp/DOE.csv"))
range_alt = [np.min(MP_n[:, 4:6]), np.max(MP_n[:, 4:6])]
range_v = [np.min(MP_n[:, 2:4]), np.max(MP_n[:, 2:4])]
range_roc = [np.min(MP_n[:, 6]), np.max(MP_n[:, 6])]
range_accel = [np.min(MP_n[:, 7]), np.max(MP_n[:, 7])]

# Environemnt
# g0 = Titan_Const()[0]
g0 = Earth_Const()[0]
alt_vec = np.linspace(range_alt[0], range_alt[1], 3)
rho_vec = np.zeros(len(alt_vec))
mu_vec = np.zeros(len(alt_vec))
a_vec = np.zeros(len(alt_vec))
T_vec = np.zeros(len(alt_vec))
for i, h in enumerate(alt_vec):
    a = TitanATM_Table(h, "Earth")
    rho_vec[i] = a[0]
    mu_vec[i] = a[5]
    a_vec[i] = a[3]
    T_vec[i] = a[2]


max_alt = range_alt[1]
min_alt = range_alt[0]
vcruisemax = range_v[1]
vcruisemin = range_v[0]
rocmax = range_roc[1]
rocmin = range_roc[0]
accel = np.average(range_accel)
rho = rho_vec[0]
mu = mu_vec[0]
sos = a_vec[0]

print(a)
## Design Loop


# Constraints
bmax = 4 * 2
Static_M = 0.07
tol = 0.1
CLmax = 1.5

# Aero inputs
loop = 0
CASE = DOE.iloc[[loop]]

CASE = pd.concat(
    [
        CASE,
        pd.DataFrame(
            {
                "g": [g0],
                "v": [vcruisemax],
                "P": [0],
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

Len1 = 4
v_array = np.linspace(vcruisemin, vcruisemax, num=Len1)
Sref = DOE.Wing_S.to_numpy()

wingx0 = 0
rootfoil = DOE.Foil0[0]
tipfoil = DOE.Foil1[0]
swl = DOE.Wing_fin_S[0]
arwl = DOE.Wing_fin_AR[0]
trwl = DOE.Wing_fin_TR[0]
tcr = DOE.Wing_TC[0]
Lf = DOE.Fuse_L[0]
df = DOE.Fuse_d[0]

Len2 = len(Sref)
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
Out_fuse_L = np.zeros((Len1, Len2))
Out_fuse_x = np.zeros((Len1, Len2))
Out_fin_S = np.zeros((Len1, Len2))
Out_cgx = np.zeros((Len1, Len2))
Out_RTG_h = np.zeros((Len1, Len2))
j = 0
for v in v_array:
    i = 0
    for S in Sref:
        CASE.at[0, "v"] = v
        AR = DOE.Wing_AR[loop]
        b = np.sqrt(AR * S)
        if b > bmax:
            b = bmax
            AR = b**2 / S
        k = 1 / np.pi / 0.85 / (AR)
        P0 = 0.12
        convergence = 1
        struc = 0
        while convergence > tol:
            q = q_calc(rho, v)
            CASE.at[0, "q"] = q
            # Weight
            if not (hasattr(struc, "__len__")):
                CASE.at[0, "EQ_W_Wing"] = 0
            else:
                CASE.at[0, "EQ_W_Wing"] = DOE.iloc[loop].EQ_W_Wing

            m, fuse_dim, cg_set = Weights(CASE, struc)
            if hasattr(fuse_dim, "__len__") and CASE.Type[0] == "Wing":
                CASE.at[0, "Fuse_L"] = fuse_dim[-1]
            mtot = m.to_numpy()[0][0]

            # Aero
            cl = cl_calc(mtot, g0, q, S)
            if cl > 1.5:
                cl = 1.5

            mach = np.divide(v, sos)
            # [dp,stab,wl,struc] = flywingaero(b,S,tr,mach,rho,v,g0,sweep,dihedral,bwl,swl,trwl,sweepwl)
            [dp, swl, wingletpnt, SM, npt, struc] = flywingaero(
                CASE, mach, rho, mu, v, g0, CGx=0, S_wet_fuse=0
            )
            fcd = interp1d(dp["CL"], dp["CD"], fill_value="extrapolate")
            cd = fcd(cl)

            station = np.array(struc["station"][0])
            Fshear_data = np.array(struc["ShearF"])
            Mbend_data = np.array(struc["BendM"])
            Fshear = np.zeros(len(station))
            Mbend = np.zeros(len(station))
            for k, st in enumerate(station):
                fF = interp1d(dp["CL"], Fshear_data[:, k], fill_value="extrapolate")
                fM = interp1d(dp["CL"], Mbend_data[:, k], fill_value="extrapolate")
                Fshear[k] = fF(cl)
                Mbend[k] = fM(cl)
            struc = pd.DataFrame(
                {"struc_st": station, "struc_F": Fshear, "struc_M": Mbend}
            )

            # Propulsion
            Preq = Prequired(rho, v, cd, S, mtot, g0, rocmin, accel, etap1, etap2)
            # Pcruise = cruise_power(rho, v, cd, S, etap1)
            # Paccelerate = accelerate(rho, v, cd, S, mtot, accel, etap2)
            # Pclimb = climb(rho, vcruisemin, cd, S, mtot, g0, rocmin, etap2)
            # Preq = max(Pclimb, Pcruise, Paccelerate)

            # Power
            Ppay = Power_Payload()
            Preq += Ppay
            CASE.at[0, "P"] = Preq
            # Sizing & Dimensioning
            cg_UPD = Position_Long_Stable(
                CASE, m, cg_set, Static_M, npt
            )  # Need updated netural point position and replace value of 1 here
            # Testing Convergence
            print(f"Total Mass [kg]: {mtot}")
            print(f"Syst. Power: {Ppay}")
            print(f"Preq: {Preq}, Pguess: {P0 * 1000}")
            convergence = abs((Preq - P0 * 1000))
            P0 = (2 / 3) * (Preq - P0 * 1000) / 1000 + P0
            # P0 = Preq/1000
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
        Out_fuse_L[j][i] = fuse_dim[-1]
        Out_fuse_x[j][i] = cg_UPD.Other[0]
        Out_fin_S[j][i] = swl
        Out_cgx[j][i] = cg_UPD.Net[0]
        Out_RTG_h[j][i] = fuse_dim[1]
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
df7 = pd.DataFrame(
    data=Out_fuse_L[0:, 0:],
    index=[i for i in range(Out_fuse_L.shape[0])],
    columns=["f" + str(i) for i in range(Out_fuse_L.shape[1])],
)
df8 = pd.DataFrame(
    data=Out_fuse_x[0:, 0:],
    index=[i for i in range(Out_fuse_x.shape[0])],
    columns=["f" + str(i) for i in range(Out_fuse_x.shape[1])],
)
df9 = pd.DataFrame(
    data=Out_fin_S[0:, 0:],
    index=[i for i in range(Out_fin_S.shape[0])],
    columns=["f" + str(i) for i in range(Out_fin_S.shape[1])],
)
df10 = pd.DataFrame(
    data=Out_cgx[0:, 0:],
    index=[i for i in range(Out_cgx.shape[0])],
    columns=["f" + str(i) for i in range(Out_cgx.shape[1])],
)
df11 = pd.DataFrame(
    data=Out_RTG_h[0:, 0:],
    index=[i for i in range(Out_RTG_h.shape[0])],
    columns=["f" + str(i) for i in range(Out_RTG_h.shape[1])],
)

df1.to_csv(os.path.join(ROOT_DIR, "Out/S_PlanformArea.csv"))
df2.to_csv(os.path.join(ROOT_DIR, "Out/b_Span.csv"))
df3.to_csv(os.path.join(ROOT_DIR, "Out/m_Mass.csv"))
df4.to_csv(os.path.join(ROOT_DIR, "Out/P_Power.csv"))
df5.to_csv(os.path.join(ROOT_DIR, "Out/CD_DragCoeff.csv"))
df6.to_csv(os.path.join(ROOT_DIR, "Out/CL_LiftCoeff.csv"))
df7.to_csv(os.path.join(ROOT_DIR, "Out/fuse_L.csv"))
df8.to_csv(os.path.join(ROOT_DIR, "Out/fuse_x.csv"))
df9.to_csv(os.path.join(ROOT_DIR, "Out/Sfin_FinArea.csv"))
df10.to_csv(os.path.join(ROOT_DIR, "Out/cg_x.csv"))
df11.to_csv(os.path.join(ROOT_DIR, "Out/RTG_h.csv"))


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
