# Reformat using Black Format Extenstion

import os

ROOT_DIR = os.path.dirname(__file__)

import numpy as np
import pandas as pd
import sys
import scipy
import matplotlib.pyplot as plt
from timeit import default_timer

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
from Propulsion.PropellerDnP import *
from Weight.TitanWeight import *
from Weight.Structure import *
from Environment.TitanAtm import *
from Aero.AnS_FlyingWing import *
from Aero.parasitic_drag_func import *
from Payload.Power_Systems import *
from Flight_Requirements.Mission import *
from Flight_Requirements.Flight_Performance import SegRequirements as ReqIN
from Flight_Requirements.Performance_Metrics import *

# Constants
ge = 9.80665  # Earth gravitational constant [m/s^2]
lbf2N = 4.44822  # [lbf/N]
N2lbf = 0.224809  # [N/lbf]
m2ft = 3.28084  # [m/ft]
psf2Pa = 0.020885434273039  # [psf/Pa]

## Load inputs

DOE = pd.read_csv(os.path.join(ROOT_DIR, "Inp/DOE.csv"))

# Environemnt
g0 = Titan_Const()[0]
DIAMETER = Titan_Const()[1]
## Design Loop


# Constraints
D_shell = 4
# bmax = D_shell * 2
Static_M = 0.07
tol = 0.1
CLmax = 1.5

# Aero inputs


CASE_START = 1
CASE_END = 4

if CASE_END == "end":
    loop_end = DOE.shape[0]
else:
    loop_end = CASE_END
Len1 = loop_end - (CASE_START - 1)
# v_array = np.linspace(vcruisemin, vcruisemax, num=Len1)
Sref = DOE.Wing_S.to_numpy()

wingx0 = 0
CGx = 0
rootfoil = DOE.Foil0[0]
tipfoil = DOE.Foil1[0]
swl = DOE.Wing_fin_S[0]
trwl = DOE.Wing_fin_TR[0]
Lf = DOE.Fuse_L[0]
df = DOE.Fuse_d[0]

Len2 = 1
# Propulsion Inputs
etap1 = 0.8
etap2 = 0.9

# Output Arrays
Out_S = np.zeros((Len1, Len2))
Out_b = np.zeros((Len1, Len2))
Out_P = np.zeros((Len1, Len2))
Out_m = np.zeros((Len1, 14))
Out_CD = np.zeros((Len1, Len2))
Out_CL = np.zeros((Len1, Len2))
Out_PropD = np.zeros((Len1, Len2))
Out_fuse_L = np.zeros((Len1, Len2))
Out_fuse_x = np.zeros((Len1, Len2))
Out_fin_S = np.zeros((Len1, Len2))
Out_cgx = np.zeros((Len1, Len2))
Out_RTG_h = np.zeros((Len1, Len2))
Out_check = np.array(np.zeros((Len1, Len2)), dtype=object)
Out_Limiting = np.array(np.zeros((Len1, Len2)), dtype=object)
Out_Perf = np.zeros((Len1, 3))
Out_Case = np.zeros((Len1, Len2))
loop = 0
j = 0

start1 = default_timer()
for loop in range(CASE_START - 1, loop_end):
    start = default_timer()
    I = 0
    # print(f"Case {loop-CASE_START+2} of {loop_end-CASE_START+1}")
    print(f"Case {loop + 1} of {loop_end}")
    CASE = DOE.iloc[[loop]]
    CASE.index = [0]

    max_alt = CASE.Max_Alt[0]
    min_alt = CASE.Min_Alt[0]
    cruise_alt = (
        CASE.Cruise_Alt[0]
        if "Cruise_Alt" in CASE.columns
        else 0.5 * (max_alt + min_alt)
    )
    cruise_v = CASE.Cruise_V[0]
    v = CASE.v[0] if "v" in CASE.columns else cruise_v
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
        a = TitanATM_Table(h, "Recommended")
        rho_vec[i] = a[0]
        mu_vec[i] = a[5]
        a_vec[i] = a[3]
        T_vec[i] = a[2]

    rho_low = rho_vec[0]
    rho_cruise = TitanATM_Table(cruise_alt, "Recommended")[0]
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

    if "Wing_TC" in CASE.columns:
        CASE.at[0, "Wing_TC"] = (root[0] + tip[0]) / 2
    else:
        CASE = pd.concat(
            [CASE, pd.DataFrame({"Wing_TC": [(root[0] + tip[0]) / 2]})], axis=1
        )

    if CASE["Num_Folds"][0] == 1:
        bmax = 3 * D_shell
    else:
        bmax = (CASE["Num_Folds"][0] + 1) * D_shell
    AR = CASE.Wing_AR[0]
    S = CASE.Wing_S[0]
    b = np.sqrt(AR * S)
    k = 1 / np.pi / 0.85 / (AR)
    P0 = 0.12
    convergence = 1
    struc = 0
    while convergence > tol:
        q = q_calc(rho_cruise, v)
        CASE.at[0, "q"] = q
        # Weight
        if not (hasattr(struc, "__len__")):
            CASE.at[0, "EQ_W_Wing"] = 0
        else:
            CASE.at[0, "EQ_W_Wing"] = DOE.iloc[loop].EQ_W_Wing
        CASE.at[0, "PropD"] = calculate_propeller_diameter(CASE)
        m, fuse_dim, cg_set, Pinfo = Weights(CASE, struc)
        if hasattr(fuse_dim, "__len__") and CASE.Type[0] == "Wing":
            CASE.at[0, "Fuse_L"] = fuse_dim[-1]
        CASE.at[0, "P"] = Pinfo[0]
        CASE.at[0, "PHeat"] = Pinfo[1]
        mtot = m.to_numpy()[0][0]

        # Aero
        cl = cl_calc(mtot, g0, q, S)
        if cl > 1.5:
            cl = 1.5

        mach = np.divide(v, sos)
        # [dp,stab,wl,struc] = flywingaero(b,S,tr,mach,rho,v,g0,sweep,dihedral,bwl,swl,trwl,sweepwl)
        [dp, swl, wingletpnt, SM, npt, struc, Yaw_check, Pitch_check] = flywingaero(
            CASE, mach, rho_cruise, mu, v, g0, CGx, D_shell
        )
        CASE.at[0, "Wing_fin_S"] = swl
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
        struc = pd.DataFrame({"struc_st": station, "struc_F": Fshear, "struc_M": Mbend})

        # Propulsion
        inp_low = [rho_low, min_v, roc_min_alt, accel]
        inp_high = [rho_high, max_v, roc_max_alt, 0]
        inp_cruise = [rho_cruise, cruise_v, (roc_max_alt + roc_min_alt) / 2, accel / 2]
        Preq, PLimiting = Prequired(
            inp_low, inp_cruise, inp_high, cd, S, mtot, g0, etap1, etap2
        )

        # Pcruise = cruise_power(rho, v, cd, S, etap1)
        # Paccelerate = accelerate(rho, v, cd, S, mtot, accel, etap2)
        # Pclimb = climb(rho, vcruisemin, cd, S, mtot, g0, rocmin, etap2)
        # Preq = max(Pclimb, Pcruise, Paccelerate)

        # Power
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
        # Testing Convergence
        print(f"Total Mass [kg]: {mtot}")
        # print(f"Syst. Power: {Ppay}")
        print(f"Preq: {Preq}, Pguess: {P0 * 1000}")
        convergence = abs((Preq - P0 * 1000))
        P0 = (2 / 3) * (Preq - P0 * 1000) / 1000 + P0
        # P0 = Preq/1000
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
        if fuse_dim[-1] > D_shell:
            if I > 10:
                convergence = tol / 2
            else:
                I += 1
            # fuse_dim[-1] = float("nan")
            Check = "Fuselage too big"
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
                Check = "Winglet too big"
            elif Yaw_check == -1:
                Check = "Winglet sizing exceeded iter. lim"
            else:
                Check = "Missin Cnb in results"
        if Pitch_check < 1:
            if I > 10:
                convergence = tol / 2
            else:
                I += 1
            Check = "Missing Cma in results"
    duration = default_timer() - start
    print(f"Case {loop + 1} took {duration} seconds")
    # Performance Metrics
    PerfA = aero_perf(
        CASE.PassN[0], v, DIAMETER * math.pi, CASE.tmission[0], min_alt, max_alt
    )
    PerfG = grnd_perf(v, PerfA, DIAMETER)

    ## Mission Performance
    Out_Case[j] = loop + 1
    Out_S[j] = S
    Out_b[j] = b
    Out_m[j] = m.to_numpy()[0]
    Out_P[j] = Preq
    Out_CD[j] = cd
    Out_CL[j] = cl
    Out_fuse_L[j] = fuse_dim[-1]
    Out_fuse_x[j] = cg_UPD.Other[0]
    Out_fin_S[j] = swl
    Out_cgx[j] = cg_UPD.Net[0]
    Out_RTG_h[j] = fuse_dim[1]
    Out_check[j] = Check
    Out_Limiting[j] = PLimiting
    Out_Perf[j][0] = PerfA[0] / (365.25 * 24 * 3600)
    Out_Perf[j][1] = PerfA[1]
    Out_Perf[j][2] = PerfG
    Out_PropD[j] = CASE.PropD[0]
    j += 1
total_duration = default_timer() - start1
print(total_duration)
# print(Out_P)

df = pd.DataFrame(
    data={
        "Case": Out_Case[0:, 0],
        "Planform": Out_S[0:, 0],
        "Span": Out_b[0:, 0],
        "Mass": Out_m[0:, 0],
        "Power": Out_P[0:, 0],
        "PowerConstraint": Out_Limiting[0:, 0],
        "AtmPassDeltaH": Out_Perf[0:, 1],
        "GroundTime": Out_Perf[0:, 2],
        "GndAreaFrac": Out_Perf[0:, 2],
        "CD": Out_CD[0:, 0],
        "CL": Out_CL[0:, 0],
        "Fuse_L": Out_fuse_L[0:, 0],
        "Fuse_x": Out_fuse_x[0:, 0],
        "Fin_S": Out_fin_S[0:, 0],
        "cgx": Out_cgx[0:, 0],
        "RTG_h": Out_RTG_h[0:, 0],
        "PropD": Out_PropD[0:, 0],
        "Check": Out_check[0:, 0],
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
    },
    index=[i for i in range(Out_S.shape[0])],
)

# df1 = pd.DataFrame(data=Out_S[0:, 0:], index=[i for i in range(Out_S.shape[0])],
#                    columns=['f' + str(i) for i in range(Out_S.shape[1])])
# df2 = pd.DataFrame(data=Out_b[0:, 0:], index=[i for i in range(Out_b.shape[0])],
#                    columns=['f' + str(i) for i in range(Out_b.shape[1])])
# df3 = pd.DataFrame(data=Out_m[0:, 0:], index=[i for i in range(Out_m.shape[0])],
#                    columns=['f' + str(i) for i in range(Out_m.shape[1])])
# df4 = pd.DataFrame(data=Out_P[0:, 0:], index=[i for i in range(Out_P.shape[0])],
#                    columns=['f' + str(i) for i in range(Out_P.shape[1])])
# df5 = pd.DataFrame(data=Out_CD[0:, 0:], index=[i for i in range(Out_CD.shape[0])],
#                    columns=['f' + str(i) for i in range(Out_CD.shape[1])])
# df6 = pd.DataFrame(data=Out_CL[0:, 0:], index=[i for i in range(Out_CL.shape[0])],
#                    columns=['f' + str(i) for i in range(Out_CL.shape[1])])
# df7 = pd.DataFrame(data=Out_fuse_L[0:, 0:], index=[i for i in range(Out_fuse_L.shape[0])],
#                    columns=['f' + str(i) for i in range(Out_fuse_L.shape[1])])
# df8 = pd.DataFrame(data=Out_fuse_x[0:, 0:], index=[i for i in range(Out_fuse_x.shape[0])],
#                    columns=['f' + str(i) for i in range(Out_fuse_x.shape[1])])
# df9 = pd.DataFrame(data=Out_fin_S[0:, 0:], index=[i for i in range(Out_fin_S.shape[0])],
#                    columns=['f' + str(i) for i in range(Out_fin_S.shape[1])])
# df10 = pd.DataFrame(data=Out_cgx[0:, 0:], index=[i for i in range(Out_cgx.shape[0])],
#                    columns=['f' + str(i) for i in range(Out_cgx.shape[1])])
# df11 = pd.DataFrame(data=Out_RTG_h[0:, 0:], index=[i for i in range(Out_RTG_h.shape[0])],
#                    columns=['f' + str(i) for i in range(Out_RTG_h.shape[1])])
# df12 = pd.DataFrame(data=Out_check[0:, 0:], index=[i for i in range(Out_check.shape[0])],
#                    columns=['f' + str(i) for i in range(Out_check.shape[1])])

df.to_csv(
    os.path.join(
        ROOT_DIR, "Out/Compiled" + str(CASE_START) + "-" + str(CASE_END) + ".csv"
    )
)
# df1.to_csv(os.path.join(ROOT_DIR, 'Out/S_PlanformArea.csv'))
# df2.to_csv(os.path.join(ROOT_DIR, 'Out/b_Span.csv'))
# df3.to_csv(os.path.join(ROOT_DIR, 'Out/m_Mass.csv'))
# df4.to_csv(os.path.join(ROOT_DIR, 'Out/P_Power.csv'))
# df5.to_csv(os.path.join(ROOT_DIR, 'Out/CD_DragCoeff.csv'))
# df6.to_csv(os.path.join(ROOT_DIR, 'Out/CL_LiftCoeff.csv'))
# df7.to_csv(os.path.join(ROOT_DIR, 'Out/fuse_L.csv'))
# df8.to_csv(os.path.join(ROOT_DIR, 'Out/fuse_x.csv'))
# df9.to_csv(os.path.join(ROOT_DIR, 'Out/Sfin_FinArea.csv'))
# df10.to_csv(os.path.join(ROOT_DIR, 'Out/cg_x.csv'))
# df11.to_csv(os.path.join(ROOT_DIR, 'Out/RTG_h.csv'))
# df12.to_csv(os.path.join(ROOT_DIR, 'Out/Check.csv'))
#

## Plot
#
# fig, ax = plt.subplots()
# x = np.divide(Out_m, Out_S)
# y = np.divide(Out_P, Out_m)
# j = 0
# for j in range(0,DOE.shape[0]):
#    ax.plot(x[j], y[j], linewidth=2.0, marker='o', label=str(v))
#    j += 1
# plt.legend()
# plt.ylabel("Power-to-Mass Ratio, P/m [W/kg]")
# plt.xlabel("Wing-Loading, m/S [kg/m^2]")

# fig, ax = plt.subplots()
# x = Out[0]
# y = np.divide(Out[2],Out[1])
# ax.plot(x, y, linewidth=2.0, marker ='o')
# plt.ylabel("Power-to-Mass Ratio, P/m [W/kg]")
# plt.xlabel("S [m^2]")


# plt.show() 