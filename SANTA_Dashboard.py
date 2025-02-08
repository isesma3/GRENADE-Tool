import os

ROOT_DIR = os.path.dirname(__file__)

import numpy as np
import streamlit as st
import sys
from math import pi as PI

sys.path.insert(0, ROOT_DIR + "/Environment")
sys.path.insert(0, ROOT_DIR + "/Aero/Airfoils")
from Environment.TitanAtm import *

# Dashboard code
st.set_page_config(layout="wide")

# Create Dashboard title
st.title("Sizing Analysis for Non-Terrestrial Aircraft (SANTA)")

# Create user interface

# Create columns
col1,col2,col3 = st.columns(3)
# Mission Requirements
with col1:
    Vm = st.slider("Motor Voltage [Volts]", 1.0, 100.0, 30.0)
    Cruise_Alt = st.slider("Cruise Altitude [meters]", 0.0, 100000.0,800.0)
    Min_Alt = st.slider("Minimum Altitude [meters]", 0.0, 100000.0, 80.0)
    Cruise_V = st.slider("Cruise Velocity [m/s]", 5.0, 1000.0, 14.0)
    RoC_min_alt = st.slider("Rate of climb at minimum altitude [m/s]", 0.0, 25.0, 0.3)
    RoC_max_alt = st.slider("Rate of climb at maximum altitude [m/s]", 0.0, 25.0, 0.1)
    Accel = st.slider("Acceleration [m^2/s]", 0.0, 15.0, 0.02)
    Launch_vehicle_mass_constraint = st.slider("Launch vehicle mass constraint [kg]", 10.0, 5000.0, 300.0)
    D_shell = st.slider("Aeroshell Diameter [m]", 1.0, 10.0, 4.5)

# TOPSIS inputs
with col2:
    b_weight = st.number_input("Wingspan Importance Weight", 0.0, 1.0, 0.1)
    b_weight_att = st.selectbox("Wingspan Metric Attribute", ['Cost', 'Benefit'])

    m_weight = st.number_input("Aircraft Mass Importance Weight", 0.0, 1.0, 0.5)
    m_weight_att = st.selectbox("Aircraft Mass Metric Attribute", ['Cost', 'Benefit'])

    P_weight = st.number_input("Power Importance Weight", 0.0, 1.0, 0.2)
    P_weight_att = st.selectbox("Power Metric Attribute", ['Cost', 'Benefit'])

    CD_weight = st.number_input("Drag Coefficient Importance Weight", 0.0, 1.0, 0.1)
    CD_weight_att = st.selectbox("Drag Coefficient Metric Attribute", ['Cost', 'Benefit'])

    mRTG_weight = st.number_input("RTG Mass Importance Weight", 0.0, 1.0, 0.1)
    mRTG_weight_att = st.selectbox("RTG Mass Metric Attribute", ['Cost', 'Benefit'])

    # Add payload dropdown
    option = st.selectbox(
    'Payload Request',
    ('Earth Comms', 'Surface Mapping', 'Atmospheric Data Collection','Probe Drop'))

# Create case ranges to be inputted

ncases = 4000 # Use 4000 cases for accuracy
n_cases = np.linspace(1,ncases,ncases)

# Create drbatt range
def drbatt_range(ncases):
    drbatt = np.zeros(ncases)
    return drbatt

drbatt = drbatt_range(ncases)

# Create dtbatt range
def dtbatt_range(ncases):
    dtbatt = np.zeros(ncases)
    return dtbatt

dtbatt = dtbatt_range(ncases)

# Create EQ_W_Wing range
def EQ_W_Wing_range(ncases):
    EQ_W_Wing = np.linspace(1,1,ncases)
    return EQ_W_Wing

EQ_W_Wing = EQ_W_Wing_range(ncases)

# Create EQ_W_Therm range
def EQ_W_Therm_range(ncases):
    EQ_W_Therm = np.linspace(1,1,ncases)
    return EQ_W_Therm

EQ_W_Therm = EQ_W_Therm_range(ncases)

# Create Foil0_Range
def Foil0_range(ncases):
    Foil0 = []
    i = 0
    for loop in range(i, ncases):
        Foil0.append('e344')
        i = i + 1
    return Foil0

Foil0 = Foil0_range(ncases)

# Create Foil1_Range
def Foil1_range(ncases):
    Foil1 = []
    i = 0
    for loop in range(i, ncases):
        Foil1.append('e340')
        i = i + 1
    return Foil1

Foil1 = Foil1_range(ncases)

# FOS range
def FOS_range(SF,ncases):
    FOS = np.linspace(SF,SF,ncases)
    return FOS

FOS = FOS_range(1.5,ncases)

# Fuse_L range
def Fuse_L_range(ncases):
    Fuse_L = np.zeros(ncases)
    return Fuse_L

Fuse_L = Fuse_L_range(ncases)

# Fuse_Lt range
def Fuse_Lt_range(ncases):
    Fuse_Lt = np.zeros(ncases)
    return Fuse_Lt

Fuse_Lt = Fuse_Lt_range(ncases)

#NACA0 range
def NACA0_range(ncases):
    NACA0 = []
    i = 0
    for loop in range(i, ncases):
        NACA0.append('FALSE')
        i = i + 1
    return NACA0

NACA0 = NACA0_range(ncases)

#NACA0 range
def NACA1_range(ncases):
    NACA1 = []
    i = 0
    for loop in range(i, ncases):
        NACA1.append('FALSE')
        i = i + 1
    return NACA1

NACA1 = NACA1_range(ncases)

# Ultimate Load factor range
def Nz_range(ncases):
    Nz = np.linspace(7,7,ncases)
    return Nz

Nz = Nz_range(ncases)

# Panels range
def Panels_range(P,ncases):
    Panels = np.linspace(P,P,ncases)
    return Panels

Panels = Panels_range(200,ncases)

# PassN range
def PassN_range(ncases):
    PassN = np.linspace(10,10,ncases)
    return PassN

PassN = PassN_range(ncases)

# Pfrac range
def Pfrac_range(ncases):
    Pfrac = np.zeros(ncases)
    return Pfrac

Pfrac = Pfrac_range(ncases)

# PropBlade range
def PropBlade_range(ncases):
    PropBlade = np.linspace(2,2,ncases)
    return PropBlade

PropBlade = PropBlade_range(ncases)

# PropD range This is just a for the initial iteration. We can add in to the prop loop to select less or more thrusters based off diameter constraints and whatnot
def PropD_range(ncases):
    PropD = np.linspace(0.37,0.37,ncases)
    return PropD

PropD = PropD_range(ncases)

# PropN range
def PropN_range(ncases):
    PropN = np.linspace(1,1,ncases)
    return PropN

PropN = PropN_range(ncases)

#PropulsionN This is just a for the initial iteration. We can add in to the prop loop to select less or more thrusters based off diameter constraints and whatnot
def PropulsionN_range(ncases):
    PropulsionN = np.linspace(4,4,ncases)
    return PropulsionN

PropulsionN = PropulsionN_range(ncases)

# rho_b & rho_s ranges
def rho_range(rho,ncases):
    rho = np.linspace(rho,rho,ncases)
    return rho

rho_b = rho_range(1970,ncases)
rho_s = rho_b

#Vehicle type range
def VehType_range(input,ncases):
    type = []
    i = 0
    for loop in range(i, ncases):
        type.append(input)
        i = i + 1
    return type

Type = VehType_range('bwb',ncases)

# Misc. DoE variables

# Making a ones vector
ones = np.linspace(1,1,ncases)
RTGfuelfrac = Pfrac
RTGfueltype = ones
tmission = ones
tstart = 0*tmission
Ult_n = 724000000*ones
Ult_s = 54500000*ones
Vmot = Vm*ones
Cruise_Alt = Cruise_Alt*ones
Min_Alt = Min_Alt*ones
Cruise_V = Cruise_V*ones
RoC_min_alt = RoC_min_alt*ones
RoC_max_alt = RoC_max_alt*ones
Accel = Accel*ones
#Num_Folds = Num_Folds*ones
Fuse_D = 0.42*ones # need to find out how to either tune this or something idk
Wing_TR = 0.35*ones
Wing_fin_S = 0*ones
Wing_fin_TR = 0*ones
RoC_max_alt = RoC_max_alt*ones
RoC_min_alt = RoC_min_alt*ones
W_inp = Launch_vehicle_mass_constraint


# Create a range of weights based off launch vehicle constraints
def Weight_Range(W,ncases):
    g0 = Titan_Const()[0]
    W = np.linspace(W*0.25,W,ncases)
    return W

W = Weight_Range(W_inp,ncases)

# Using the launch vehicle weight constraints, calculate a range of wing areas 
def Wing_S(W,V):
    rho_cruise = TitanATM_Table(Cruise_Alt, "Recommended")[0]
    CL = 0.55 # Lift coefficient assumption
    S = (2*W)/(rho_cruise*(V**2)*CL)
    return S

S = Wing_S(W,Cruise_V)

# Calculate range of wingspans based off aeroshell constraints
def Wingspan(D,ncases):
    b = np.linspace(0.25*D_shell,D_shell,ncases)
    return b

b = Wingspan(D_shell,ncases)

# Calculate range of aspect ratios
def Wing_AR(S,b):
    AR = (b**2)/S
    return AR

AR = Wing_AR(S,b)

# Calculate range of wing sweeps based off cruise velocity
def Wing_Sweep(V): # Need to model the data better. It estimates too high
    a = TitanATM_Table(Cruise_Alt, "Recommended")[3]
    M = V/a
    Sweep = 3.7745*(M**3)-30.4220*(M**2)+83.4788*M-20.1291
    Sw_W = np.linspace(Sweep,Sweep,ncases)
    Sw_W = np.where(M < 0.3, 0, Sweep)
    return(Sw_W)
    
Sw_W = Wing_Sweep(Cruise_V)

# Calculate the required change in lift per unit span coefficient wrto change in angle of attack (Airfoil lift curve slope)
def airfoil(W,AR,V,S): # Need to edit so it can select an airfoil
    e = 0.85
    aoa_0 = -2
    aoa = 2
    rho_cruise = TitanATM_Table(Cruise_Alt, "Recommended")[0]
    CL = (2*W)/(rho_cruise*(V**2)*S)
    Cla = -CL/((CL/(PI*e*AR)) - (aoa - aoa_0))
    return Cla 

Cla_need = airfoil(W,AR,Cruise_V,S)

# Calculate the required range of horizontal tail areas
def Htail_S(S,b,AR,Cla):
    e = 0.85
    tr = 0.35
    Xbacw = 0.25
    Xbach = -2 # Need to define how these asssumptions were made
    XbCG = 0.1
    etah = 0.8
    CLaw = (Cla/(1 + (Cla)/(PI*e*AR)))*(180/PI)
    CLah = (0.1/(1+ (0.1)/(PI*e*4)))*(180/PI) # Initial assumption of an ARh = 4
    SM = 0.12
    Cma = -SM*CLaw
    XbNP = SM + XbCG
    dah_da = 0.7 # downwash assumption. Need to find a better way to do it 
    Sh = (XbNP*CLaw - CLaw*Xbacw)/(etah*(1/S)*CLah*dah_da*Xbach - XbNP*etah*(1/S)*CLah*dah_da*Xbach)
    return Sh

# Calculate the Sh by assuming its a percentage of the wing area and then move RTG to obtain good SM

Sh = Htail_S(S,b,AR,Cla_need)

# Calculate the required range of vertical tail areas
def Vtail_S(S,b,lb,tr): # Need to get more physics based assumptions
    cr = 2*S/(b*(1+tr))
    cb = (2/3)*cr*((1+tr+tr**2)/(1+tr))
    lv = lb + 0.75*cb
    CnBv = 0.04
    etav = 0.9
    dsig_dB = 0.3
    e = 0.85
    ARh = 1.5
    CLav = (0.1/(1+ 0.1/(PI*e*ARh)))*(180/PI)
    Sv = (CnBv*S)/(lv*etav*CLav*(1-dsig_dB))
    return Sv

lb = 0.27
Sv = Vtail_S(S,b,lb,Wing_TR[0])

# Calculate the required range of wing dihedral angles
def wing_dihedral(ncases,Cla,AR,tr):
    ClBG = np.linspace(-0.0415,-0.0829,ncases)
    e = 0.85
    CLaw = Cla/(1 + Cla/(PI*e*AR))
    dih_wing = (-6*ClBG/CLaw)*((1+tr)/(1+2*tr))
    return dih_wing

dih_wing = wing_dihedral(ncases,Cla_need,AR,Wing_TR[0])


# Surrogate models
# Find out which variables need to be outputted with the surrogate model

m = W
P = [456.9501353,	457.747348,	395.5562889,	364.6161502,	192.4871235,	436.2250448,	397.7609402,	501.8438866,	452.1799591,	475.9976144,	506.4768937,	549.8476297,	580.4804957,	611.7992754,	636.7256729,	613.0994494]
CD = [0.156939259,	0.130824527,	0.094255839,	0.074651151,	0.024716862,	0.0736246,	0.059819216,	0.069045972,	0.058404085	,0.057221376,	0.057072635,	0.058735144,	0.058634185,	0.05860927,	0.057981056,	0.053696198]
mRTG = [105.5257185	,105.8542932,	91.74391728,	84.19245404	,64.63094443,	100.4236362,	92.0407299,	155.5575423	,106.4514044	,110.3806822	,116.3287306	,125.7088915	,132.6310181,	140.0472686	,145.8853546,	135.5345094]

# TOPSIS
CASE_END = 16

def ssq2(x):
    i = 0
    x2 = []
    for i in range(CASE_END):
        x2.append(x[i]**2)
    xs = sum(x2)
    xssq2 = np.sqrt(xs)
    return xssq2

i = 0
b1 = []
m1 = []
P1 = []
CD1 = []
mRTG1 = []
for i in range(CASE_END):
    b1.append(b[i] / ssq2(b))
    m1.append(m[i] / ssq2(m))
    P1.append(P[i] / ssq2(P))
    CD1.append(CD[i] / ssq2(CD))
    mRTG1.append(mRTG[i] / ssq2(mRTG))

# Multiply by the weights
b_weighted = [val * float(b_weight) for val in b1]
m_weighted = [val * float(m_weight) for val in m1]
P_weighted = [val * float(P_weight) for val in P1]
CD_weighted = [val * float(CD_weight) for val in CD1]
mRTG_weighted = [val * float(mRTG_weight) for val in mRTG1]

# Apply the criterion
def pos(x,c):
    if c == 'Cost':
        y = np.min(x)
    else:
         y = np.max(x)
    return y

import numpy as np

def neg(x, c):
    if c == 'Cost':
        y = np.max(x)
    else:
        y = np.min(x)
    return y
 
             
# Seperations
i = 0
Sip = []
Sin = []
for i in range(CASE_END):
    Sip.append(np.sqrt((b_weighted[i] - pos(b_weighted,b_weight_att))**2 + (m_weighted[i] - pos(m_weighted,m_weight_att))**2 + (P_weighted[i] - pos(P_weighted,P_weight_att))**2 + (CD_weighted[i] - pos(CD_weighted,CD_weight_att))**2 + (mRTG_weighted[i] - pos(mRTG_weighted,mRTG_weight_att))**2))
    Sin.append(np.sqrt((b_weighted[i] - neg(b_weighted,b_weight_att))**2 + (m_weighted[i] - neg(m_weighted,m_weight_att))**2 + (P_weighted[i] - neg(P_weighted,P_weight_att))**2 + (CD_weighted[i] - neg(CD_weighted,CD_weight_att))**2 + (mRTG_weighted[i] - neg(mRTG_weighted,mRTG_weight_att))**2))

# Closeness values
C = [] 
for i in range(CASE_END):
    C.append(Sin[i]/(Sip[i]+Sin[i]))

# Determine the max closeness value and what case 
A = np.max(C)
Case_num = C.index(A)

# # Aircraft visualization

# # Given parameters for the BWB
# total_span = b[Case_num] / 2  # meters, total wingspan halved for symmetry
# desired_volume = 1  # m^3, significantly larger desired volume for the first 4 sections

# # Original span length ratios
# original_span_lengths = b
# adjusted_span_lengths = [x / sum(original_span_lengths) * total_span for x in original_span_lengths]
# span_starts = np.cumsum([0] + adjusted_span_lengths[:-1])

# # Calculate linear interpolation for chord lengths from root to tip
# root_chord = (2*S[Case_num]/(b*(1+Wing_TR)))
# tip_chord = Wing_TR*root_chord
# chord_lengths = np.linspace(root_chord, tip_chord, len(adjusted_span_lengths))

# # Increase chord lengths for the first four sections
# scale_factor = 2  # Example scale factor for demonstration
# for i in range(4):  # Adjust only the first four sections
#     chord_lengths[i] *= scale_factor

# # Adjust the rest of the chord lengths to ensure a smooth transition
# # This might involve recalculating the taper ratio or adjusting chords directly

# # Generate sections data with adjusted chords for the first four sections
# sections = []
# for i in range(len(adjusted_span_lengths)):
#     span_end = span_starts[i] + adjusted_span_lengths[i]
#     current_root_chord = chord_lengths[i]
#     current_tip_chord = chord_lengths[i + 1] if i + 1 < len(chord_lengths) else chord_lengths[i]

#     sections.append({
#         'Span Start': span_starts[i],
#         'Span End': span_end,
#         'Span Length': adjusted_span_lengths[i],
#         'Root C': current_root_chord,
#         'Tip C': current_tip_chord,
#         'Sweep': Sw_W[0],  # Sweep angle
#         'Twist': 0,  # Twist angle
#         'Dihedral': dih_wing[0]  # Dihedral angle
#     })

# # Convert to DataFrame for display
# df_sections = pd.DataFrame(sections)

# df_sections

# #print(df_sections.to_string(index=False))

with col3:
    # Create tabs
    tab1,tab2,tab3 = st.tabs(['Aircraft Visualization','Payload','Detailed Design'])
    with tab1:
        st.header("Aircraft Visualization")
        st.image("C:/Users/tyobe/OneDrive/Documents/GitHub/SANTA/NTADE_BwB.png", use_column_width="always")
    with tab2:
        st.header("Payload Configuration")
        st.image("C:/Users/tyobe/OneDrive/Documents/GitHub/SANTA/Payload_Image.png", use_column_width="always")
    with tab3:
        st.header("Detailed Design table")
        data = {
            'Values': [
                [0.537, 2.025, 153.84, 64.63, 192.49, 0, 0.025],
            ]
        }

        column_names = ['Wing Area [m^2]', 'Wing Span [m]', 'Total Mass [kg]', 'RTG Mass [kg]', 'Power Required [Watts]', 'Number of Folds', 'Drag Coefficient']

        df = pd.DataFrame(data['Values'], columns=column_names)
        df_rounded = df.round(2)  # Round to two decimal places
        styled_df = df_rounded.T.style.format("{:.3f}")  # Transpose and then apply styling
        st.table(styled_df)