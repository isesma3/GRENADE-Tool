# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# File Name: TitanWeight.py                                 
# Creat Date: January 3, 2023                        
# Contributors (In-Order): Daniel J. Moore, Jacob Webb, Yassine Fouchal                   
# Last Edited: March 13, 2023                    
# Last Edited By: Jacob Webb                         
# For: Aerospace Systems Design Lab @ Georgia Tech     
# Description: 
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Functions:                                          
#   Weights()
#   Weight_Battery()
#   Weight_Payload()
#   Weight_Empty()
#   Weight_Wing()
#   Weight_Fuse()
#   Weight_FC()
#   Weight_Emp()
#   Weight_EPS()
#   Weight_Avion()
#   Req_Fuse_Dim()
#
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Version Sumamry
#   1 - Initial Commit
#   2 - Update as Masses
#   3 - Add beam-bending weight-based estimate (current version)
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Additional Notes
#
#
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Import Libraries
import os
from Environment.MarsAtm import *
LOCAL_DIR = os.path.dirname(__file__)
ROOT_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))
import pandas as pd
from scipy.interpolate import interp1d
from scipy.optimize import root_scalar as root
import numpy as np
import math
from scipy.optimize import minimize
from Structure import *

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def Prelim_Weights(Data):
    '''
    Inputs:
        Data - Dataframe of geometry, performance parameters, and fixed masses

    Outputs:
        W_set - Dataframe of Masses [kg]
                Mass [Total, RTG, Battery, Payload, Empty]
    '''
    # Define Constants
    ge = 9.80665 # Earth gravitational constant [m/s^2]
    lbf2N = 4.44822 # [lbf/N]
    N2lbf = 0.224809    # [N/lbf]

    # Import Masses
    mbatt = Data.Batt[0]
    mpay = Data.Pay[0]
    #Wbatt = Weight_Battery()
    mRTG = Weight_RTG(Data)
    mpay = Weight_Payload(Data)

    # Convert to Earth lbf for calculation
    Wbatt = mbatt * ge * N2lbf
    Wpay = mpay * ge * N2lbf
    WRTG = mRTG * ge * N2lbf

    # Iteration Loop
    Wsum_guess = 1000
    tol = 0.1
    iter = 10000
    Wsum = 1500
    i = 1
    while abs(Wsum-Wsum_guess) > tol and i < iter:
        Wsum_guess = Wsum
        Wempty = 1.93*np.power(Wsum,0.815)    #Based on fits from Nicolai and Carichner
        Wsum = Wbatt + Wpay + Wempty + WRTG
        i += 1
    if i >= iter:
        print('Weight difference greater than tolerance: ',Wsum-Wsum_guess)

    # Convert to Mass
    msum = Wsum * (lbf2N/ge)
    mempty = Wempty * (lbf2N/ge)

    # Compile Outputs
    d = {'Total': [msum], 'RTG': [mRTG], 'Battery': [mbatt], 'Payload': [mpay], 'Empty': [mempty]}
    m_set = pd.DataFrame(data = d)
    return m_set

def Weights(Data,struc,Nf,fold_loc):
    '''
    Inputs:
        Data - Dataframe of geometry, performance parameters, and fixed masses

    Outputs:
        Data        - Dataframe of Masses [kg]
                        Weight [Total, RTG, Battery, Payload, Empty, Wing, Fuselage, Flight Controls, Stabilizers, Propulsion, Avionics, Heating]
        fuse_dim    - Fuselage Dimensions, nonzero array if Flying Wing [m^3, m, m, m]
                        [Volume, Cylindrical RTG height, other height, Total Fuselage Length]
    Assumptions:
        - Cylindrical fuselage with circular caps assumed to estimate fuselage wetted area
        - 2.1x wing planform estimage for wetted area
    
    '''
    # Define Constants
    ge = 9.80665 # Earth gravitational constant [m/s^2]
    lbf2N = 4.44822 # [lbf/N]
    N2lbf = 0.224809    # [N/lbf]
    
    # Import Parmeters for initial calculations
    AR = Data.Wing_AR[0]
    S = Data.Wing_S[0]
    TR = Data.Wing_TR[0]

    df = Data.Fuse_d[0]
    Lf = Data.Fuse_L[0]
    Type = Data.Type[0]

    #Initial calculations
    b = np.sqrt(AR*S)           # wing span
    cr = (2*b)/(AR*(1+TR))      # wing root chord
    ct = (2*b*TR)/(AR*(1+TR))   # wing tip chord
    Swf = math.pi*df*Lf         # Estiamted fuselage wetted area
    Sw = 2.1*S                  # Estimated wing wetted area

    # Updata Data
    Data = pd.concat([Data,pd.DataFrame({"Wing_b":[b],"Wing_cr":[cr],"Wing_ct":[ct],"Fuse_Sw":[Swf],"Wing_Sw":[Sw],"Wdg":[0]})],axis=1)
    # Extract Data

    # Iteration Loop
    msum_guess = 50
    msum = 75
    tol = 0.1
    iter = 2500
    i = 1
    fuse_dim = 0
    P = 0
    Pheat = 0
    while  abs(msum-msum_guess) > tol and i < iter and abs(msum) < 1000000:
        msum_guess = 0.5*(msum - msum_guess)+msum_guess
        Data["Wdg"] = Data["Wdg"].astype(float)
        Data.loc[0,"Wdg"] = msum_guess

        # Calculate Component Weights
        mbatt = 0
        mspanels = 0
        mRTG = 0
        mRTG,Check = Weight_RTG(Data)
        mbatt = Weight_Battery(Data)
        mspanels = Weight_SolarPanels(Data)
        mpay = Weight_Payload(Data)
        if Data.EQ_W_Wing[0] and hasattr(struc,"__len__"):
            mwing, cg_wing = SkinPanel(Data,struc)
        else:
            mwing = Weight_Wing(Data)
            cg_wing = [0.25*cr,0,0]
        if Type == 'Wing' or Type == 'wing' or Type == 'bwb' and Data.Fuse_d[0] > 0:
            fuse_dim = Req_Fuse_Dim(Data)
            Data.at[0, 'Fuse_L'] = fuse_dim["Fuse_L"]
            Data.at[0, 'Fuse_Lt'] = fuse_dim["Fuse_L"]
            Data.at[0, 'Fuse_d'] = fuse_dim["Fuse_d"]
            Data.at[0, 'Fuse_Sw'] = math.pi*fuse_dim["Fuse_d"]*fuse_dim["Fuse_L"]
        mfuse = Weight_Fuselage(Data)
        mfc = Weight_FC(Data)
        mstab = Weight_Stabilizer(Data)
        mprop = Weight_Propulsion(Data)
        mav = Weight_Avionics(Data)
        #if Solv[1] == 1:
        #    mht = Weight_Thermal(Data)
        #else:
        if Data.EQ_W_Therm[0]:
            mht,minsul,mrad,mheater,Data= Weight_Thermal_Ctrl(Data)
            P = Data.P[0]
            Pheat = Data.PHeat[0]
        else:
            mht,minsul,mrad,mheater = Weight_Heat(Data)[0:4]
            
        mempty = sum([mRTG,mbatt,mwing,mfuse,mfc,mstab,mprop,mav,mht,mspanels])
        if Nf != 0:
            mfold_TOGW = Weight_WingFold(b, cr, ct, fold_loc)
            mfold = Nf*mempty*mfold_TOGW
        else:
            mfold = 0
        msum = mpay + mempty + mfold

        i += 1
        if i >= iter:
            print('Failed to Converge within', iter, 'iterations')
            print('Weight difference greater than tolerance: ',msum-msum_guess)

    d = {'Total': [msum], 'RTG': [mRTG], 'Battery': [mbatt], 'Payload': [mpay], 'Empty': [mempty], 'Wing': [mwing],'Fuselage': [mfuse], 'Flight Controls': [mfc], 'Stabilizers': [mstab], 'Propulsion': [mprop], 'Avionics': [mav], 'Insulation': [minsul], 'Radiator': [mrad], 'Heater': [mheater], 'Solar Panels': [mspanels], 'Wing fold': [mfold]}
    m_set = pd.DataFrame(data = d)
    cg_set =pd.DataFrame(data = {'Wing':cg_wing})
    Pupdate = [P,Pheat]
    return m_set, fuse_dim, cg_set,Pupdate, Check
    



def Weight_RTG(Data):
    '''
    Inputs:
        P           - Required Power [kW]
        t           - Time since RTG start [yr]
        RTGfuelfrac - molar fraction of U232 in fuel
        RTGfueltype - 0 --> RTG
                      1 --> SRG
    Outputs:
        mRTG - Mass of Radiothermo Generator [kg]
    Assumptions:
        Casing/structure sized for same themal output as Pu238-based
    '''
    # Extract Data
    PfracRTG = Data.PfracRTG[0]
    P = Data.P[0]*1000*PfracRTG
    t = Data.tmission[0]        if "tmission"           in Data.columns else 1
    ts = Data.tstart[0]     if "tstart"         in Data.columns else 0
    n = Data.RTGfuelfrac[0] if "RTGfuelfrac"    in Data.columns else 0
    T = Data.RTGfueltype[0] if "RTGfueltype"    in Data.columns else 0
    nRTG = max(round(P/150,0),1)
    P = P/nRTG
    Check = 1
        

    # Setup Time-Dependency Function
    df = pd.read_csv(os.path.join(LOCAL_DIR,'RTG_Power_vs_Time.csv'))
    df = df.to_numpy()
    xU = df[:,0]
    yU = df[:,1]
    xPu = df[:,2]
    yPu = df[:,3]
    PF_U = interp1d(xU,yU)
    PF_Pu = interp1d(xPu,yPu)
    PF1 = lambda x: n*PF_U(x) + (1-n)*PF_Pu(x)

    # Read in RTG info
    RTGdf = pd.read_csv(os.path.join(LOCAL_DIR,'RTG.int'),header = None)
    if T==1:
        Type = 'SRG'
    else:
        Type = 'RTG'
    rhoU = RTGdf.loc[RTGdf[0] == 'rhoU'][1].to_numpy()
    rhoPu = RTGdf.loc[RTGdf[0] == 'rhoPu'][1].to_numpy()
    P2W0 = RTGdf.loc[RTGdf[0] == 'P2W_Pu_'+Type][1].to_numpy()
    MF0 = RTGdf.loc[RTGdf[0] == 'MF_Pu_'+Type][1].to_numpy()

    ### Calculations ###
    # Power-to-Weight with input U232-Pu238 mix
    dt = np.linspace(0,t*1.5,round(t*1.5/0.01))
    if dt.size == 0:
        Pmax = 2000
        Check = 'RTG sizing failure'
    else:
        Pmax = max(PF1(dt))
    P2W = P2W0 / (1+MF0*((1-n*(1-(rhoU/rhoPu)))*(1/Pmax)-1))
    #Evaluate RTG Mass2
    PF2 = lambda x: PF1(x)/Pmax
    Pref = np.min(PF2(np.linspace(ts,ts+t,1000)))
    mRTG = nRTG*P/(Pref*P2W)
    if math.isnan(mRTG): mRTG=0

    return mRTG,Check

def Weight_SolarPanels(Data):
    '''
    Inputs:
        Pfrac   - Power fraction supplied by solar panels
        P       - Required Power [kW]
        Spanel       - Available Surface [m^2]
        Planet        - Studied planet

    Outputs:
        mpanels - Solar Panels Mass [kg]
    '''
    # P = Data.P[0] * 1000
    # PfracSpanels = Data.PfracSpanels[0]
    # Planet = Data.Planet[0]
    # SPeff = 0.3
    # Savail = Data.Wing_S[0] + Data.Fuse_Sw[0]*3/4
    rhoPanels = 2.4 # [kg/m^2] Mass per surface area of solar panels https://www.esa.int/gsp/ACT/doc/POW/ACT-RPT-NRG-0412-SPS_EcoFys_FinalReport.pdf
    # Irrad = 0
    # if Planet == 'Earth':
    #     Irrad = 340 # [W/m^2] Mean value over the year and the day
        
    # if Planet == 'Titan':
    #     Irrad = 15.2 # [W/m^2] Mean value over the year and the day: https://ntrs.nasa.gov/api/citations/20110023012/downloads/20110023012.pdf
    
    # Spanels = P* PfracSpanels/(Irrad * SPeff)
    Spanels = Surface_SolarPanels(Data)
    # if Spanels > Savail:
    #     Spanels = 0
    mPanels = Spanels * rhoPanels
    return mPanels

def Surface_SolarPanels(Data):
    '''
    Inputs:
        PfracSpanels   - Power fraction supplied by solar panels
        P       - Required Power [kW]
        Irrad       - Irradiance  [W/m^2]
        SPeff        - Solar Panels effciency

    Outputs:
        Spanels - Solar Panels area [m^2]
    '''
    P = Data.P[0] * 1000
    PfracSpanels = Data.PfracSpanels[0]
    Planet = Data.Planet[0]
    SPeff = 0.3
    Irrad = 0
    if Planet == 'Earth':
        Irrad = 340 # [W/m^2] Mean value over the year and the day
    if Planet == 'Mars':
        Irrad = 340*0.431
    if Planet == 'Titan':
        Irrad = 15.2 # [W/m^2] Mean value over the year and the day: https://ntrs.nasa.gov/api/citations/20110023012/downloads/20110023012.pdf

    Spanels = P* PfracSpanels/(Irrad * SPeff)
    if Spanels != 0:
        Data.at[0,"Wing_S"] = Spanels*1.2
        Data.at[0,"Span"] = np.sqrt(Data.Wing_S[0]*Data.Wing_AR[0])
    return Spanels


def Weight_Battery(Data):
    '''
    Inputs:
        Pfrac   - Power fraction supplied by battery
        P       - Required Power [kW]
        E       - Endurance [s] (if R nonzero set E=0)
        R       - Range [km] (if E nonzero set R=0)
        v       - Airspeed [m/s] (must be nonzero if R nonzero)

    Outputs:
        mbatt - Battery Mass [kg]

    Assumptions:
        - The fraction of power that is not distributed by the battery, directly comes from solar panels
    '''
    # Extract Data
    P = Data.P[0]
    PfracBattery = Data.PfracBattery[0]
    E = Data.dtbatt[0]
    R = Data.drbatt[0]
    v = Data.v[0]
    daytime = Planet_Const(Data.Planet[0])[2]
    # Constants
    P2W = 150 # [wh/kg] From Dragonfly
    # P2W = 3600*P2W # Convert wh to J

    Pbatt = P*PfracBattery*1000
    mb = (Pbatt/P2W)*daytime/2 # Night time
    # if E == 0:
    #     mb = (Pbatt/P2W)*((R*1000)/v)
    # else:
    #     mb = (Pbatt/P2W)*E
    if math.isnan(mb): mb=0
    return mb

def Weight_Payload(Data):
    '''
    Inputs:
        If called, reads in "included" payload and sensors masses from ./Payload/Payload_and_Sensors.csv
    Outputs:
        mpay - Payload Mass [kg]

    Notes:
    - Will not read in "Battery" or "RTG" components
    - Components require "TRUE" in the Include col. to be included in the summation
    - Assumes FOS of 1.1
    
    '''
    df = pd.read_csv(os.path.join(ROOT_DIR,'Payload/Payload_and_Sensors.csv'))
    mpay = 0
    Nnan = df['Component'].isna().sum()
    Comp = df['Component'].to_numpy()
    Mass = df['Mass'].to_numpy()
    In = df['Include'].to_numpy()
    CNT = df['Count'].to_numpy()
    Len = Comp.size
    mpay = 0
    i=0
    while i < (Len-Nnan):
        if Comp[i] != 'RTG' and Comp[i] != 'Battery' and In[i] == True:
            mpay += Mass[i] * CNT[i]
        i += 1
    mpay = 1.1*mpay
    if math.isnan(mpay): mpay=0
    return mpay

def Therm_Prop_Payload(Data):
    '''
    Inputs:
    Outputs:
    '''
    df = pd.read_csv(os.path.join(ROOT_DIR,'Payload/Payload_and_Sensors.csv'))
    Nnan = df['Component'].isna().sum()
    In = df['Include'].to_numpy()
    #Placeholder
    A_equiv = 0
    Tref = 0
    return A_equiv,Tref

def Weight_Wing(Data):
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
        K   - Material Fudge Factor
    '''
    # Extract Data
    Sw = Data.Wing_Sw[0]
    AR = Data.Wing_AR[0]
    SWP = Data.Wing_Sweep[0]
    q = Data.q[0]
    tpr = Data.Wing_TR[0]
    tc = Data.Wing_TC[0]
    Nz = Data.Nz[0]
    mdg = Data.Wdg[0]
    g = Data.g[0]

    # Constants
    K = 0.85    # Material Fudge Factor
    ge = 9.80665 # Earth gravitational constant [m/s^2]
    lbf2N = 4.44822 # [lbf/N]
    N2lbf = 0.224809    # [N/lbf]
    m2ft = 3.28084  # [m/ft]
    psf2Pa = 0.020885434273039  # [psf/Pa]

    # Convert to Earth Reference
    Wdg = mdg*ge
    Nz = Nz*(g/ge)

    # Convert from Metric to Imperial Units
    Sw = m2ft*m2ft*Sw     # Wetted Area
    q = psf2Pa*q   #Dynamic Pressure
    Wdg = N2lbf*Wdg # Guess Total Weight

    # Wing Weight, based on Raymer Eq. 15.46
    Wwing = 0.036*K*np.power(Sw,0.758)*np.power(np.divide(AR,np.square(math.cos(SWP*np.pi/180))),0.6)*np.power(q,0.006)*np.power(tpr,0.04)*np.power(100*tc/math.cos(SWP*np.pi/180),-0.3)*np.power(Nz*Wdg,0.49)
    
    #Kdw = 1
    #Kvs = 1
    #Wwing2 = 0.0103*Kdw*Kvs*np.sqrt(Wdg*Nz)*np.power(Sw,0.622)*np.power(AR,0.785)*tc*np.power(1+tpr,0.05)*np.power(np.cos(SWP),-1)*np.power(Scsw,0.04)
    mwing = lbf2N*Wwing/ge
    if math.isnan(mwing): mwing=0
    return mwing

def Weight_Fuselage(Data):
    '''
    Inputs:
        Sfw - Wetted Area [m^2]
        Nz  - Ultimate Load Factor (1.5x Limit Load)
        Mdg - Flight Design Gross Weight [kg]
        Lt  - 
        Lf  - Fuselage Length [m]
        df  - Fuselage depth [m]
        q   - Dynamic Pressure [Pa]

    Outputs:
        mfuse - Fuselage Mass [kg]

    Assumptions:
        K   - Material Fudge Factor
    '''
    # Extract Data
    Sfw = Data.Fuse_Sw[0]
    Nz = Data.Nz[0]
    mdg = Data.Wdg[0]
    Lt = Data.Fuse_Lt[0]
    Lf = Data.Fuse_L[0]
    df = Data.Fuse_d[0]
    q = Data.q[0]
    g = Data.g[0]

    # Constants
    K = 0.9 # Material Fudge Factor
    ge = 9.80665 # Earth gravitational constant [m/s^2]
    lbf2N = 4.44822 # [lbf/N]
    N2lbf = 0.224809    # [N/lbf]
    m2ft = 3.28084  # [m/ft]
    psf2Pa = 0.020885434273039  # [psf/Pa]

    # Convert to Earth Reference
    Wdg = mdg*ge
    Nz = Nz*(g/ge)

    # Convert from Metric to Imperial Units
    Sfw = m2ft*m2ft*Sfw     # Wetted Area
    Lf = m2ft*Lf # Fuselage Length
    df = m2ft*df # Fuselage Depth
    Lt = m2ft*Lt
    q = psf2Pa*q   #Dynamic Pressure
    Wdg = N2lbf*Wdg # Guess Total Weight

    # Fuselage Weight based on Raymer Eq. 15.49
    if Lf != 0:
        Wfuse = 0.052*K*np.power(Sfw,1.086)*np.power(Nz*Wdg,0.177)*np.power(Lt,-0.051)*np.power(Lf/df,-0.072)*np.power(q,0.241)
        mfuse = lbf2N*Wfuse/ge
    else:
        mfuse = 0
    if math.isnan(mfuse): mfuse=0
    return mfuse

def Weight_FC(Data):
    '''
    Inputs:
        Lf  - Fuselage Length [m]
        b   - Wing Span [m]
        Nz  - Ultimate Load Factor (1.5x Limit Load)
        mdg - Flight Design Gross Mass [kg]
    Outputs:
        mfc - Flight Controls Mass [kg]
    '''
    # Extract Data
    Lf = Data.Fuse_L[0]
    b = Data.Wing_b[0]
    Nz = Data.Nz[0]
    Wdg = Data.Wdg[0]
    g = Data.g[0]

    # Constants
    ge = 9.80665 # Earth gravitational constant [m/s^2]
    lbf2N = 4.44822 # [lbf/N]
    N2lbf = 0.224809    # [N/lbf]
    m2ft = 3.28084  # [m/ft]
    psf2Pa = 0.020885434273039  # [psf/Pa]
    
    # Convert to Earth Reference
    Wdg = Wdg*ge
    Nz = Nz*(g/ge)
    # Convert from Metric to Imperial Units
    Lf = m2ft*Lf # Fuselage Length
    b = m2ft*b   # Wing Span
    Wdg = N2lbf*Wdg  # Guess Total Weight

    # Flight Controls Weight based on Raymer Eq. 15.54
    if Lf == 0:
        C_fuse = 1
    else:
        C_fuse = np.power(Lf,1.536)
    Wfc = 0.053*C_fuse*np.power(b,0.371)*np.power(Nz*Wdg*0.0001,0.8)
    mfc = lbf2N*Wfc/ge
    if math.isnan(mfc): mfc=0
    return mfc

def Weight_Stabilizer(Data):
    '''
    Inputs:
        S_Ht    - Horizontal Stabilizer Planform Area [m^2]
        S_Vt    - Vertical Stabilizer Planform area [m^2]
    Outputs:
        Wstab - Sum of Stabilizer Weights [N]

    Assumptions:
        S_empannage wetted area ~ 2(S_HT+S_VT)
        WA_emp ~ 0.5 lb/ft^2 for composites
    '''
    # Extract Data
    S_fin = 2*Data.Wing_fin_S[0]    # Wing-Tip Stabilizer Fin Area
    S_Ht = Data.Ht_S[0]             # Horizontal Stabilizer Area
    S_Vt = Data.Vt_S[0]             # Vertical Stabilizer area
    g = Data.g[0]

    # Constants
    ge = 9.80665 # Earth gravitational constant [m/s^2]
    lbf2N = 4.44822 # [lbf/N]
    m2ft = 3.28084  # [m/ft]
    WA_emp = 0.5    #[lb/ft^2]
    # Convert from Metric to Imperial Units
    S_Ht *= m2ft*m2ft
    S_Vt *= m2ft*m2ft
    S_fin *= m2ft*m2ft
    # Stabilizer Weights based on Gundlach Eq. 6.39
    Wstab = WA_emp*2*(S_Ht+S_Vt + S_fin)
    mstab = lbf2N*Wstab/ge
    if math.isnan(mstab): mstab=0
    return mstab

def Weight_WingFold(b, cr, ct, fold_loc):
    '''
    Inputs:
        b - wingspan
        fold_loc - Wing fold location
    '''
    c_fold = cr - ((cr-ct)/b)*fold_loc # Folding mechanism chord length
    eta_fold = c_fold/(b/2)
    Fs_TOGW = 0.5*(1 - (2/math.pi)*eta_fold*np.sqrt(1 - eta_fold**2)  - (2/math.pi)*np.arcsin(eta_fold)) # Ref: https://link.springer.com/article/10.1007/s00158-010-0612-9
    Wfold_TOGW = 0.07*Fs_TOGW
    return Wfold_TOGW

#Weight_WingFold(4.75,0.9481,0.3319,4.5)

def Weight_Propulsion(Data):
    '''
    Inputs:
        Pmm     - Max Motor Power [kW]
        Vmax    - Max Rated Motor Voltage [V]
        Nprop   - Number of Props
        Nblade  - Number Blades per Prop
        D       - Prop Diamter [m]
    Outputs:
        mprop - Propulsion Systems Weight [kg]

    W_prop = W_EPS = W_Motor + W_Gear + W_ESC + W_Prop

    Assumptions:
    W_Gear = 0
    F_ESC = 0.08 lb/kW
    K_Prop = 15 (assumed plastic/composite prop)
    Brushless Outrunner (F1 = 0.889, E1=-0.288, E2 = 0.1588)

    '''
    # Extract Data
    Preq = Data.P[0]           # Required Power [kW]
    Ppay = Data.PPay[0]/1000 if 'PPay' in Data.columns else 0
    Pmm = Data.Pm[0] if 'Pm' in Data.columns else Preq - Ppay

    Vmax = Data.Vm[0]           if "Vm"         in Data.columns else 30
    Nunit = Data.PropulsorN[0]  if "PropulsorN" in Data.columns else 1
    Nprop = Data.PropN[0]       if "PropN"      in Data.columns else 1
    Nblade = Data.PropBlade[0]  if "PropBlade"  in Data.columns else 2
    D = Data.PropD[0]
    g = Data.g[0]

    # Constants
    F_ESC = 0.08
    K_Prop = 15
    ge = 9.80665 # Earth gravitational constant [m/s^2]
    lbf2N = 4.44822 # [lbf/N]
    m2ft = 3.28084  # [m/ft]
    

    # Convert from Metric to Imperial Length Units
    D *= m2ft

    # W_Motor from Gundlach Eq. 6.3
    W_Motor = 0.889 * np.power((Pmm/Nunit),-0.288+1)*np.power(Vmax,0.1588)
    # Other weights from Gundlach Eq. 6.50-6.52, 6.54
    W_Props = K_Prop * Nprop*np.power(Nblade,0.391)*np.power(D*(Pmm/Nunit)*0.001/Nprop,0.782)
    W_Controller = F_ESC*(Pmm/Nunit)

    W_prop = Nunit*sum([W_Motor, W_Props, W_Controller])
    m_prop = lbf2N*W_prop/ge
    if math.isnan(m_prop): m_prop=0

    return m_prop

def Weight_Avionics(Data):
    '''
    Inputs:
        mdg     - Flight Design Gross Mass [kg]
    Outputs:
        mav - Avionics Weight [kg]

    Assumptions:
        MF_Avionics ~ 0.1
        MF_Margin ~ 0.1
    '''  
    # Extract Data
    mdg = Data.Wdg[0]
    # Constants
    W_Avionics = 0.1
    W_Margin = 0
    # Calculation
    MF_av = sum([W_Avionics,W_Margin])
    mav = MF_av*mdg
    if math.isnan(mav): mav=0

    return mav

def Weight_Heat(Data):
    '''
    Inputs:
        mdg     - Flight Design Gross Weight [kg]
    Outputs
        Wht - Heating component(s) and insulation weight

    Wht = .126*Wdg
    W_heat = W_thermalbatterystorage + W_RTGradiator + W_Wiring + W_RadiationShielding + W_AerogelInsulation
    '''
    # Extract Data
    mdg = Data.Wdg[0]
    #Calculation
    mht = .126*mdg
    if math.isnan(mht): mht=0

    return mht

def Weight_Therm2(Data):
    '''
    Assumptions:
        - Awall ~ fuselage wetted area
        - RTG efficiencies
        - negligible conduction except through main fuselage wall
    '''
    # Read in parameters from Case Data
    T_in = Data.Tp_mid[0]       # Internal temperature sensors and payload [K]
    Awall = Data.Fuse_Sw[0]       # Wetted Fueslage Area estimate [m^2]
    T_amb = np.mean(np.array([Data.Tmin[0],Data.Tmax[0]]))  # Mean ambient external temperature expected [K]
    Preq = Data.P[0]*1000          # Required Electrical Power [W]
    TY = Data.RTGfueltype[0]    # RTG Type
    d = Data.Fuse_d[0]          # DOE-defined fuselage diameter [m]
    PfracRTG = Data.PfracRTG[0]
    ## Define Constants ##
    # Conductivity
    k_Fuse = 0.04  #W/m.K - conductivity of fiberglass (Source: http://hyperphysics.phy-astr.gsu.edu/hbase/Tables/thrcn.html)
    k_ins = 0.013 #W/m•K - conductivity of aerogel (Source: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6225116/#:~:text=Aerogels%20have%20the%20lowest%20thermal,pressure%20%5B6%2C7%5D.)
    # Convection
    h_out = 4.4     # [W/(m^2*K)] - convection coefficient within fuselage
    h_in = 4.4      # [W/(m^2*K)] - convection coefficient outside fuselage
    # Density
    rho_ins = 50    #[kg/m^3] density of aerogel
    rho_Al = 2700   #[kg/m^3] density of aluminum
    # Other Constants
    sig = 5.670e-8  # Stefan-Bolzmann’s  W/m2-K4
    PI=math.pi  

    ## Thickness Assumptions ##
    t_Fuse = 0.003  # [m] Assumed fuselage wall thickness
    t_rad  =0.005   # [m] thickness of radiator fins

    # Read in RTG efficiency
    RTGdf = pd.read_csv(os.path.join(LOCAL_DIR,'RTG.int'),header = None)
    if TY==1:
        Type = 'SRG'
    else:
        Type = 'RTG'
    eff = RTGdf.loc[RTGdf[0] == 'Eff_'+Type][1].to_numpy()  # Efficiency of RTG
    # Calculate Thermal power output
    qRTG = Preq*((1/eff)-1)
    qRTG = qRTG[0]

    dT = -1
    qHeat = qRTG
    m_ins = 0
    m_rad = 0
    if qRTG != 0 and PfracRTG:   # If there is a non-zero power requirement
        while dT < 0:   # condition of Exterior > interior
            f1 = lambda T: (qRTG/Awall) - T_in*(h_in + sig*(T_in**3)) + h_in*T + sig*T**4       # Function for heat transfer to inner surface of fuselage wall
            fp1 = lambda T: h_in + 4*sig* T**3
            f2 = lambda T: (qRTG/Awall) + T_amb*(h_out + sig*(T_amb**3)) - h_out*T - sig*T**4   # Function for heat transfer between outer surface of fuselage and environment
            fp2 = lambda T: -h_in - 4*sig* T**3

            T_si = root(f1,method='newton',fprime=fp1, x0=T_amb,xtol = 0.01,maxiter = 150).root # Find temperature on interior surface of fuselage wall
            T_so = root(f2,method='newton',fprime=fp2, x0=T_amb,xtol = 0.01,maxiter = 150).root # Find temperature on exterior surface of fuselage wall
            tk_wall = -(Awall * (T_so-T_si))/qRTG                                               # [W/K] Thickness/Conductivity of wall
            t_ins = ((tk_wall) * (k_ins + k_Fuse) - (t_Fuse*k_ins))/k_Fuse                      # [m] Required thickness of insulation
            dT = (T_si-T_so)    # Temperature differential across fuselage wall
            if dT < 0:          # reduce fraction heat fed into the main fuselage
                if abs(dT) < 10:
                    qRTG *= 0.99
                elif abs(dT) < 100:
                    qRTG *= 0.92
                elif abs(dT) < 1000:
                    qRTG *= 0.75
                else:
                    qRTG *= 0.5
        q_excess = qHeat - qRTG # Excess waste heat not used to warm the fuselage interior
        if q_excess > 0:        # If nonzero excess waste heat, assume wast heat directly transfered from RTG/radiator fins to the environment
            f3 = lambda T: (qRTG/(2*PI*(d/2)**2)) + T_in*(h_out + sig*(T_in**3)) - h_out*T - sig*T**4    # Function for heat transfer between outer surface of RTG and environment
            fp3 = lambda T: -h_in - 4*sig* T**3
            T_RTG = root(f3,method='newton',fprime=fp3, x0=T_amb,xtol = 0.01,maxiter = 150).root        # [K] Estimated temperature of the RTG
            A = q_excess / (h_out*(T_RTG-T_amb) + sig*(T_RTG**4 - T_amb**4))                            # [m^2] Minimum area required to dump excess waste heat
            fuse_dim = Req_Fuse_Dim(Data) # RTG height
            h = fuse_dim["h_RTG"]                                                                 # [m] Estimated length of RTG
            Aexcess = A - (h*d*PI)                                                                      # [m^2] additional radiating area needed if RTG cylindrical surface exposed to atmosphere for wast heat rejection
            if Aexcess > 0:
                m_rad = (Aexcess*rho_Al*t_rad)  # Mass of Radiators
        m_ins = t_ins*Awall*rho_ins             # Mass of insulation
    m_therm = m_ins+m_rad                          # Total thermal mass
    return m_therm


def Weight_Thermal_Ctrl(Data):
    '''
    Inputs:
        Data - Dataframe of geometry, performance parameters, and fixed masses
    Outputs:
        Data - Dataframe of Masses [kg]
                Weight [Total, RTG, Battery, Payload, Empty, Wing, Fuselage, Flight Controls, Stabilizers, Propulsion, Avionics, Heating, Insulation, Radiators, Heaters]
    '''
    Preq = Data.P[0]*1000            # Required Electrical Power [W]
    TY = Data.RTGfueltype[0]    # RTG Type
    Tmid = Data.Tp_mid[0]
    Sfw = Data.Fuse_Sw[0]
    Tamb = (Data.Tmin[0]+Data.Tmax[0])/2
    d = Data.Fuse_d[0]
    PI=math.pi
    PfracRTG = Data.PfracRTG[0]

    #  weight of non-thermal control components 
    # ...

    # Input variables, these variables are not fixed as they can be changed.
    
    #the idea of multiplying the area and depth so I can find the amount of space that is occupied in one 
    #direction  (depth) by a two-dimensional object(area), however, this only works when the material has a uniform
    # depth throughout its area, from here we can find the weight by utilizng the volume that was calculated
    #by density*gravity
    A_wall = PI*(d/2)**2 # m^2
    A_ceiling = Sfw
    A_window = 0 # m^2
    H_wall = d # m
    H_ceiling = d # m
    t_wall = 0.003 # m
    t_ceiling = 0.004 # m
    t_window = 0.005 # m
    rho_insulation = 50 # kg/m^3
    rho_radiators = 2700 # kg/m^3 density of aluminum
    T_inside = Tmid # k
    T_outside = Tamb # k Initial value
    solarConstant = 15.2 # W/m^2 solar radiation constant for Titan
    absorbtivity = .7328 # arbitrary value
    emissivity = .5 # arbitrary value
    sigma = 5.670e-8 # Stefan-Bolzmann’s  W/m2-K4
    # Convection and conduction coefficients
    #these values can be changed, are not set
    h_conv_inside = 15 # W/m^2-K 
    h_conv_outside = 50 # W/m^2-K
    #k_wall = 205 # W/m-K
    k_ceiling = 0.04 # W/m-K
    k_window = 0.8 # W/m-K
    k_wall = 0.04  #W/m.K - conductivity of fiberglass (Source: http://hyperphysics.phy-astr.gsu.edu/hbase/Tables/thrcn.html)
    k_insulation = 0.013 #W/m•K - conductivity of aerogel (Source: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6225116/#:~:text=Aerogels%20have%20the%20lowest%20thermal,pressure%20%5B6%2C7%5D.)
    temperature_Titan= Tamb #kelvin
    Walls_area = 2*A_wall#2 * A_wall * H_wall 
    
    # outside temperature due to solar radiation
    T_outside_Kelvin = (solarConstant*(absorbtivity/emissivity)*(A_window/A_wall)/sigma)**(1/4) # K
    T_outside = T_outside_Kelvin 


    RTGdf = pd.read_csv(os.path.join(LOCAL_DIR,'RTG.int'),header = None)
    if TY==1:
        Type = 'SRG'
    else:
        Type = 'RTG'
    eff = RTGdf.loc[RTGdf[0] == 'Eff_'+Type][1].to_numpy()  # Efficiency of RTG
    # Calculate Thermal power output
    qRTG = Preq*((1/eff)-1)
    qRTG = qRTG[0]
    if qRTG !=0 and PfracRTG:
        fuse_dim = Req_Fuse_Dim(Data) # RTG height
        h = fuse_dim["h_RTG"]
        qRTG_S = qRTG * (Walls_area/(Walls_area + h*PI*d))
        qRTG -= qRTG_S
        f1 = lambda T: (qRTG/(h*PI*d)) - (T-T_inside)*h_conv_inside + sigma*(T_inside**4 -T**4)       # Function for heat transfer to inner surface of fuselage wall
        fp1 = lambda T: -h_conv_inside - 4*sigma* T**3
        f2 = lambda T: qRTG/(h*PI*(d+2*t_ceiling)) - (T-Tamb)*h_conv_outside + sigma*(Tamb**4 - T**4)  # Function for heat transfer between outer surface of fuselage and environment
        fp2 = lambda T: -h_conv_outside - 4*sigma* T**3
        T_inner = root(f1,method='newton',fprime=fp1, x0=Tamb,xtol = 0.01,maxiter = 150).root # Find temperature on interior surface of fuselage wall
        T_outside = root(f2,method='newton',fprime=fp2, x0=Tamb,xtol = 0.01,maxiter = 150).root # Find temperature on exterior surface of fuselage wall
        
        # heat loss through walls and ceiling
        q_wall = k_wall * Walls_area / t_wall * (T_inside - T_outside)  
        q_ceiling = k_ceiling * A_ceiling / t_ceiling * (T_inside - T_outside)
        q_conv_inside = h_conv_inside * (Walls_area + A_ceiling) * (T_inside - T_inner)
        q_conv_outside = h_conv_outside * (Walls_area + A_ceiling) * (T_inside - T_outside)
        
        # heat loss through windows
        q_window = k_window * A_window / t_window * (T_inside - T_outside) 
        
        #total heat loss
        Total_heat_loss_ = q_wall + q_ceiling + q_window + q_conv_inside + q_conv_outside
        
        tk_wall = -(A_wall * (T_outside-T_inner))/qRTG                                             # [W/K] Thickness/Conductivity of wall
        thickness_insulation = ((tk_wall) * (k_insulation + k_wall) - (t_wall*k_insulation))/k_wall                      # [m] Required thickness of insulation
        if thickness_insulation < 0:
            thickness_insulation = 0
        

        #Insulation_area = Walls_area + Ceiling_area
        Insulation_area = A_ceiling - (PI*d*h)
        #thickness_insulation = 0.02 # m can be changed
        insulation_weight = Insulation_area * thickness_insulation * rho_insulation

    # radiator weight
        #f3 = lambda T: (qRTG/(Walls_area + h*PI*d)) - h_conv_inside*(T-T_inside) - sigma*(T**4-T_inside**4)    # Function for heat transfer between outer surface of RTG and environment
        #fp3 = lambda T: -h_conv_inside - 4*sigma* T**3
        #T_RTG = root(f3,method='newton',fprime=fp3, x0=Tamb,xtol = 0.01,maxiter = 150).root        # [K] Estimated temperature of the RTG
        T_RTG = T_inner
        Area_req = qRTG/(h_conv_outside*(T_RTG-Tamb) + sigma*(T_RTG**4-Tamb**4))
        Radiators_area = 0
        if Area_req > PI*h*d:
            Radiators_area = Area_req - PI*h*d
            print("Radiators_area",Radiators_area)
        #Radiators_area = Walls_area + Ceiling_area 
        radiators_thickness = 0.05 # m
        radiators_weight = Radiators_area * radiators_thickness * rho_radiators


    #Conduction heat transfer from RTG
        k_conduct = 0.04  #W/m.K - conductivity of fiberglass (Source: http://hyperphysics.phy-astr.gsu.edu/hbase/Tables/thrcn.html)
        # k_conduct = 0.013 #W/m•K - conductivity of aerogel (Source: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6225116/#:~:text=Aerogels%20have%20the%20lowest%20thermal,pressure%20%5B6%2C7%5D.)
        r_cond = 0.5 # m - distance between RTG and the nose of the aircraft
        A_cond = 4 * PI * r_cond**2
        Q_cond = k_conduct * A_cond * (temperature_Titan - Tmid) / r_cond

    #Convection heat transfer 
        h_conv = 4.4 #W/m^2.K - convection coefficient 
        A_conv = PI * (d/2)**2
        Q_conv = h_conv * A_conv * (temperature_Titan - Tamb)

    #Conduction heat transfer for FC-Servo
        k_wall = 0.04  #W/m.K - conductivity of fiberglass (Source: http://hyperphysics.phy-astr.gsu.edu/hbase/Tables/thrcn.html)
        k_insulation = 0.013 #W/m•K - conductivity of aerogel (Source: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6225116/#:~:text=Aerogels%20have%20the%20lowest%20thermal,pressure%20%5B6%2C7%5D.)
        #t_insulation = 0.02 # m - thickness of aerogel insulation
        #t_wall = .0625 # m - thickness of fiberglass wall
        diameter_servo_housing = .1 # m
        kwallOvertwall = (k_insulation*k_wall)/(k_insulation*thickness_insulation+k_wall*t_wall)
        A_wall_servo = PI * (diameter_servo_housing/2)**2
        Q_cond_servo = kwallOvertwall * A_wall_servo * (temperature_Titan - Tmid)

    #Convection heat transfer for FC-Servo
        h_conv = 4.4 #W/m^2.K - convection coefficient
        diameter_servo_housing = .1 # m
        A_wall_servo = PI * (diameter_servo_housing/2)**2
        Q_conv_servo = -h_conv * A_wall_servo * (temperature_Titan - Tmid)
    #Radiation heat transfer for FC-Servo
        epsilon_insulation = .9 # aerogel insulation emissivity
        Q_rad_servo = -epsilon_insulation * sigma * A_wall_servo * (Tamb ** 4-Tmid**4) # servo radiation heat transfer
    # Radiator Heat required for FC-Servo
        #radiator_heat_required = Q_cond_servo + Q_conv_servo
        radiator_heat_required = (Q_conv_servo + Q_rad_servo)
        
        insulation_weight += thickness_insulation*A_wall_servo*rho_insulation
        #print("Total heat loss for each FC-Servo:", radiator_heat_required, "W")
        # Each servo require ~8 W, which is satisfied by one 500 W heater 

        # heater weight. I just assumed 500 W heaters
        heaters_weight = radiator_heat_required * (0.77/500)    #from 500 W heater with 0.77kg mass   
    # Total Heat Generation
        Q_total = Q_conv + Total_heat_loss_

        #print("Total heat generation:", Q_total, "W")
        #print("Conduction heat transfer:", Q_cond, "W")
        #print("Convection heat transfer:", Q_conv, "W")


    # Total Weight
        #total_weight = mbatt + mRTG + mpay + mwing + mfuse ++ insulation_weight + radiators_weight + heaters_weight

        #Data.loc[0,'Weight_RTG'] = mRTG
        #Data.loc[0,'Weight_Battery'] = mbatt
        #Data.loc[0,'Weight_Payload'] = mpay
        #Data.loc[0,'Weight_Wing'] = mwing
        #Data.loc[0,'Weight_Fuselage'] = mfuse
        Data.loc[0,'Weight_Insulation'] = insulation_weight
        Data.loc[0,'Weight_Radiators'] = radiators_weight
        Data.loc[0,'Weight_Heaters'] = heaters_weight
        Data.loc[0,'P'] += radiator_heat_required/1000 - Data.PHeat[0]
        Data.loc[0,'PHeat'] = radiator_heat_required/1000
        #Data.loc[0,'Weight_Total'] = total_weight
    else:
        heaters_weight = 0
        radiators_weight = 0
        insulation_weight = 0
    m_therm = heaters_weight + radiators_weight + insulation_weight
    return m_therm, insulation_weight,radiators_weight,heaters_weight,Data


def Req_Fuse_Dim(Data):
    '''
    Inputs:
        P           - Required Power[kW]
        D           - Fuselage Diameter [m]
        T           - RTG Type
    Outputs:
        Array       - Array of Dimensions
            - V     - Fuselage Volume [m^3]
            - h     - Cylindrical RTG height [m]
            - h2    - Cylindrical non-RTG height [m]
            - L     - Fuselage Length [m]
    Assumptions:
        - Cylindrical fuselage with spherical end caps
        - 50% Factor of Safety on fuselage volume
        - Regression on RTG & SRG Volume/Surface Area to thermal power output
    '''
    # Extract Data
    PfracRTG = Data.PfracRTG[0]
    PfracBattery = Data.PfracBattery[0]
    P = Data.P[0]*1000
    output = {}
    # Constants
    #C = [0.5441,53.076]  # [m/w^2, m/W]  regression for RTG Side Area/Electrical Power as function of Electrical Power, (A/P) = C0*P + C1
    C = [0.0025,0.7233]

    df = pd.read_csv(os.path.join(ROOT_DIR,'Payload/Payload_and_Sensors.csv'))
    Nnan = df['Component'].isna().sum()
    Comp = df['Component'].to_numpy()
    #Mass = df['Mass'].to_numpy()
    Volume = df['Volume'].to_numpy()
    In = df['Include'].to_numpy()
    CNT = df['Count'].to_numpy()
    Len = Comp.size
    Vpay = 0
    i=0
    while i < (Len-Nnan):
        if Comp[i] != 'RTG' and Comp[i] != 'Battery' and In[i] == True:
            Vpay += Volume[i] * CNT[i]
        i += 1
    ### Calculations ###
    # Estimate RTG Volume
    hrtg = 0
    Drtg = 0
    nRTG = 0
    if PfracRTG != 0:
        # hrtg = (C[0]*P*PfracRTG + C[1])/(math.pi*D)
        nRTG = max(round(P*PfracRTG/150,0),1)
        hrtg = nRTG*0.0254*(0.0629*P*PfracRTG/nRTG + 26.241) #RTG Length regression based off rover RTG's
        Drtg = 0.0254*(0.0437*P*PfracRTG/nRTG + 19.512) #RTG Diameter regression based off rover RTG's
        D = Drtg + 0.1*Drtg
    else:
        D = 0.4
    Vrtg = hrtg*math.pi * (Drtg/2)**2

    # Define fuselage diameter to be 10% bigger than RTG diameter

    # Battery calculations
    hbat = 0
    daytime = Planet_Const(Data.Planet[0])[2]
    if PfracBattery!= 0:
        hbat = (PfracBattery*P*daytime/2/400/1000)/(math.pi*D) # Assuming energy density of 400 Wh/L -> 400,000 Wh/m3
    Vbat = PfracBattery*P*daytime/2/400/1000
    V = 1.1*(Vpay+Vrtg+Vbat) # [m^3], required volume
    h2 = (4*V/(math.pi*D**2)) - (2*D/3) - hbat - hrtg
    if h2 < 0:
        h2 = 0
    hnet = hbat + h2 + hrtg
    L = hnet
    if D/L > 0.3: # If the t/c > 0.3 find what the fuselage length needs to be in order to keep it at 0.3. 
        L = D/0.3

    output["V"] = V
    output["h_RTG"] = hrtg
    output["n_RTG"] = nRTG
    output["h_avionics"] = h2
    output["h_bat"] = hbat
    output["Fuse_L"] = L
    output["Fuse_d"] = D

    return output

def Position_Long_Stable(Data,mset,cg_set,SM,npt):
    '''
    Inputs:
        Data    - Dataframe of geometry, performance parameters, and fixed masses
        mset    - Dataframe of vehicle masses
        cg_set  - Dataframe of known cg positions
        SM      - Static Margin
        npt     - Neutral Point
    Outputs:
        cg_set  - Updated dataframe with x position of 'Other' component(s) added

    Assumption:
        - Datum at wing leading edge
    '''
    # Import Parameters
    AR = Data.Wing_AR[0]    # Wing Aspect Ratio
    S = Data.Wing_S[0]      # Wing Planform Area [m^2]
    TR = Data.Wing_TR[0]    # Wing Taper Ratio
    mwing = mset.Wing[0]    # Wing mass [kg]
    cg_wing = cg_set.Wing[0]    # Wing cg position [m]
    mnet = mset.Total[0]    # Total vehicle mass [kg]

    #Initial calculations
    m_other = mnet - mwing
    b = np.sqrt(AR*S)           # wing span
    cr = (2*b)/(AR*(1+TR))      # wing root chord
    ct = (2*b*TR)/(AR*(1+TR))   # wing tip chord
    mac = (2/3)*(cr+ct-(cr*ct)/(cr+ct)) # Wing MAC

    # Evaluate desired cg_x position
    cg = npt  - SM*mac
    # Evaluate and save position for "other component"
    cg_other = Comp_Place(m_other,mwing,cg,cg_wing)
    cgy = cg_set.Wing[1] * mwing/mnet
    cgz = cg_set.Wing[2] * mwing/mnet
    cg_set = pd.concat([cg_set, pd.DataFrame({'Other':[cg_other,0,0],'Net':[cg,cgy,cgz]})],axis=1)
    return cg_set

def Comp_Place(m1,m2,xcg,xcg_m2):
    '''
    Inputs:
        m1      - mass of component being placed
        m2      - mass of rest of system
        xgc     - cg position for whole system
        xcg_m2  - cg position ofl rest of system
    Outputs:
        xcg_m1  - position of component being placed
    '''
    xcg_m1 = (xcg*(m1+m2)-m2*xcg_m2)/m1
    return xcg_m1

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Testing



#d1 = {'Wdg': [100], 'g': [1.352], 'Nz': [0], 'q': [0], 't':[0],'P': [0.12], 'Vmax': [0], 'PropN': [0], 'PropBlade': [0], 'PropD': [0]}
#d2 = {'Ht_S': [0], 'Vt_S': [0]}
#d3 = {'Fuse_Sw': [0], 'Fuse_L': [0], 'Fuse_Lt': [0], 'Fuse_d': [0]}
#d4 = {'Wing_b': [0], 'Wing_Sw': [0], 'Wing_AR': [0], 'Wing_SWP': [0], 'Wing_TR': [0], 'Wing_TC': [0]}
#d = {'Batt': [100],'Pay': [100], 'P': [0.5], 't': [23], 'g':[1.352], 'RTGfuelfrac': [0.05], 'RTGfueltype': [0]}
#set = pd.DataFrame(data=d)
#print(set)
#RTG = Prelim_Weights(set)
#print(RTG)

#d = {'Batt': [100],'Pay': [100], 'P': [0.5], 't': [23], 'g':[1.352], 'RTGfuelfrac': [0.05], 'RTGfueltype': [1]}
#set = pd.DataFrame(data=d)
#print(set)
#RTG = Prelim_Weights(set)
#print(RTG)
