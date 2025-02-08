import pandas as pd
import numpy as np
from math import pi as PI
from Environment.TitanAtm import *
from Velocity_Area_Calc import *
from Config_generation import *

def Case_generator(input):
    # Environment
    planet = input.Planet[0]
    atm_table = planet
    planetConstants = Planet_Const(planet)
    g0 = planetConstants[0]
    DIAMETER = planetConstants[1]

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
    seas = input.Season[0]

    def selected_planet(ncases):
        planetArray = []
        for i in range(ncases):
            planetArray.append(input.Planet[0])
        return np.array(planetArray)
    planetArray = selected_planet(ncases)

    # Create drbatt range
    def drbatt_range(ncases):
        drbatt = np.zeros(ncases)
        return drbatt
    drbatt = drbatt_range(ncases)

    # Create dtbatt range
    def dtbatt_range(ncases):
        # dtbatt = np.zeros(ncases)
        dtbatt = np.ones(ncases)*365*24 # 1 year in hour
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
    FOS = FOS_range(input.FOS[0],ncases)

    # Fuse_L range
    def Fuse_L_range(ncases):
        Fuse_L = np.ones(ncases)
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
            NACA0.append(False)
            i = i + 1
        return NACA0
    NACA0 = NACA0_range(ncases)

    #NACA0 range
    def NACA1_range(ncases):
        NACA1 = []
        i = 0
        for loop in range(i, ncases):
            NACA1.append(False)
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
    Panels = Panels_range(input.Panels[0],ncases)

    # PassN range
    def PassN_range(ncases):
        PassN = np.linspace(10,10,ncases)
        return PassN
    PassN = PassN_range(ncases)

    # Pfrac range
    def RTGFuelFrac_range(ncases):
        Pfrac = np.zeros(ncases)
        return Pfrac

    def Pdistrib(ncases):
        array = np.ones(ncases)
        if planet == "Earth":
            PfracSpanels = array*0.8
            PfracBattery = array*0.2
        if planet == "Mars":
            PfracSpanels = array*0.7
            PfracBattery = array*0.2
        if planet == "Titan":
            PfracSpanels = array*0.0
            PfracBattery = array*0.0
        PfracRTG = np.round(1 - PfracSpanels - PfracBattery,2)
        return PfracSpanels, PfracRTG, PfracBattery
    PfracSpanels, PfracRTG, PfracBattery = Pdistrib(ncases)

    # PropBlade range
    def PropBlade_range(ncases):
        PropBlade = np.linspace(3,3,ncases)
        return PropBlade
    PropBlade = PropBlade_range(ncases)

    # PropD range This is just a for the initial iteration. We can add in to the prop loop to select less or more thrusters based off diameter constraints and whatnot
    def PropD_range(ncases):
        PropD = np.linspace(0.37,0.37,ncases)
        return PropD
    PropD = PropD_range(ncases)

    # PropN range
    def PropN_range(ncases):
        PropN = np.linspace(2,2,ncases)
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
    rho_b = rho_range(input.rho_b[0],ncases)
    rho_s = rho_b

    #Vehicle type range
    def VehType_range(input,ncases):
        type = []
        i = 0
        for loop in range(i, ncases):
            type.append(input)
            i = i + 1
        return type
    Type = VehType_range(input.Type[0],ncases)

    # Misc. DoE variables
    # Making a ones vector
    ones = np.linspace(1,1,ncases)
    RTGfuelfrac = RTGFuelFrac_range(ncases)
    RTGfueltype = ones
    tmission = ones
    tstart = input.tstart[0]*tmission
    Ult_n = input.Ult_n[0]*ones
    Ult_s = input.Ult_s[0]*ones
    Vm = input.Vm[0]*ones
    Cruise_Alt = CruiseAlt*ones
    Min_Alt = MinAlt*ones
    Cruise_V = CruiseV*ones
    RoC_min_alt = RoCminAlt*ones
    RoC_max_alt = RoCmaxAlt*ones
    Accel = Accel*ones
    num_Folds = num_Folds*ones
    Fuse_D = 0.42*ones
    Wing_TR = 0.35*ones
    Wing_fin_S = 0*ones
    Wing_fin_TR = 0*ones
    RoC_max_alt = RoCmaxAlt*ones
    RoC_min_alt = RoCminAlt*ones
    season = [seas for _ in range(ncases)]

    # Create a range of weights based off launch vehicle constraints
    def Weight_Range(W_inp,ncases):
        g0 = planetConstants[0]
        W = np.linspace(W_inp*0.25,W_inp,ncases)
        return W
    W = Weight_Range(W_inp,ncases)

    # Using the launch vehicle weight constraints, calculate a range of wing areas 
    def Wing_S(W,V):
        rho_cruise = ATM_Table(CruiseAlt, atm_table)[0]
        CL = 0.55 # Lift coefficient assumption
        S = (2*W)/(rho_cruise*(V**2)*CL)
        return S
    S = Wing_S(W,CruiseV)

    # Calculate range of aspect ratios
    def Wing_AR():
        AR = np.linspace(5,17,ncases)
        return AR
    AR = Wing_AR()

    # Calculate range of wingspans based off aeroshell constraints
    def Wingspan(AR,S):
        b = np.sqrt(AR*S)
        return b
    b = Wingspan(AR,S)

    # Calculate range of wing sweeps based off cruise velocity
    def Wing_Sweep(V): # Need to model the data better. It estimates too high
        a = ATM_Table(CruiseAlt, atm_table)[3]
        M = V/a
        Sweep = 3.7745*(M**3)-30.4220*(M**2)+83.4788*M-20.1291
        Sw_W = np.linspace(Sweep,Sweep,ncases)
        if Sw_W[0] < 0:
            Sw_W = np.zeros(len(n_cases))
        return(Sw_W) 
    Sw_W = Wing_Sweep(CruiseV)

    # Calculate the required change in lift per unit span coefficient wrto change in angle of attack (Airfoil lift curve slope)
    def airfoil(W,AR,V,S): # Need to edit so it can select an airfoil
        e = 0.85
        aoa_0 = -2
        aoa = 2
        rho_cruise = ATM_Table(CruiseAlt, atm_table)[0]
        CL = (2*W)/(rho_cruise*(V**2)*S)
        Cla = -CL/((CL/(PI*e*AR)) - (aoa - aoa_0))
        return Cla 
    Cla_need = airfoil(W,AR,CruiseV,S)

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
        SM = 0.07
        Cma = -SM*CLaw
        XbNP = SM + XbCG
        dah_da = 0.7 # downwash assumption. Need to find a better way to do it 
        Sh = (XbNP*CLaw - CLaw*Xbacw)/(etah*(1/S)*CLah*dah_da*Xbach - XbNP*etah*(1/S)*CLah*dah_da*Xbach)
        return Sh
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
        ARh = 1.65
        CLav = (0.1/(1+ 0.1/(PI*e*ARh)))*(180/PI)
        Sv = (CnBv*S)/(lv*etav*CLav*(1-dsig_dB))
        return Sv
    Sv = Vtail_S(S,b,input.lb[0],Wing_TR[0])

    # Calculate the required range of wing dihedral angles
    def wing_dihedral(ncases,Cla,AR,tr):
        ClBG = np.linspace(-0.0415,-0.0829,ncases)
        e = 0.85
        CLaw = Cla/(1 + Cla/(PI*e*AR))
        dih_wing = (-6*ClBG/CLaw)*((1+tr)/(1+2*tr))
        return dih_wing
    dih_wing = wing_dihedral(ncases,Cla_need,AR,Wing_TR[0])

    # Configuration generation
    dfx = config_gen()

    # Read the DOE.csv file
    dF = pd.read_csv('Inp/DOE.csv')
    dF = pd.DataFrame(columns=dF.columns) # Clear the dataframe so that the new cases aren't appended to the old cases
    df = pd.read_csv('Inp/DOE.csv')
    df = pd.DataFrame(columns=df.columns) # Clear the dataframe so that the new cases aren't appended to the old cases
    # Create DOE with configurations
    for i in range(0, dfx.shape[0]):
        for j in range(0,ncases):
        # Update the columns with the new data
            df['CASE'] = pd.Series(n_cases)
            df['drbatt'] = pd.Series(drbatt)
            df['dtbatt'] = pd.Series(dtbatt)
            df['EQ_W_Wing'] = pd.Series(EQ_W_Wing)
            df['EQ_W_Therm'] = pd.Series(EQ_W_Therm)
            df['Foil0'] = pd.Series(Foil0)
            df['Foil1'] = pd.Series(Foil1)
            df['FOS'] = pd.Series(FOS)
            df['Fuse_d'] = pd.Series(Fuse_D)
            df['Fuse_L'] = pd.Series(Fuse_L)
            df['Fuse_Lt'] = pd.Series(Fuse_Lt)
            df['Ht_S'] = pd.Series(Sh)
            df['NACA0'] = pd.Series(NACA0)
            df['NACA1'] = pd.Series(NACA1)
            df['Nz'] = pd.Series(Nz)
            df['Panels'] = pd.Series(Panels)
            df['PassN'] = pd.Series(PassN)
            df['PfracBattery'] = pd.Series(PfracBattery)
            df['PfracRTG'] = pd.Series(PfracRTG)
            df['PfracSpanels'] = pd.Series(PfracSpanels)
            df['Planet'] = pd.Series(planetArray)
            df['PropBlade'] = pd.Series(PropBlade)
            df['PropD'] = pd.Series(PropD)
            df['PropN'] = dfx.iloc[i,1]
            df['PropulsionN'] = dfx.iloc[i,1]
            df['rho_b'] = pd.Series(rho_b)
            df['rho_s'] = pd.Series(rho_s)
            df['RTGfuelfrac'] = pd.Series(RTGfuelfrac)
            df['RTGfueltype'] = pd.Series(RTGfueltype)
            df['tstart'] = pd.Series(tstart)
            df['tmission'] = pd.Series(tmission)
            df['Type'] = pd.Series(Type)
            df['Tail_type'] = dfx.iloc[i,0]
            df['Winglet'] = dfx.iloc[i,2]
            df['Ult_n'] = pd.Series(Ult_n)
            df['Ult_s'] = pd.Series(Ult_s)
            df['Vm'] = pd.Series(Vm)
            df['Vt_S'] = pd.Series(Sv)
            df['Wing_AR'] = pd.Series(AR)
            df['Wing_dhl'] = pd.Series(dih_wing)
            df['Wing_S'] = pd.Series(S)
            df['Wing_Sweep'] = pd.Series(Sw_W)
            df['Wing_TR'] = pd.Series(Wing_TR)
            df['Wing_fin_S'] = pd.Series(Wing_fin_S)
            df['Wing_fin_TR'] = pd.Series(Wing_fin_TR)
            df['Cruise_Alt'] = pd.Series(np.linspace(CruiseAlt,CruiseAlt,ncases))
            df['Min_Alt'] = pd.Series(Min_Alt)
            df['Cruise_V'] = pd.Series(np.linspace(CruiseV,CruiseV,ncases))
            df['RoC_min_alt'] = pd.Series(RoC_min_alt)
            df['RoC_max_alt'] = pd.Series(RoC_max_alt)
            df['Accel'] = pd.Series(Accel)
            df['Wing_folding'] = dfx.iloc[i,3]
            df['Num_Folds'] = pd.Series(num_Folds)
            df['Season'] = pd.Series(season)
            df['Sensor_swath_width'] = pd.Series(input.Sensor_swath_width[0])
            df['Area_coverage'] = pd.Series(input.Area_coverage[0])
            df['D_shell'] = pd.Series(input.D_shell[0])
        dF_cleaned = dF.dropna(how='all').dropna(axis=1, how='all')
        df_cleaned = df.dropna(how='all').dropna(axis=1, how='all')
        dF = pd.concat([dF_cleaned, df_cleaned])

    # Write data to DOE.csv
    dF.to_csv('Inp/DOE.csv', index=False)
    return df