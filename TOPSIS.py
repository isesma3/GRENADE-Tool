import os
import pandas as pd

ROOT_DIR = os.path.dirname(__file__)

def TOPSIS(input,df,CASE_END):
    import pandas as pd
    import numpy as np
    # ROOT_DIR = os.path.dirname(__file__)

    # input = pd.read_csv(os.path.join(ROOT_DIR, "Inp/Input.csv"))
    # DOE = pd.read_csv(os.path.join(ROOT_DIR, "Inp/DOE.csv"))

    # CASE_START = 1
    # CASE_END = int(input.n_Cases[0])

    # # Read in output file 
    # out = pd.read_csv(os.path.join(ROOT_DIR, "Out/Compiled" + str(CASE_START) + "-" + str(CASE_END) + ".csv"))

    # # Read in the TOPSIS input file
    # TOPSISinput = pd.read_csv(os.path.join(ROOT_DIR, "Inp/TOPSIS_Input.csv"))
    # # Read in metric arrays
    # CalcDF = pd.DataFrame(columns=TOPSISinput['Metric'].unique())
    # for metric in TOPSISinput['Metric'].unique():
    #     CalcDF[metric] = (TOPSISinput['Metric'] == metric).astype(int)
    # CalcDF = CalcDF.reindex(df.index)
    # for i, row in TOPSISinput.iterrows():
    #     metric = row['Metric']
    #     index = df.columns.get_loc(metric)
    #     CalcDF[metric] = df.iloc[:, index]

    # column_arrays = {}
    # for column_name in CalcDF.columns:
    #     # Extract values of the column into an array
    #     column_values = CalcDF[column_name].values
    #     # Store the array in the dictionary
    #     column_arrays[column_name] = column_values
    
    b = df.Span
    m = df.Mass
    P = df.Power
    LD = df['L/D']
    #mRTG = df.mRTG
    PropN = df.PropulsionN
    # Num_fold = df.Num_wing_fold


    # Sum of squares
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
    LD1 = []
    #mRTG1 = []
    PropN1 = []
    #Num_fold1 = []
    for i in range(CASE_END):
        b1.append(b[i] / ssq2(b))
        m1.append(m[i] / ssq2(m))
        P1.append(P[i] / ssq2(P))
        LD1.append(LD[i] / ssq2(LD))
        PropN1.append(PropN[i] / ssq2(PropN))
        #Num_fold1.append(Num_fold[i] / ssq2(Num_fold))
        # if ssq2(mRTG) != 0:
        #     mRTG1.append(mRTG[i] / ssq2(mRTG))
        # else:
        #     mRTG1.append(0)
    # Multiply by the weights
    b_weighted = [val * float(input.b_weight[0]) for val in b1]
    m_weighted = [val * float(input.m_weight[0]) for val in m1]
    P_weighted = [val * float(input.P_weight[0]) for val in P1]
    LD_weighted = [val * float(input.LD_weight[0]) for val in LD1]
    #mRTG_weighted = [val * float(input.mRTG_weight[0]) for val in mRTG1]
    PropN_weighted = [val * float(input.PropN_weight[0]) for val in PropN1]
    #Num_fold_weighted = [val * float(input.Num_fold_weight[0]) for val in Num_fold1]

    # Apply the criterion
    def pos(x,c):
        if c == 'cost':
            y = np.min(x)
        else:
            y = np.max(x)
        return y

    import numpy as np

    def neg(x, c):
        if c == 'cost':
            y = np.max(x)
        else:
            y = np.min(x)
        return y
    
                
    # Seperations
    i = 0
    Sip = []
    Sin = []
    for i in range(CASE_END):
        Sip.append(np.sqrt((b_weighted[i] - pos(b_weighted,input.b_weight[1]))**2 + (m_weighted[i] - pos(m_weighted,input.m_weight[1]))**2 
                           + (P_weighted[i] - pos(P_weighted,input.P_weight[1]))**2 + (LD_weighted[i] - pos(LD_weighted,input.LD_weight[1]))**2 
                           + (PropN_weighted[i] - pos(PropN_weighted,input.PropN_weight[1]))**2))
                           #'''+ (Num_fold_weighted[i] - pos(Num_fold_weighted,input.Num_fold_weight[1]))**2 + (mRTG_weighted[i] - pos(mRTG_weighted,input.mRTG_weight[1]))**2))'''
        
        Sin.append(np.sqrt((b_weighted[i] - neg(b_weighted,input.b_weight[1]))**2 + (m_weighted[i] - neg(m_weighted,input.m_weight[1]))**2 
                           + (P_weighted[i] - neg(P_weighted,input.P_weight[1]))**2 + (LD_weighted[i] - neg(LD_weighted,input.LD_weight[1]))**2
                           + (PropN_weighted[i] - neg(PropN_weighted,input.PropN_weight[1]))**2))
                           #+ (Num_fold_weighted[i] - neg(Num_fold_weighted,input.Num_fold_weight[1]))**2 + (mRTG_weighted[i] - neg(mRTG_weighted,input.mRTG_weight[1]))**2))

    # Closeness values
    C = [] 
    for i in range(CASE_END):
        C.append(Sin[i]/(Sip[i]+Sin[i]))

    # Determine the max closeness value and what case 
    A = np.max(C)
    Case_num = C.index(A)
    
    custom = 0
    if custom == 0:
        Tdf = pd.DataFrame(
            data={
                "Case": df.Case[Case_num],
                "Planform": df.Planform[Case_num],
                "Span": df.Span[Case_num],
                "Wing_Sweep": df.Wing_Sweep[Case_num],
                'Wing_AR':  df.Wing_AR[Case_num],
                'Wing_dhl': df.Wing_dhl[Case_num],
                'Wing_TR': df.Wing_TR[Case_num],
                'Wing_Twist': df.Wing_Twist[Case_num],
                "Mass": df.Mass[Case_num],
                "Power": df.Power[Case_num],
                "PowerConstraint": df.PowerConstraint[Case_num],
                "AreaCoverage": df.AreaCoverage[Case_num],
                "Airspeed": df.Airspeed[Case_num],
                "CD": df.CD[Case_num],
                "CL": df.CL[Case_num],
                "L/D": df.CL[Case_num]/df.CD[Case_num],
                "AoA": df.AoA[Case_num],
                "Fuse_L": df.Fuse_L[Case_num],
                "Fuse_D": df.Fuse_D[Case_num],
                "Fuse_x": df.Fuse_x[Case_num],
                "S_VTail": df.S_VTail[Case_num],
                "S_HorTail": df.S_HorTail[Case_num],
                "S_VerTail": df.S_VerTail[Case_num],
                "cgx": df.cgx[Case_num],
                "RTG_h": df.RTG_h[Case_num],
                "n_RTG": df.n_RTG[Case_num],
                "Batt_h": df.Batt_h[Case_num],
                "PropulsionN": df.PropulsionN[Case_num],
                "PropD": df.PropD[Case_num],
                "PropBlade": df.PropBlade[Case_num],
                "Check": df.Check[Case_num],
                "TailCheck": df.TailCheck[Case_num],
                "Planet": df.Planet[Case_num],
                "mTotal": df.mTotal[Case_num],
                "mRTG": df.mRTG[Case_num],
                "mBattery": df.mBattery[Case_num],
                "mPayload": df.mPayload[Case_num],
                "mEmpty": df.mEmpty[Case_num],
                "mWing": df.mWing[Case_num],
                "mFuselage": df.mFuselage[Case_num],
                "mFC": df.mFC[Case_num],
                "mStabilizer": df.mStabilizer[Case_num],
                "mProp": df.mProp[Case_num],
                "mAvionic": df.mAvionic[Case_num],
                "mInsulation": df.mInsulation[Case_num],
                "mRadiator": df.mRadiator[Case_num],
                "mHeater": df.mHeater[Case_num],
                "mSpanels": df.mSpanels[Case_num],
                "mWingFold": df.mWingFold[Case_num],
                "Tail_type": df.Tail_type[Case_num],
                "Winglet": df.Winglet[Case_num],
                "Wing_folding": df.Wing_folding[Case_num],
                "Num_wing_fold": df.Num_wing_fold[Case_num],
                "t_mission": df.t_mission[Case_num],
                "Sensor_Swath_width": df.Sensor_Swath_width[Case_num],
                "Aeroshell_Diameter": df.Aeroshell_Diameter[Case_num]
            },
            index=[i for i in range(1)],
        )
    else:
        Tdf = pd.DataFrame(
            data={
                "Case": df.Case[Case_num],
                "Planform": df.Planform[Case_num],
                "Span": df.Span[Case_num],
                'Wing_AR':  df.Wing_AR[Case_num],
                'Wing_dhl': df.Wing_dhl[Case_num],
                "Mass": df.Mass[Case_num],
                "Power": df.Power[Case_num],
                "L/D": df.loc[Case_num, 'L/D'],
                "Fuse_L": df.Fuse_L[Case_num],
                "Fuse_D": df.Fuse_D[Case_num],
                "PropulsionN": df.PropulsionN[Case_num],
                "Tail_type": df.Tail_type[Case_num],
                "Winglet": df.Winglet[Case_num],
                "t_mission": df.t_mission[Case_num],
                "Sensor_Swath_width": df.Sensor_Swath_width[Case_num],
                "Aeroshell_Diameter": df.Aeroshell_Diameter[Case_num]
            },
            index=[i for i in range(1)],
        )
    return Tdf,Case_num


# Use to postprocess data even after an error occurs
# df = pd.read_csv(os.path.join(ROOT_DIR, "Out/180_mass_10_SW_4m_AD.csv"))
# input = pd.read_csv(os.path.join(ROOT_DIR, "Inp/Input.csv"))
# CASE_END = df.shape[0]
# Tdf,Case_num = TOPSIS(input,df,CASE_END)
# print(df['Case'][Case_num])
# print(Tdf)
# TOPSISinput = pd.read_csv(os.path.join(ROOT_DIR, "Inp/TOPSIS_Input.csv"))
# print(TOPSISinput)
# print(TOPSISinput.Metric)
