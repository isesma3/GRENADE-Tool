import pandas as pd
import numpy as np
import time

import openvsp_config
openvsp_config.LOAD_GRAPHICS = False
openvsp_config.LOAD_FACADE = False
import openvsp as vsp

def generate_BWB_VSPfile(data):
    # Change OpenVSP Geometry
    vsp.ClearVSPModel()
    file = vsp.ReadVSPFile("./VSP_tests/config_blended.vsp3")
    wing_id = vsp.FindGeomsWithName("Wing")[0]
    tail_id = vsp.FindGeomsWithName("Tail")[0]
    left_boom_id = vsp.FindGeomsWithName("Left Port Boom")[0]
    right_boom_id = vsp.FindGeomsWithName("Right Port Boom")[0]
    xsec_0 = vsp.GetXSecSurf(wing_id,1)
    xsec_tail = vsp.GetXSecSurf(tail_id,1)
    PodGeom_R1_id = vsp.FindGeomsWithName("PodGeom_R1")[0]
    PodGeom_R2_id = vsp.FindGeomsWithName("PodGeom_R2")[0]
    PodGeom_L1_id = vsp.FindGeomsWithName("PodGeom_L1")[0]
    PodGeom_L2_id = vsp.FindGeomsWithName("PodGeom_L2")[0]
    PodGeoms = { "PodGeom_R1": PodGeom_R1_id,
                 "PodGeom_R2": PodGeom_R2_id,
                 "PodGeom_L1": PodGeom_L1_id,
                 "PodGeom_L2": PodGeom_L2_id
                 }
    PropGeom_R1 = vsp.FindGeomsWithName("PropGeom_R1")[0]
    PropGeom_R2 = vsp.FindGeomsWithName("PropGeom_R2")[0]
    PropGeom_L1 = vsp.FindGeomsWithName("PropGeom_L1")[0]
    PropGeom_L2 = vsp.FindGeomsWithName("PropGeom_L2")[0]
    PropGeoms = { "PropGeom_R1": PropGeom_R1,
                 "PropGeom_R2": PropGeom_R2,
                 "PropGeom_L1": PropGeom_L1,
                 "PropGeom_L2": PropGeom_L2
                 }  
    for i in range(len(data)):
        if data.Planform[i] <= 0:
            print("Case %d: Negative Planform"%(i))
            continue
        if data.Span[i] <= 0:
            print("Case %d: Negative Span"%(i))
            continue
        if data.S_VTail[i] <= 0:
            print("Case %d: Negative Vtail Surface"%(i))
            continue

        # Given parameters for the BWB
        total_span = data.Span[i] / 2  # meters, total wingspan halved for symmetry
        TotalWingArea_id = vsp.GetParm(wing_id,"TotalArea","WingGeom")
        TotalWingSpan_id = vsp.GetParm(wing_id,"TotalSpan","WingGeom")
        TotalTailArea_id = vsp.GetParm(tail_id,"TotalArea","WingGeom")
        TotalTailSpan_id = vsp.GetParm(tail_id,"TotalSpan","WingGeom")
        
        for j in range(8):
            vsp.SetParmValUpdate(TotalWingArea_id,data.Planform[i])
            vsp.SetParmValUpdate(TotalWingSpan_id,data.Span[i])
            vsp.SetParmValUpdate(TotalTailArea_id,data.S_VTail[i])


        desired_volume = 1  # m^3, significantly larger desired volume for the first 4 sections

        # Original span length ratios
        original_span_lengths = [0.05843, 0.05843, 0.11686, 0.23371, 0.23371, 0.23371, 0.46742, 0.23371, 0.11686, 0.02921, 0.01461, 
                                0.01461, 0.01461, 0.01461, 0.01461, 0.00730, 0.00365, 0.00183, 0.00046, 0.00023, 0.00011, 0.00011]
        adjusted_span_lengths = [x / sum(original_span_lengths) * total_span for x in original_span_lengths]
        cum_adjusted_span_lengths = [sum(adjusted_span_lengths[:i]) for i in range(len(adjusted_span_lengths))]
        # span_starts = np.cumsum(adjusted_span_lengths)
        nsection = len(adjusted_span_lengths)

        # Adjust the section sweep angles so the total sweep angle corresponds to the case's
        original_sweep_angles = [18.0, 25.0, 30.0, 24.0, 20.0, 13.0, 5.0, 4.0, 3.0, 4.0, 12.0, 20.0, 30.0, 35.0, 42.0, 50.0, 52.0, 54.0, 58.0, 70.0, 80.0, 89.0]
        # Adjust the section dihedral angles so the total dihedral angle corresponds to the case's
        adjusted_dihedral_angles = [x / sum(adjusted_span_lengths) * data.Wing_dhl[i] for x in adjusted_span_lengths]

        sec_id = 4
        xsec10_id = vsp.GetXSec(xsec_0, sec_id)
        xsec10_RootC_id = vsp.FindParm(xsec10_id, "Root_Chord","XSec")
        xsec10_tcratio_id = vsp.GetXSecParm(xsec10_id, "ThickChord")
        xsec10_RootC = vsp.GetParmVal(xsec10_RootC_id)
        xsec10_tcratio = vsp.GetParmVal(xsec10_tcratio_id)
        FuseL = data.Fuse_L[i]
        FuseD = data.Fuse_D[i]
        for j in range(0,sec_id):
            xseci_id = vsp.GetXSec(xsec_0,j)
            xseci_RootC_id = vsp.FindParm(xseci_id, "Root_Chord","XSec")
            xseci_tcratio_id = vsp.GetXSecParm(xseci_id, "ThickChord")
            xseci_TipC_id = vsp.FindParm(xseci_id, "Tip_Chord","XSec")
            vsp.SetParmValUpdate(xseci_RootC_id, (FuseL-xsec10_RootC)*(sec_id-j)/sec_id + xsec10_RootC)
            vsp.SetParmValUpdate(xseci_TipC_id, (FuseL-xsec10_RootC)*(sec_id-j-1)/sec_id + xsec10_RootC)
            vsp.SetParmValUpdate(xseci_tcratio_id, (FuseD/FuseL-xsec10_tcratio)*(sec_id-j)/sec_id + xsec10_tcratio)

        xloc_lboom_id = vsp.GetParm(left_boom_id,"X_Rel_Location","XForm")
        xloc_rboom_id = vsp.GetParm(right_boom_id,"X_Rel_Location","XForm")
        yloc_lboom_id = vsp.GetParm(left_boom_id,"Y_Rel_Location","XForm")
        yloc_rboom_id = vsp.GetParm(right_boom_id,"Y_Rel_Location","XForm")
        zloc_rboom_id = vsp.GetParm(right_boom_id,"Z_Rel_Location","XForm")
        boom_angle_id = vsp.GetParm(right_boom_id,"Y_Rel_Rotation","XForm")
        xloc_tail_id = vsp.GetParm(tail_id,"X_Rel_Location","XForm")
        zloc_tail_id = vsp.GetParm(tail_id,"Z_Rel_Location","XForm")
        xsec_tail_id = vsp.GetXSec(xsec_tail,1)
        tail_dhl_id = vsp.FindParm(xsec_tail_id, "Dihedral","XSec")
        tail_dhl = vsp.GetParmVal(tail_dhl_id)
        boom_angle = vsp.GetParmVal(boom_angle_id)
        zloc_rboom = vsp.GetParmVal(zloc_rboom_id)
        TotalTailSpan = vsp.GetParmVal(TotalTailSpan_id)

        idx_xsec = min(range(len(cum_adjusted_span_lengths)), key=lambda i: abs(cum_adjusted_span_lengths[i] - TotalTailSpan/2))
        xsec_id = vsp.GetXSec(xsec_0,idx_xsec)
        tipC_id = vsp.FindParm(xsec_id, "Tip_Chord","XSec")
        tipC_xsec = vsp.GetParmVal(tipC_id)
        x_le_sec = sum(adjusted_span_lengths[:idx_xsec])*np.tan(data.Wing_Sweep[i]*np.pi/180)
        lboom_length_id = vsp.GetParm(left_boom_id,"Length","Design")
        rboom_length_id = vsp.GetParm(right_boom_id,"Length","Design")
        boom_length =  0.27*data.Span[i]        
        vsp.SetParmValUpdate(lboom_length_id, boom_length) # Setting the boom length as 1/3 of the total span
        vsp.SetParmValUpdate(rboom_length_id, boom_length) # Setting the boom length as 1/3 of the total span
        vsp.SetParmValUpdate(xloc_lboom_id, x_le_sec + tipC_xsec*0.75)
        vsp.SetParmValUpdate(xloc_rboom_id, x_le_sec + tipC_xsec*0.75)
        vsp.SetParmValUpdate(yloc_lboom_id, -TotalTailSpan/3)
        vsp.SetParmValUpdate(yloc_rboom_id, TotalTailSpan/3)
        vsp.SetParmValUpdate(xloc_tail_id, x_le_sec + tipC_xsec*0.75 + boom_length*0.95)
        vsp.SetParmValUpdate(zloc_tail_id, np.sin(-boom_angle)*boom_length*0.95 + np.sin(-tail_dhl)*TotalTailSpan + zloc_rboom)
        
        podR1_length_id = vsp.GetParm(PodGeom_R1_id,"Length","Design")
        podR1_length = vsp.GetParmVal(podR1_length_id)
        for pod_tag, pod_id in PodGeoms.items():
            x_pod_id = vsp.GetParm(pod_id,"X_Rel_Location","XForm")
            y_pod_id = vsp.GetParm(pod_id,"Y_Rel_Location","XForm")
            if "R1" in pod_tag:
                vsp.SetParmValUpdate(y_pod_id, sum(adjusted_span_lengths[:6]))
                x_pod = sum(adjusted_span_lengths[:6])*np.tan(data.Wing_Sweep[i]*np.pi/180) - 2*podR1_length/3
                vsp.SetParmValUpdate(x_pod_id, x_pod)
            if "R2" in pod_tag:
                vsp.SetParmValUpdate(y_pod_id, sum(adjusted_span_lengths[:9]))
                x_pod = sum(adjusted_span_lengths[:9])*np.tan(data.Wing_Sweep[i]*np.pi/180) - 2*podR1_length/3
                vsp.SetParmValUpdate(x_pod_id, x_pod)
            if "L1" in pod_tag:
                vsp.SetParmValUpdate(y_pod_id, -sum(adjusted_span_lengths[:6]))
                x_pod = sum(adjusted_span_lengths[:6])*np.tan(data.Wing_Sweep[i]*np.pi/180) - 2*podR1_length/3
                vsp.SetParmValUpdate(x_pod_id, x_pod)
            if "L2" in pod_tag:
                vsp.SetParmValUpdate(y_pod_id, -sum(adjusted_span_lengths[:9]))
                x_pod = sum(adjusted_span_lengths[:9])*np.tan(data.Wing_Sweep[i]*np.pi/180) - 2*podR1_length/3
                vsp.SetParmValUpdate(x_pod_id, x_pod)

        for prop_tag, prop_id in PropGeoms.items():
            diam_id = vsp.GetParm(prop_id,"Diameter","Design")
            numblade_id = vsp.GetParm(prop_id,"NumBlade","Design")
            x_prop_id = vsp.GetParm(prop_id,"X_Rel_Location","XForm")
            y_prop_id = vsp.GetParm(prop_id,"Y_Rel_Location","XForm")
            vsp.SetParmValUpdate(diam_id, data.PropD[i])
            vsp.SetParmValUpdate(numblade_id, data.PropBlade[i])
            if "R1" in prop_tag:
                vsp.SetParmValUpdate(y_prop_id, sum(adjusted_span_lengths[:6]))
                x_prop = sum(adjusted_span_lengths[:6])*np.tan(data.Wing_Sweep[i]*np.pi/180) - 2*podR1_length/3
                vsp.SetParmValUpdate(x_prop_id, x_prop)
            if "R2" in prop_tag:
                vsp.SetParmValUpdate(y_prop_id, sum(adjusted_span_lengths[:9]))
                x_prop = sum(adjusted_span_lengths[:9])*np.tan(data.Wing_Sweep[i]*np.pi/180) - 2*podR1_length/3
                vsp.SetParmValUpdate(x_prop_id, x_prop)
            if "L1" in prop_tag:
                vsp.SetParmValUpdate(y_prop_id, -sum(adjusted_span_lengths[:6]))
                x_prop = sum(adjusted_span_lengths[:6])*np.tan(data.Wing_Sweep[i]*np.pi/180) - 2*podR1_length/3
                vsp.SetParmValUpdate(x_prop_id, x_prop)
            if "L2" in prop_tag:
                vsp.SetParmValUpdate(y_prop_id, -sum(adjusted_span_lengths[:9]))
                x_prop = sum(adjusted_span_lengths[:9])*np.tan(data.Wing_Sweep[i]*np.pi/180) - 2*podR1_length/3
                vsp.SetParmValUpdate(x_prop_id, x_prop)


        # Calculate linear interpolation for chord lengths from root to tip
        # chord_max = data.Root_C[i]
        # chord_min = data.Tip_C[i]
        # chord = np.linspace(chord_max, chord_min, nsection+1)
        # Increase chord lengths for the first four sections
        # scale_factor = 2  # Example scale factor for demonstration
        # for j in range(4):  # Adjust only the first four sections
        #     chord[j] *= scale_factor
        # Adjust the rest of the chord lengths to ensure a smooth transition
        # This might involve recalculating the taper ratio or adjusting chords directly

        # Generate sections data with adjusted chords for the first four sections
        sections = []
        for j in range(nsection):
            # span_end = span_starts[j] + adjusted_span_lengths[j]
            # current_root_chord = chord[j]
            # current_tip_chord = chord[j+1]

            sections.append({
                'Span': adjusted_span_lengths[j],
                # 'Root_C': current_root_chord,
                # 'Tip_C': current_tip_chord,
                'Sweep': data.Wing_Sweep[i],  # Sweep angle
                'Twist': data.Wing_Twist[i],  # Twist angle 
                'Dihedral': data.Wing_dhl[i]  # Dihedral angle
            })

        # Convert to DataFrame for display
        df_sections = pd.DataFrame(sections)
        
        for j in range(0,vsp.GetNumXSec(xsec_0)-1):
            xsec_id = vsp.GetXSec(xsec_0,j+1)
            Sweep_id = vsp.FindParm(xsec_id, "Sweep","XSec")
            Sweep = df_sections.loc[j,'Sweep']
            vsp.SetParmValUpdate(Sweep_id,Sweep)

            # Root_C_id = vsp.FindParm(xsec_id, "Root_Chord","XSec")
            # Root_C = df_sections.loc[j,'Root_C']
            # vsp.SetParmValUpdate(Root_C_id,Root_C)

            # Tip_C_id = vsp.FindParm(xsec_id, "Tip_Chord","XSec")
            # Tip_C = df_sections.loc[j,'Tip_C']
            # vsp.SetParmValUpdate(Tip_C_id,Tip_C)

            Dihedral_id = vsp.FindParm(xsec_id, "Dihedral","XSec")
            Dihedral = df_sections.loc[j,'Dihedral']
            vsp.SetParmValUpdate(Dihedral_id,Dihedral)

            Twist_id = vsp.FindParm(xsec_id, "Twist","XSec")
            Twist = df_sections.loc[i,'Twist']
            vsp.SetParmValUpdate(Twist_id,Twist)
        vsp.UpdateGeom(wing_id)
        fname = "./VSP_tests/blendedwingbody_{}_case{}.vsp3".format(data.Planet[i],int(data.Case[i]))
        vsp.WriteVSPFile(fname)

    return




def generate_Beluga_VSPfile(data):
    # Change OpenVSP Geometry
    vsp.ClearVSPModel()
    file = vsp.ReadVSPFile("./VSP_tests/config_beluga.vsp3")
    wing_id = vsp.FindGeomsWithName("Wing")[0]
    tail_id = vsp.FindGeomsWithName("Tail")[0]
    xsec_0 = vsp.GetXSecSurf(wing_id,1)
    xsec_tail = vsp.GetXSecSurf(tail_id,1)
    PodGeom_R1_id = vsp.FindGeomsWithName("PodGeom_R1")[0]
    PodGeom_R2_id = vsp.FindGeomsWithName("PodGeom_R2")[0]
    PodGeom_L1_id = vsp.FindGeomsWithName("PodGeom_L1")[0]
    PodGeom_L2_id = vsp.FindGeomsWithName("PodGeom_L2")[0]
    PodGeoms = { "PodGeom_R1": PodGeom_R1_id,
                 "PodGeom_R2": PodGeom_R2_id,
                 "PodGeom_L1": PodGeom_L1_id,
                 "PodGeom_L2": PodGeom_L2_id
                 }
    PropGeom_R1 = vsp.FindGeomsWithName("PropGeom_R1")[0]
    PropGeom_R2 = vsp.FindGeomsWithName("PropGeom_R2")[0]
    PropGeom_L1 = vsp.FindGeomsWithName("PropGeom_L1")[0]
    PropGeom_L2 = vsp.FindGeomsWithName("PropGeom_L2")[0]
    PropGeoms = { "PropGeom_R1": PropGeom_R1,
                 "PropGeom_R2": PropGeom_R2,
                 "PropGeom_L1": PropGeom_L1,
                 "PropGeom_L2": PropGeom_L2
                 }  
    for i in range(len(data)):
        if data.Planform[i] <= 0:
            print("Case %d: Negative Planform"%(i))
            continue
        if data.Span[i] <= 0:
            print("Case %d: Negative Span"%(i))
            continue
        if data.S_VTail[i] <= 0:
            print("Case %d: Negative Vtail Surface"%(i))
            continue

        # Given parameters for the BWB
        total_span = data.Span[i] / 2  # meters, total wingspan halved for symmetry
        TotalWingArea_id = vsp.GetParm(wing_id,"TotalArea","WingGeom")
        TotalWingSpan_id = vsp.GetParm(wing_id,"TotalSpan","WingGeom")
        TotalTailArea_id = vsp.GetParm(tail_id,"TotalArea","WingGeom")
        TotalTailSpan_id = vsp.GetParm(tail_id,"TotalSpan","WingGeom")
        
        for j in range(8):
            vsp.SetParmValUpdate(TotalWingArea_id, data.Planform[i])
            vsp.SetParmValUpdate(TotalWingSpan_id, data.Span[i])
            vsp.SetParmValUpdate(TotalTailArea_id, data.S_VTail[i])


        desired_volume = 1  # m^3, significantly larger desired volume for the first 4 sections

        # Original span length ratios
        original_span_lengths = [0.15058, 0.09760, 0.04643, 0.08932, 0.23634, 0.23634, 0.23634, 0.47269, 0.23634, 0.11817, 0.02954, 
                                0.01477, 0.01477, 0.01477, 0.01477, 0.01477, 0.00739, 0.00369, 0.00185, 0.00046, 0.00023, 0.00012]
        adjusted_span_lengths = [x / sum(original_span_lengths) * total_span for x in original_span_lengths]
        # span_starts = np.cumsum(adjusted_span_lengths)
        nsection = len(adjusted_span_lengths)

        sec_id = 3
        FuseL = data.Fuse_L[i]
        FuseD = data.Fuse_D[i]
        xsec10_id = vsp.GetXSec(xsec_0, sec_id)
        xsec10_RootC_id = vsp.FindParm(xsec10_id, "Root_Chord","XSec")
        xsec10_tcratio_id = vsp.GetXSecParm(xsec10_id, "ThickChord")
        xsec10_RootC = vsp.GetParmVal(xsec10_RootC_id)
        vsp.SetParmValUpdate(xsec10_tcratio_id, FuseD/FuseL)
        
        for j in range(1,sec_id+1):
            xseci_id = vsp.GetXSec(xsec_0,j)
            xseci_RootC_id = vsp.FindParm(xseci_id, "Root_Chord","XSec")
            if j == 1: 
                vsp.SetParmValUpdate(xseci_RootC_id, (FuseL-xsec10_RootC)*(sec_id-j+1)/sec_id + xsec10_RootC)
            elif j == 2:
                vsp.SetParmValUpdate(xseci_RootC_id, (FuseL-xsec10_RootC)*(sec_id-j+1)*1.15/sec_id + xsec10_RootC)
            else:
                vsp.SetParmValUpdate(xseci_RootC_id, (FuseL-xsec10_RootC)*(sec_id-j+1)*0.95/sec_id + xsec10_RootC)
            

        for j in range(0,sec_id+2):
            airfoil_id = vsp.GetXSec(xsec_0,j)
            xseci_tcratio_id = vsp.GetXSecParm(xseci_id, "ThickChord")
            xsec0_OutBlendLE_id = vsp.GetXSecParm(airfoil_id, "OutLEMode")
            xsec0_OutBlendTE_id = vsp.GetXSecParm(airfoil_id, "OutTEMode")
            xsec0_InBlendLE_id = vsp.GetXSecParm(airfoil_id, "InLEMode")
            xsec0_InBlendTE_id = vsp.GetXSecParm(airfoil_id, "InTEMode")
            xsec0_OutBlendLE_Sweep_id = vsp.GetXSecParm(airfoil_id, "OutLESweep")
            xsec0_OutBlendTE_Sweep_id = vsp.GetXSecParm(airfoil_id, "OutTESweep")
            xsec0_InBlendLE_Strength_id = vsp.GetXSecParm(airfoil_id, "InLEStrength")
            xsec0_InBlendTE_Strength_id = vsp.GetXSecParm(airfoil_id, "InTEStrength")
            vsp.SetParmValUpdate(xseci_tcratio_id, FuseD/FuseL)
            
            if j == 0:
                vsp.SetParmValUpdate(xsec0_OutBlendLE_id, 1.0)
                vsp.SetParmValUpdate(xsec0_OutBlendTE_id, 1.0)
                vsp.SetParmValUpdate(xsec0_OutBlendLE_Sweep_id, 0.0)
                vsp.SetParmValUpdate(xsec0_OutBlendTE_Sweep_id, 0.0)
            else:
                vsp.SetParmValUpdate(xsec0_InBlendLE_id, 4.0)
                vsp.SetParmValUpdate(xsec0_InBlendTE_id, 5.0)
                vsp.SetParmValUpdate(xsec0_InBlendLE_Strength_id, 0.5)
                vsp.SetParmValUpdate(xsec0_InBlendTE_Strength_id, 0.1)


        xsec1_id = vsp.GetXSec(xsec_0,1)
        xsec2_id = vsp.GetXSec(xsec_0,2)
        xsec3_id = vsp.GetXSec(xsec_0,3)
        Sweep1_id = vsp.FindParm(xsec1_id, "Sweep","XSec")
        Sweep2_id = vsp.FindParm(xsec2_id, "Sweep","XSec")
        Sweep3_id = vsp.FindParm(xsec3_id, "Sweep","XSec")
        Sweep1 = vsp.GetParmVal(Sweep1_id)
        Sweep2 = vsp.GetParmVal(Sweep2_id)
        Sweep3 = vsp.GetParmVal(Sweep3_id)

        xloc_tail_id = vsp.GetParm(tail_id,"X_Rel_Location","XForm")
        zloc_tail_id = vsp.GetParm(tail_id,"Z_Rel_Location","XForm")
        vsp.SetParmValUpdate(xloc_tail_id, FuseL*0.90)
        vsp.SetParmValUpdate(zloc_tail_id, 0)
        
        podR1_length_id = vsp.GetParm(PodGeom_R1_id,"Length","Design")
        podR1_length = vsp.GetParmVal(podR1_length_id)
        sweep_displacement_1 = sum(adjusted_span_lengths[3:6])*np.tan(data.Wing_Sweep[i]*np.pi/180) + adjusted_span_lengths[0]*np.tan(Sweep1*np.pi/180) + adjusted_span_lengths[1]*np.tan(Sweep2*np.pi/180) + adjusted_span_lengths[2]*np.tan(Sweep3*np.pi/180)
        sweep_displacement_2 = sum(adjusted_span_lengths[3:9])*np.tan(data.Wing_Sweep[i]*np.pi/180) + adjusted_span_lengths[0]*np.tan(Sweep1*np.pi/180) + adjusted_span_lengths[1]*np.tan(Sweep2*np.pi/180) + adjusted_span_lengths[2]*np.tan(Sweep3*np.pi/180)

        for pod_tag, pod_id in PodGeoms.items():
            x_pod_id = vsp.GetParm(pod_id,"X_Rel_Location","XForm")
            y_pod_id = vsp.GetParm(pod_id,"Y_Rel_Location","XForm")
            if "R1" in pod_tag:
                vsp.SetParmValUpdate(y_pod_id, sum(adjusted_span_lengths[:6]))
                x_pod = sweep_displacement_1 - 2*podR1_length/3
                vsp.SetParmValUpdate(x_pod_id, x_pod)
            if "R2" in pod_tag:
                vsp.SetParmValUpdate(y_pod_id, sum(adjusted_span_lengths[:9]))
                x_pod = sweep_displacement_2 - 2*podR1_length/3
                vsp.SetParmValUpdate(x_pod_id, x_pod)
            if "L1" in pod_tag:
                vsp.SetParmValUpdate(y_pod_id, -sum(adjusted_span_lengths[:6]))
                x_pod = sweep_displacement_1 - 2*podR1_length/3
                vsp.SetParmValUpdate(x_pod_id, x_pod)
            if "L2" in pod_tag:
                vsp.SetParmValUpdate(y_pod_id, -sum(adjusted_span_lengths[:9]))
                x_pod = sweep_displacement_2 - 2*podR1_length/3
                vsp.SetParmValUpdate(x_pod_id, x_pod)

        for prop_tag, prop_id in PropGeoms.items():
            diam_id = vsp.GetParm(prop_id,"Diameter","Design")
            numblade_id = vsp.GetParm(prop_id,"NumBlade","Design")
            x_prop_id = vsp.GetParm(prop_id,"X_Rel_Location","XForm")
            y_prop_id = vsp.GetParm(prop_id,"Y_Rel_Location","XForm")
            vsp.SetParmValUpdate(diam_id, data.PropD[i])
            vsp.SetParmValUpdate(numblade_id, data.PropBlade[i])
            if "R1" in prop_tag:
                vsp.SetParmValUpdate(y_prop_id, sum(adjusted_span_lengths[:6]))
                x_prop = sweep_displacement_1 - 2*podR1_length/3
                vsp.SetParmValUpdate(x_prop_id, x_prop)
            if "R2" in prop_tag:
                vsp.SetParmValUpdate(y_prop_id, sum(adjusted_span_lengths[:9]))
                x_prop = sweep_displacement_2 - 2*podR1_length/3
                vsp.SetParmValUpdate(x_prop_id, x_prop)
            if "L1" in prop_tag:
                vsp.SetParmValUpdate(y_prop_id, -sum(adjusted_span_lengths[:6]))
                x_prop = sweep_displacement_1 - 2*podR1_length/3
                vsp.SetParmValUpdate(x_prop_id, x_prop)
            if "L2" in prop_tag:
                vsp.SetParmValUpdate(y_prop_id, -sum(adjusted_span_lengths[:9]))
                x_prop = sweep_displacement_2 - 2*podR1_length/3
                vsp.SetParmValUpdate(x_prop_id, x_prop)



        # Generate sections data with adjusted chords for the first four sections
        sections = []
        for j in range(nsection):
            sections.append({
                'Sweep': data.Wing_Sweep[i],  # Sweep angle
                'Twist': data.Wing_Twist[i],  # Twist angle 
                'Dihedral': data.Wing_dhl[i]  # Dihedral angle
            })

        # Convert to DataFrame for display
        df_sections = pd.DataFrame(sections)
        
        for j in range(3,vsp.GetNumXSec(xsec_0)-1):
            xsec_id = vsp.GetXSec(xsec_0,j+1)
            Sweep_id = vsp.FindParm(xsec_id, "Sweep","XSec")
            Sweep = df_sections.loc[i,'Sweep']
            vsp.SetParmValUpdate(Sweep_id,Sweep)

            Dihedral_id = vsp.FindParm(xsec_id, "Dihedral","XSec")
            Dihedral = df_sections.loc[i,'Dihedral']
            vsp.SetParmValUpdate(Dihedral_id,Dihedral)

            Twist_id = vsp.FindParm(xsec_id, "Twist","XSec")
            Twist = df_sections.loc[i,'Twist']
            vsp.SetParmValUpdate(Twist_id,Twist)
        vsp.UpdateGeom(wing_id)
        fname = "./VSP_tests/blendedwingbody_{}_case{}.vsp3".format(data.Planet[i],int(data.Case[i]))
        vsp.WriteVSPFile(fname)

    return



if __name__ == "__main__":
    df = pd.DataFrame(
        data={
            "Case": 0,
            "Mass": 148,
            "Planform": 1.52,
            "Span": 5.0,
            "Wing_Sweep": 0.0,
            'Wing_dhl': 2.4,
            'Wing_TR': 0.35,
            'Wing_Twist': 0.0,
            "Fuse_L": 3.0,
            "Fuse_D": 0.76,
            "S_VTail": 0.34,
            "PropD": 0.2026,
            "PropBlade": 2.0,
            "Planet": "Titan",
            "lb": 0.27
        },
        index=[i for i in range(1)],
    )
    # generate_BWB_VSPfile(df)
    generate_Beluga_VSPfile(df)
#     vsp.ReadVSPFile("./VSP_tests/config_blended.vsp3")
#     vsp.InitGUI()
#     vsp.StartGUI()
#     # Load a VSP file
    
#     wing_id = vsp.FindGeomsWithName("Wing")[0]

#     # Continuously update parameters
#     while True:
#         # Update the parameter value
        
#         TotalWingSpan_id = vsp.GetParm(wing_id,"TotalSpan","WingGeom")
#         time.sleep(5)
#         vsp.SetParmVal(TotalWingSpan_id, 1.0)
#         print("param set")
#         # Trigger update in the OpenVSP GUI
#         vsp.Update()
#         vsp.UpdateGUI()