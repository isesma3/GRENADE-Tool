# Function that implements a vtail for analysis in AVL

import numpy as np
import avlwrapper as avl
import math

def V_tail(CASE,S_Vtail,S_Htail,S_Vertail,cnt,neg_val,fuse_L,fuse_D):
    # Wing calcs
    wing_aspect_ratio = CASE.Wing_AR[0]
    wing_area = CASE.Wing_S[0]
    wing_span = np.sqrt(wing_aspect_ratio * wing_area)
    wing_taper = CASE.Wing_TR[0]
    wing_root_chord = 2 * wing_span / (wing_aspect_ratio * (1 + wing_taper))
    wing_tip_chord = wing_root_chord * wing_taper
    wing_c_bar = (2/3)*wing_root_chord*((wing_taper**2 + wing_taper + 1)/(wing_taper+1))
    b = np.sqrt(CASE.Wing_AR[0]*CASE.Wing_S[0])
    Vtail_taper = 1

    Vtail_area = S_Vtail if S_Vtail >= 0 else 0.01
    if S_Vtail < 0:
        S_Vtail = 0.01
        neg_val = 1
        cnt += 1
    elif 0 < cnt < 7:
        cnt += 1
    else:
        neg_val = 0
        cnt = 0

        Vertail_area = S_Vertail if S_Vertail >= 0 else 0.01
    if S_Vertail < 0:
        S_Vertail = 0.01
        neg_val = 1
        cnt += 1
    elif 0 < cnt < 7:
        cnt += 1
    else:
        neg_val = 0
        cnt = 0

    Hortail_area = S_Htail if S_Htail >= 0 else 0.01
    if S_Htail < 0:
        S_Htail = 0.01
        neg_val = 1
        cnt += 1
    elif 0 < cnt < 7:
        cnt += 1
    else:
        neg_val = 0
        cnt = 0
    
    AR_Vtail = 6 # Assumption
    b_Vtail = np.sqrt(AR_Vtail*S_Vtail)
    Vtail_root_chord = (2*S_Vtail)/(b_Vtail*(1+Vtail_taper))
    Vtail_tip_chord = Vtail_taper * Vtail_root_chord
    Y_Vtail = b_Vtail/2
    Vtail_mac = (2/3)*Vtail_root_chord*((Vtail_taper**2 + Vtail_taper +1)/(Vtail_taper+1))
    Fuse_front = 0.0857 # Assumption for the percentage of the fuselage sticking out of the front of the aircraft
    tail_L = fuse_L - fuse_L*Fuse_front - wing_c_bar
    
    Vtail_root_le_pnt = avl.Point(x=fuse_L -(fuse_D/2)*np.tan(math.radians((30+60+65)/3)) - Vtail_root_chord, # Places the tail
                                    y=0,
                                    z=0)

    rudder = avl.Control(name='rudder',
                            gain=1,
                            x_hinge=0.7,
                            duplicate_sign=-1)

    Vtail_tip_le_pnt = avl.Point(x=fuse_L -(fuse_D/2)*np.tan(math.radians((30+60+65)/3)) - Vtail_root_chord,
                                    y=b_Vtail/2,
                                    z=Y_Vtail*np.tan(np.arctan(np.sqrt(S_Vertail/S_Htail))))

    root_section_Vtail = avl.Section(leading_edge_point=Vtail_root_le_pnt,
                                    chord=Vtail_root_chord,
                                    airfoil=avl.NacaAirfoil('0010'),
                                    controls=[rudder])
    tip_section_Vtail = avl.Section(leading_edge_point=Vtail_tip_le_pnt,
                                    chord=Vtail_tip_chord,
                                    airfoil=avl.NacaAirfoil('0008'),
                                    controls=[rudder])
    # y_duplicate=0.0 duplicates the winglet over a XZ-plane at Y=0.0
    Vtail = avl.Surface(name='Vtail',
                            n_chordwise=10,
                            chord_spacing=avl.Spacing.cosine,
                            n_spanwise=10,
                            span_spacing=avl.Spacing.cosine,
                            y_duplicate=0.0,
                            component=1,
                            sections=[root_section_Vtail, tip_section_Vtail])
    return(Vtail, Vtail_mac, b_Vtail, Vtail_tip_le_pnt, Vtail_root_chord, cnt, neg_val)