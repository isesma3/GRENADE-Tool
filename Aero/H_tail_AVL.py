# Function that implements a H-tail for analysis in AVL

import numpy as np
import avlwrapper as avl
import math

def H_tail(CASE,S_Htail,S_Vertail,cnt,neg_val,fuse_L,fuse_D):
    # Wing calcs
    wing_aspect_ratio = CASE.Wing_AR[0]
    wing_area = CASE.Wing_S[0]
    wing_span = np.sqrt(wing_aspect_ratio * wing_area)
    wing_taper = CASE.Wing_TR[0]
    wing_root_chord = 2 * wing_span / (wing_aspect_ratio * (1 + wing_taper))
    wing_tip_chord = wing_root_chord * wing_taper
    wing_c_bar = (2/3)*wing_root_chord*((wing_taper**2 + wing_taper + 1)/(wing_taper+1))
    b = np.sqrt(CASE.Wing_AR[0]*CASE.Wing_S[0])

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
    
    # Horizontal tail 
    AR_Htail = 4 # Assumption
    Htail_taper = 0.45  # Assumption
    b_Htail = np.sqrt(AR_Htail*S_Htail)
    Htail_root_chord = (2*S_Htail)/(b_Htail*(1+Htail_taper))
    Htail_tip_chord = Htail_taper * Htail_root_chord
    Y_Htail = b_Htail/2
    Htail_mac = (2/3)*Htail_root_chord*((Htail_taper**2 + Htail_taper +1)/(Htail_taper+1))
    Fuse_front = 0.0857 # Assumption for the percentage of the fuselage sticking out of the front of the aircraft
    tail_L = fuse_L - fuse_L*Fuse_front - wing_c_bar
    
    Htail_root_le_pnt = avl.Point(x=fuse_L -(fuse_D/2)*np.tan(math.radians((30+60+65)/3)) - Htail_root_chord, # Places the tail
                                    y = fuse_D/2,
                                    z=0)

    # rudder = avl.Control(name='rudder',
    #                         gain=1,
    #                         x_hinge=0.7,
    #                         duplicate_sign=-1)

    Htail_tip_le_pnt = avl.Point(x=fuse_L -(fuse_D/2)*np.tan(math.radians((30+60+65)/3)) - Htail_root_chord,
                                    y=b_Htail/2 + fuse_D/2,
                                    z=0)

    root_section_Htail = avl.Section(leading_edge_point=Htail_root_le_pnt,
                                    chord=Htail_root_chord,
                                    airfoil=avl.NacaAirfoil('0010'))
    tip_section_Htail = avl.Section(leading_edge_point=Htail_tip_le_pnt,
                                    chord=Htail_tip_chord,
                                    airfoil=avl.NacaAirfoil('0008'))
    # y_duplicate=0.0 duplicates the winglet over a XZ-plane at Y=0.0
    Htail = avl.Surface(name='Htail',
                            n_chordwise=10,
                            chord_spacing=avl.Spacing.cosine,
                            n_spanwise=10,
                            span_spacing=avl.Spacing.cosine,
                            y_duplicate=0.0,
                            component=1,
                            sections=[root_section_Htail, tip_section_Htail])
    
    # Vertical tail
    AR_Vertail = 1.65 # Assumption
    Vertail_taper = 0.45 # Assumption
    S_Vertail = S_Vertail/2 # Half the vertical tail area for H-tail config
    b_Vertail = np.sqrt(AR_Vertail*(S_Vertail))
    Vertail_root_chord = Htail_tip_chord
    Vertail_tip_chord = Vertail_taper * Vertail_root_chord
    Y_Htail = b_Vertail/2
    Vertail_mac = (2/3)*Vertail_root_chord*((Vertail_taper**2 + Vertail_taper +1)/(Vertail_taper+1))

    Vertail_root_le_pnt = Htail_tip_le_pnt

    rudder = avl.Control(name='rudder',
                            gain=1,
                            x_hinge=0.7,
                            duplicate_sign=-1)

    Vertail_tip_le_pnt = avl.Point(x=fuse_L -(fuse_D/2)*np.tan(math.radians((30+60+65)/3)) - Htail_root_chord,
                                    y=b_Htail/2 + fuse_D/2,
                                    z=b_Vertail/2)

    root_section_Vertail = avl.Section(leading_edge_point=Vertail_root_le_pnt,
                                    chord=Vertail_root_chord,
                                    airfoil=avl.NacaAirfoil('0010'),
                                    controls=[rudder])
    tip_section_Vertail = avl.Section(leading_edge_point=Vertail_tip_le_pnt,
                                    chord=Vertail_tip_chord,
                                    airfoil=avl.NacaAirfoil('0008'),
                                    controls=[rudder])
    # y_duplicate=0.0 duplicates the winglet over a XZ-plane at Y=0.0
    Vertail = avl.Surface(name='Vertail',
                            n_chordwise=10,
                            chord_spacing=avl.Spacing.cosine,
                            n_spanwise=10,
                            span_spacing=avl.Spacing.cosine,
                            y_duplicate=0.0,
                            component=1,
                            sections=[root_section_Vertail, tip_section_Vertail])
    
    return(Htail, Htail_mac, b_Htail, Htail_tip_le_pnt, Htail_root_chord, Vertail, Vertail_mac, b_Vertail, Vertail_tip_le_pnt, Vertail_root_chord, cnt, neg_val)