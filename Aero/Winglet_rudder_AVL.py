# Function that implements a winglet with rudders for analysis in AVL

import numpy as np
import avlwrapper as avl

def Winglet_rudd(wing_tip_chord, wing_tip_le_pnt,cnt,neg_val,swl):
    # ## Winglet Surface
    # winglets will have rudders.
    winglet_area = swl if swl >= 0 else 0.01
    if swl < 0:
        swl = 0.01
        neg_val = 1
        cnt += 1
    elif 0 < cnt < 7:
        cnt += 1
    else:
        neg_val = 0
        cnt = 0
    
    winglet_taper = 0.2 #Assumption
    winglet_root_chord = wing_tip_chord
    winglet_tip_chord = winglet_taper * winglet_root_chord
    b_winglet = winglet_area / (0.5 * (winglet_root_chord - winglet_tip_chord) + winglet_tip_chord)
    winglet_mac = (2/3)*winglet_root_chord*((winglet_taper**2 + winglet_taper + 1)/(winglet_taper + 1))
    # winglet_aspect_ratio = winglet_span ** 2 / winglet_area
    winglet_le_sweep = np.arctan((winglet_root_chord - winglet_tip_chord)/b_winglet)

    winglet_root_le_pnt = avl.Point(x=wing_tip_le_pnt.x, y=wing_tip_le_pnt.y, z=wing_tip_le_pnt.z)

    rudder = avl.Control(name='rudder',
                            gain=1,
                            x_hinge=0.7,
                            duplicate_sign=-1)

    winglet_tip_le_pnt = avl.Point(x=winglet_root_le_pnt.x + b_winglet * np.tan(winglet_le_sweep),
                                    y=winglet_root_le_pnt.y,
                                    z=winglet_root_le_pnt.z + b_winglet)

    root_section_wl = avl.Section(leading_edge_point=winglet_root_le_pnt,
                                    chord=winglet_root_chord,
                                    airfoil=avl.NacaAirfoil('0010'),
                                    controls=[rudder])
    tip_section_wl = avl.Section(leading_edge_point=winglet_tip_le_pnt,
                                    chord=winglet_tip_chord,
                                    airfoil=avl.NacaAirfoil('0008'),
                                    controls=[rudder])
    # y_duplicate=0.0 duplicates the winglet over a XZ-plane at Y=0.0
    winglet_rudd = avl.Surface(name='winglet_rudd',
                            n_chordwise=10,
                            chord_spacing=avl.Spacing.cosine,
                            n_spanwise=10,
                            span_spacing=avl.Spacing.cosine,
                            y_duplicate=0.0,
                            component=1,
                            sections=[root_section_wl, tip_section_wl])
    return(winglet_rudd, winglet_mac, b_winglet, winglet_tip_le_pnt, winglet_root_chord, cnt, neg_val)