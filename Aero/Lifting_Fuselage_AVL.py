# Function that implements a lifting fuselage for analysis in AVL
import os

LOCAL_DIR = os.path.dirname(__file__)
ROOT_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))
import numpy as np
import avlwrapper as avl
import math

def AVL_fuselage(CASE,fuse_L,fuse_D):
    # Lifting Fuselage AVL Setup
    Fuse_front = 0.0857 # Assumption for the percentage of the fuselage sticking out of the front of the aircraft

    # Wing root airfoil
    if not (CASE.NACA0[0]):
        af0 = os.path.join(ROOT_DIR, 'Aero/Airfoils', CASE.Foil0[0] + '.dat')
    
    fuse_root_le_pnt = avl.Point(x=-(fuse_D/2)*np.tan(math.radians((30+60+65)/3)),
                                    y=0,
                                    z=0)

    fuse_tip_le_pnt = avl.Point(x=0,
                                    y=fuse_D/2,
                                    z=0)

    root_section_fuse = avl.Section(leading_edge_point=fuse_root_le_pnt,
                                    chord=fuse_L,
                                    airfoil=avl.FileAirfoil(os.path.join(ROOT_DIR, 'Aero/Airfoils/naca45130.dat')))

    tip_section_fuse = avl.Section(leading_edge_point=fuse_tip_le_pnt,
                                   chord=fuse_L-(fuse_D/2)*np.tan(math.radians((30+60+65)/3)),
                                   airfoil=avl.FileAirfoil(os.path.join(ROOT_DIR, 'Aero/Airfoils/naca45130.dat')))
    # y_duplicate=0.0 duplicates the winglet over a XZ-plane at Y=0.0
    fuselage = avl.Surface(name='fuselage',
                            n_chordwise=10,
                            chord_spacing=avl.Spacing.cosine,
                            n_spanwise=10,
                            span_spacing=avl.Spacing.cosine,
                            y_duplicate=0.0,
                            component=1,
                            sections=[root_section_fuse, tip_section_fuse])
    
    return fuselage