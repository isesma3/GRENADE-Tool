# Wing input for AVL
import os

LOCAL_DIR = os.path.dirname(__file__)
ROOT_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from math import radians, tan 
import avlwrapper as avl

def AVLwing(CASE, fuse_D):
    # Wing Inputs
    wing_aspect_ratio = CASE.Wing_AR[0]
    wing_area = CASE.Wing_S[0]
    wing_span = np.sqrt(wing_aspect_ratio * wing_area)
    wing_taper = CASE.Wing_TR[0]
    wing_le_sweep = radians(CASE.Wing_Sweep[0])
    wing_dihedral = radians(CASE.Wing_dhl[0])
    wing_root_tc = CASE.Wing_r_tc[0]
    wing_root_xc = CASE.Wing_r_xc[0]
    wing_root_xtc = CASE.Wing_r_xtc[0]
    wing_tip_tc = CASE.Wing_t_tc[0]
    wing_tip_xc = CASE.Wing_t_xc[0]
    wing_tip_xtc = CASE.Wing_t_xtc[0]
    b = np.sqrt(CASE.Wing_AR[0]*CASE.Wing_S[0])
    
    # Wing Calculations
    if not (CASE.NACA0[0]):
        af0 = os.path.join(ROOT_DIR, 'Aero/Airfoils', CASE.Foil0[0] + '.dat')
    if not (CASE.NACA1[0]):
        af1 = os.path.join(ROOT_DIR, 'Aero/Airfoils', CASE.Foil1[0] + '.dat')

    # wing_root_chord = 2 * wing_span / (wing_aspect_ratio * (1 + wing_taper))
    wing_root_chord = 2 * wing_span / (wing_aspect_ratio * (1 + wing_taper))
    wing_tip_chord = wing_root_chord * wing_taper
    wing_c_bar = (2/3)*wing_root_chord*((wing_taper**2 + wing_taper + 1)/(wing_taper+1))

    wing_root_le_pnt = avl.Point(x=0,
                                y=0.5 * fuse_D,
                                z=0)

    wing_tip_le_pnt = avl.Point(x=0.5 * wing_span * tan(wing_le_sweep),
                                y=0.5 * wing_span + 0.5 * fuse_D,
                                z=0.5 * wing_span * tan(wing_dihedral))
    if CASE.NACA0[0]:
        root_section = avl.Section(leading_edge_point=wing_root_le_pnt,
                                   chord=wing_root_chord,
                                   airfoil=avl.NacaAirfoil(CASE.Foil0[0]))
    else:
        root_section = avl.Section(leading_edge_point=wing_root_le_pnt,
                                   chord=wing_root_chord,
                                   airfoil=avl.FileAirfoil(af0))
    if CASE.NACA1[0]:
        tip_section = avl.Section(leading_edge_point=wing_tip_le_pnt,
                                  chord=wing_tip_chord,
                                  airfoil=avl.NacaAirfoil(CASE.Foil1[0]))
    else:
        tip_section = avl.Section(leading_edge_point=wing_tip_le_pnt,
                                  chord=wing_tip_chord,
                                  airfoil=avl.FileAirfoil(af1))

    # y_duplicate=0.0 duplicates the wing over a XZ-plane at Y=0.0
    wing = avl.Surface(name='wing',
                       n_chordwise=10,
                       chord_spacing=avl.Spacing.cosine,
                       n_spanwise=10,
                       span_spacing=avl.Spacing.cosine,
                       y_duplicate=0.0,
                       component=1,
                       sections=[root_section, tip_section])
    return(wing_root_chord, wing_tip_chord, wing_taper, wing_root_le_pnt, wing_tip_le_pnt, wing_area, wing_span, wing, wing_c_bar, wing_root_xtc, wing_root_tc, wing_le_sweep, b)