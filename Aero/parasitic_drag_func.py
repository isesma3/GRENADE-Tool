import numpy as np
import pandas as pd
import sys 
import math
from math import radians

import os

ROOT_DIR = os.path.dirname(__file__)

sys.path.insert(0, ROOT_DIR + "Main_v5.py")


# # This is a simplistic way to find parasitic drag from the values that an AVL run will find. 
# # Those values will not be accurate since they technically need CD0 to calculate them
# def pd(cl, cd, ar, e):
#     """ Calculates CD0 based on results from avl
#         Would need to find some way to calculate e for Titan
#     """
#     CD0 = []
#     for i in results:
#         cl = results[i]['Totals']['CLtot']
#         cd = results[i]['Totals']['CDtot']
#         cd0 = cd - cl**2/(np.pi*ar*e)
#         CD0.append(cd0)

#     return CD0


# #This is the function that approximates CD0 from the returned values of other blocks. However,
# # this still requires some in-situ performance data
# def pd(psl, np, vmax, AR, W, rho, rho0, s, e):
#     """ psl = power at sea level                        ;we're supposed to get power from here so find a way to approximate this
#         np = propeller efficiency                       ;assumed value/operating point for a particular prop
#         vmax = max velocity at given altitude           ;we input this value
#         e = oswalds efficiency factor                   ;based on AR (might be earth dependent) - maybe tie to avl
#         AR = aspect ratio                               ;we input this value
#         W = aircraft weight                             ;weight block looping
#         rho = density of air at given altitude          ;atmosphere look up table
#         rho0 = density of air at reference altitude     ;atmosphere look up table
#         s = reference area                              ;internally defined based on how we decide to loop things
#     """
#     K = 1/ (np.pi * e * AR)
#     cd0 = ((2*psl*np)/vmax - (4*K*W**2)/(rho*(rho/rho0)*vmax**2*s)) / rho0*vmax**2*s
#     return cd0



#Now this function is using actual formulas from Raymer and hopefully is exactly what we need 
def form_fact_wing(t_c, Lambda_m, x_c, M):
    """ To be used for wing, tail, strut and pylon
        t = thickness
        c = chord length
        Lambda_m =
        x_c = chordwise location of the airfoil max thickness line
        M = Mach number
    """
    FF = (1 + (0.6/x_c)*(t_c) + 100*(t_c)**4) * (1.34*M**0.18 * (np.cos(Lambda_m))**0.28)
    return FF

def form_fact_fuse(l, d):
    """ form factor for fuselage and smooth canopy
        l = length of fuselage 
        d = diameter of fuselage
    """
    f = l/d
    FF = 0.9 + 5/f**1.5 + f/400
    return FF


def form_fact_nac(f):
    """ form factor for nacelle and smooth external store
        l = length of ncaelle 
        d = diameter of nacelle
        f = l/d = nacelle fineness ratio
    """
    FF = 1 + 0.35/f
    return FF


def sf_coeff(rho, V, l, mu, k):
    """ Skin friction coefficient required for parasitic drag from flat-plate assumption 
        rho - atmospheric density
        V = velocity of flight
        mu = atmospheric viscosity
        l = length of fuselage
        k = skin-roughness value --- take fiberglass and not have this be a variable
    """
    #Reynold's no. for laminar flow
    R = (rho*V*l)/mu
    #Reynold's no. for turbulent flow 
    R_c = 38.21*(l/k)**1.053

    # Maybe add in the compressible turbulent skin friction coeff. eqn. when the M > 0.3
    if R < 499:
        Cf = 1.328/np.sqrt(R)
    else:
        Cf = 0.074/R_c**0.2
    return Cf


def parasitic_drag_boom(rho, V, mu, t, b, c,Lambda_m, ct, tt, l, d, lb, k, S_ref, M, x_c, S_wet_wing, S_wet_tail, S_wet_fuse):
    """ rho - atmospheric density
        V = velocity of flight
        mu = atmospheric viscosity
        t = thickness
        c = chord length
        Lambda_m =
        l = length of fuselage
        d = diameter of fuselage
        k = skin-roughness value --- to be extracted from table
        S_ref = reference area
        M = mach no.
        x_c = chordwise location of the airfoil max thickness line - if we assume an airfoil then we can just hard code this value into the function def
    """
    
    # CD0 for wings
    Cfw = sf_coeff(rho, V, c, mu, k)
    CD0_wing = Cfw * form_fact_wing(t, Lambda_m, x_c, M) * S_wet_wing / S_ref

    # CD0 for fuselage
    if l != 0:
        Cff = sf_coeff(rho, V, l, mu, k)
        CD0_fuse = Cff * form_fact_wing(d/l, math.radians((30+60+65)/3), x_c, M) * S_wet_fuse / S_ref # Assumes a lambda_m to be an average of 30, 60, and 65 degrees
    else:
        CD0_fuse = 0

    # CD0 for tail
    Cft = sf_coeff(rho,V, ct, mu, k)
    CD0_tail = Cft*form_fact_wing(tt, Lambda_m, x_c, M) * S_wet_tail / S_ref

    # CD0 for booms
    Lb = lb*b
    db = 0.06*Lb # 0.06 found from using the NASA ARES studies. The ARES aircraft had a db/lb ~ 0.06
    Cfb = sf_coeff(rho, V, lb, mu, k)
    S_wet_boom = np.pi*db*Lb
    CD0_boom = Cfb*form_fact_fuse(Lb,db) * S_wet_boom / S_ref

    CD0 = CD0_fuse + CD0_wing + CD0_tail + CD0_boom

    return CD0

def parasitic_drag_Vtail(rho, V, mu, t, tt, c, ct, Lambda_m, l, d, k, S_ref, M, x_c, S_wet_wing, S_wet_tail, S_wet_fuse):
    """ rho - atmospheric density
        V = velocity of flight
        mu = atmospheric viscosity
        t = thickness
        c = chord length
        Lambda_m =
        l = length of fuselage
        d = diameter of fuselage
        k = skin-roughness value --- to be extracted from table
        S_ref = reference area
        M = mach no.
        x_c = chordwise location of the airfoil max thickness line - if we assume an airfoil then we can just hard code this value into the function def
    """
    
    # CD0 for wings
    Cfw = sf_coeff(rho, V, c, mu, k)
    CD0_wing = Cfw * form_fact_wing(t, Lambda_m, x_c, M) * S_wet_wing / S_ref

    # CD0 for fuselage
    if l != 0:
        Cff = sf_coeff(rho, V, l, mu, k)
        CD0_fuse = Cff * form_fact_wing(d/l, math.radians((30+60+65)/3), x_c, M) * S_wet_fuse / S_ref # Assumes a lambda_m to be an average of 30, 60, and 65 degrees
    else:
        CD0_fuse = 0

    # CD) for V-tail
    Cft = sf_coeff(rho,V, ct, mu, k)
    CD0_tail = Cft*form_fact_wing(tt, Lambda_m, x_c, M) * S_wet_tail / S_ref

    CD0 = CD0_fuse + CD0_wing + CD0_tail

    return CD0

def parasitic_drag_tail(rho, V, mu, t, tht, tvt, c, cht, cvt, Lambda_m, l, d, k, S_ref, M, x_c, S_wet_wing, S_wet_ht, S_wet_vt, S_wet_fuse):
    """ rho - atmospheric density
        V = velocity of flight
        mu = atmospheric viscosity
        t = thickness
        c = chord length
        Lambda_m =
        l = length of fuselage
        d = diameter of fuselage
        k = skin-roughness value --- to be extracted from table
        S_ref = reference area
        M = mach no.
        x_c = chordwise location of the airfoil max thickness line - if we assume an airfoil then we can just hard code this value into the function def
    """
    
    # CD0 for wings
    Cfw = sf_coeff(rho, V, c, mu, k)
    CD0_wing = Cfw * form_fact_wing(t, Lambda_m, x_c, M) * S_wet_wing / S_ref

    # CD0 for fuselage
    if l != 0:
        Cff = sf_coeff(rho, V, l, mu, k)
        CD0_fuse = Cff * form_fact_wing(d/l, math.radians((30+60+65)/3), x_c, M) * S_wet_fuse / S_ref # Assumes a lambda_m to be an average of 30, 60, and 65 degrees
    else:
        CD0_fuse = 0

    # CD0 for horizontal tail
    Cfht = sf_coeff(rho, V, cht, mu, k)
    CD0_ht = Cfht*form_fact_wing(tht, Lambda_m, x_c, M)*1.08 * S_wet_ht / S_ref

    # CD0 for vertical tail
    Cfvt = sf_coeff(rho, V, cvt, mu, k)
    CD0_vt = Cfvt*form_fact_wing(tvt, Lambda_m, x_c, M)*1.03 * S_wet_vt / S_ref

    CD0 = CD0_fuse + CD0_wing + CD0_ht + CD0_vt

    return CD0

def parasitic_drag_wing(rho, V, mu, t, twl, c, cwl, Lambda_m, x_c, M, l, d, k, S_ref, S_wet_wing, S_wet_wl, S_wet_fuse):

    # CD0 for wings
    Cfw = sf_coeff(rho, V, c, mu, k)
    CD0_wing = Cfw * form_fact_wing(t, Lambda_m, x_c, M) * S_wet_wing / S_ref

    # CD0 for fuselage
    if l != 0:
        Cff = sf_coeff(rho, V, l, mu, k)
        CD0_fuse = Cff * form_fact_wing(d/l, math.radians((30+60+65)/3), x_c, M) * S_wet_fuse / S_ref # Assumes a lambda_m to be an average of 30, 60, and 65 degrees
    else:
        CD0_fuse = 0

    # CD0 for winglets

    Cfwl = sf_coeff(rho, V, cwl, mu, k)
    CD0_wl = Cfwl * form_fact_wing(twl, 0.1, x_c, M) * S_wet_wl / S_ref

    CD0 = CD0_wing + CD0_wl + CD0_fuse

    return CD0
    
def parasitic_drag_nacelle(rho, V, l, mu, k, f, S_ref):
    
    d_n = l/f
    S_wet_n = (math.pi*d_n)*l
    Cfn = sf_coeff(rho, V, l, mu, k)
    CD0_n = Cfn*form_fact_nac(f)*(1.27)* S_wet_n / S_ref

    return CD0_n

def parasitic_drag_winglet(rho, V, cw, mu, k, x_c, M, S_ref, swl):

    S_wet_w = 2.003*swl
    Cfw = sf_coeff(rho, V, cw, mu, k)
    CD0_w = Cfw*form_fact_wing(0.05, 20, x_c, M)* S_wet_w/S_ref

    return CD0_w