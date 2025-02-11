#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Nov 16 10:19:53 2024

@author: parks
"""

#mass estimating relationships - university of maryland
#https://spacecraft.ssl.umd.edu/academics/791S16/791S16L08.MERsx.pdf

import numpy as np

def rocketPropulsion(T_req, Isp, g, M, gamma):
    mdot = T_req / (Isp * g)
    Wdot = mdot * g
    v_eq = Isp * g
    A_ratio = ((gamma+1)/2)**(-1*(gamma+1)/(2*gamma-2)) * ((1 + (gamma-1)/2*M**2)/M)**((gamma+1)/(2*(gamma-1)))
    T_req_N = T_req * 4.448222
    m_eng_kg = 7.81 * 10**-4 * T_req_N + 3.37 * 10**-5 * T_req_N * A_ratio**0.5 + 59
    #size fuel tanks
    
    return Wdot, m_eng_kg

    
