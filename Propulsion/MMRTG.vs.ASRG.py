# -*- coding: utf-8 -*-
"""
Created on Mon Jan 30 23:17:56 2023

@author: yassine
"""

def compare_generators(mmrtg_efficiency, asrg_efficiency):
    mmrtg_output = 200 * mmrtg_efficiency #140 is in watt
    asrg_output = 140* asrg_efficiency #140 is in watt
    
    print("MMRTG Output:", mmrtg_output, "W")
    print("ASRG Output:", asrg_output, "W")
    
    if mmrtg_output > asrg_output:
        print("MMRTG is more efficient")
    elif mmrtg_output < asrg_output:
        print("ASRG is more efficient")
    else:
        print("Both generators have the same efficiency")

compare_generators(0.6, 0.7)
