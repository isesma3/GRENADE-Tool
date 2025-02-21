import os

LOCAL_DIR = os.path.dirname(__file__)
ROOT_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))


import pandas as pd
from math import radians, tan
from math import pi as PI 
import avlwrapper as avl
import numpy as np
from parasitic_drag_func import *
from Wing_AVL import *
from Inv_Vtail_AVL import *
from V_tail_AVL import *
from Winglet_rudder_AVL import *
from Conventional_tail_AVL import *
from T_tail_AVL import *
from H_tail_AVL import *
from Winglet_AVL import *
from Lifting_Fuselage_AVL import *

# Aerodynamics and stability code
# Utilizes different functions to analyze aero and stability for different tail types
# In the future can add different aircraft configs instead of just doing a wing analysis

def AnS(CASE, mach, rho, mu, v, g0, CGx, Dshell, lb, Tail_type, Winglet, fuse_L, fuse_D):
    # Wing setup
    if Tail_type == 'No_tail': # Set wing sweep to 30 degrees if flying wing
        CASE.loc[0, 'Wing_Sweep'] = 30
    [wing_root_chord, wing_tip_chord, wing_taper, wing_root_le_pnt, wing_tip_le_pnt, wing_area, 
     wing_span, wing, wing_c_bar, wing_root_xtc, wing_root_tc, wing_le_sweep, b] = AVLwing(CASE, fuse_D)
    
    # Lifting fuselage setup
    fuselage = AVL_fuselage(CASE, fuse_L, fuse_D)
    
    if Winglet == 'yes':
        [winglet, winglet_mac, b_winglet, 
         winglet_tip_le_pnt, winglet_root_chord, swl] = Winglet_AVL(wing_tip_chord, wing_tip_le_pnt,b)
        
    # V-tail Surface
    # V-tail will have control surfaces
    S_Htail = CASE.Ht_S[0]
    S_Vertail = CASE.Vt_S[0]
    S_Vtail = S_Htail+S_Vertail
    if Tail_type == 'No_tail':
        S_Vertail = 0.01
    converged = 0
    i = 0
    Scale = 1
    step = 0.01
    neg_val = 0
    cnt = 0
    grad_descent = {'S_Vertail': [], 'Cnb': []}
    grad_ascent = {'S_Htail': [], 'SM': []}
    S_tail_check = 0
    while converged == 0:
    # The rest of the loop just builds the aircraft in AVL and then iterates until the tail satisfies stability requirements    
        if Tail_type == 'Boom-mounted_inverted_V':
            #print('V-tail area: ', S_Vtail)
            #print('Horizontal tail area: ', S_Htail)
            #print('Vertical tail area: ', S_Vertail)
            [BMI_Vtail, Vtail_mac, b_tail, tail_tip_le_pnt, 
            tail_root_chord, cnt, neg_val] = Inv_Vtail(CASE,S_Vtail,S_Htail,S_Vertail,cnt,neg_val, lb)
            tail = BMI_Vtail
        elif Tail_type == 'Inverted_V':
            #print('V-tail area: ', S_Vtail)
            #print('Horizontal tail area: ', S_Htail)
            #print('Vertical tail area: ', S_Vertail)
            [I_Vtail, Vtail_mac, b_tail, tail_tip_le_pnt, 
            tail_root_chord, cnt, neg_val] = Inv_Vtail(CASE,S_Vtail,S_Htail,S_Vertail,cnt,neg_val, fuse_L ,fuse_D)
            tail = I_Vtail
        elif Tail_type == 'V-tail':
            #print('V-tail area: ', S_Vtail)
            #print('Horizontal tail area: ', S_Htail)
            #print('Vertical tail area: ', S_Vertail)
            [Vtail, Vtail_mac, b_tail, tail_tip_le_pnt, 
            tail_root_chord, cnt, neg_val] = V_tail(CASE,S_Vtail,S_Htail,S_Vertail,cnt,neg_val, fuse_L,fuse_D)
            tail = Vtail
        elif Tail_type == 'H-tail':
            #print('Horizontal tail area: ', S_Htail)
            #print('Vertical tail area: ', S_Vertail)
            [Htail, Htail_mac, b_tail, tail_tip_le_pnt, tail_root_chord, Vertail, Vertail_mac, b_Vertail, 
            Vertail_tip_le_pnt, Vertail_root_chord, cnt, neg_val] = H_tail(CASE,S_Htail,S_Vertail,cnt,neg_val,fuse_L,fuse_D)
        elif Tail_type == 'T-tail':
            #print('Horizontal tail area: ', S_Htail)
            #print('Vertical tail area: ', S_Vertail)
            [Htail, Htail_mac, b_tail, tail_tip_le_pnt, tail_root_chord, Vertail, Vertail_mac, b_Vertail, 
            Vertail_tip_le_pnt, Vertail_root_chord, cnt, neg_val] = T_tail(CASE,S_Htail,S_Vertail,cnt,neg_val,fuse_L,fuse_D)
        elif Tail_type == 'Conventional':
            #print('Horizontal tail area: ', S_Htail)
            #print('Vertical tail area: ', S_Vertail)
            [Htail, Htail_mac, b_tail, tail_tip_le_pnt, tail_root_chord, Vertail, Vertail_mac, b_Vertail, 
            Vertail_tip_le_pnt, Vertail_root_chord, cnt, neg_val] = Conv_tail(CASE,S_Htail,S_Vertail,cnt,neg_val,fuse_L,fuse_D)
        elif Tail_type == 'No_tail':
            #print('Winglet area: ', S_Vertail)
            [winglet_rudd, winglet_rudd_mac, b_tail, tail_tip_le_pnt, 
            tail_root_chord, cnt, neg_val] = Winglet_rudd(wing_tip_chord, wing_tip_le_pnt,cnt,neg_val,S_Vertail)
            tail = winglet_rudd
        
        # ## Creating the geometry object
        # We're going to use the wing area, mean aerodynamic chord and span
        # as the references which AVL will use to normalise the results.

        # Case Calculations
        wing_mac = ((2 * wing_root_chord / 3) *
                    (1 + wing_taper + wing_taper ** 2) /
                    (1 + wing_taper))

        # calculate the m.a.c. leading edge location
        def mac_le_pnt(root_chord, tip_chord, root_pnt, tip_pnt):
            pnt = ((2 * root_chord * root_pnt[dim] +
                    root_chord * tip_pnt[dim] +
                    tip_chord * root_pnt[dim] +
                    2 * tip_chord * tip_pnt[dim]) /
                (3 * (root_chord + tip_chord))
                for dim in range(3))
            return avl.Point(*pnt)

        le_pnt = mac_le_pnt(wing_root_chord, wing_tip_chord,
                            wing_root_le_pnt, wing_tip_le_pnt)

        ref_pnt = avl.Point(CGx, y=le_pnt.y, z=le_pnt.z)
        # ref_pnt = avl.Point(x=le_pnt.x + 0.25 * wing_mac,
        #                     y=le_pnt.y, z=le_pnt.z)

        #Currently this is setting the referece point to the 1/4 chord of the MAC
        #We may want to change that to CG as per AVL documentation
        if Winglet == 'yes' and Tail_type != 'No_tail':
            if Tail_type == 'Conventional' or Tail_type == 'T-tail' or Tail_type == 'H-tail':
                aircraft = avl.Aircraft(name='aircraft',
                                    reference_area=wing_area,
                                    reference_chord=wing_mac,
                                    reference_span=wing_span,
                                    reference_point=ref_pnt,
                                    mach=mach,
                                    surfaces=[wing, fuselage, Htail, Vertail, winglet])
            else:
                aircraft = avl.Aircraft(name='aircraft',
                                    reference_area=wing_area,
                                    reference_chord=wing_mac,
                                    reference_span=wing_span,
                                    reference_point=ref_pnt,
                                    mach=mach,
                                    surfaces=[wing, fuselage, tail, winglet])
        else:
            if Tail_type == 'Conventional' or Tail_type == 'T-tail' or Tail_type == 'H-tail':
                aircraft = avl.Aircraft(name='aircraft',
                                    reference_area=wing_area,
                                    reference_chord=wing_mac,
                                    reference_span=wing_span,
                                    reference_point=ref_pnt,
                                    mach=mach,
                                    surfaces=[wing, fuselage, Htail, Vertail])
            else:
                aircraft = avl.Aircraft(name='aircraft',
                                    reference_area=wing_area,
                                    reference_chord=wing_mac,
                                    reference_span=wing_span,
                                    reference_point=ref_pnt,
                                    mach=mach,
                                    surfaces=[wing, fuselage, tail])

        # Set case parameters

        base_case = avl.Case(name='sweep', mach=mach, velocity=v, density=rho, gravity=g0)

        # Create sweep case to cover AoA range

        alphas = list(range(-10, 20, 1))
        # trim_param = avl.Parameter(name='elevator', setting='Cm', value=0.0)
        # elevators = list(range(-15, 16, 3))
        # all_cases = avl.create_sweep_cases(base_case=base_case,
        #                                    parameters=[{'name': 'alpha',
        #                                                 'values': alphas},
        #                                                {'name': 'elevator',
        #                                                 'values': trim_param}])
        all_cases = avl.create_sweep_cases(base_case=base_case,
                                        parameters=[{'name': 'alpha',
                                                        'values': alphas}])

        # Raymer defines Lambda_m as the sweep of the max thickness line
        k = 0.052e-5  # Value for carbon-fiber
        l = fuse_L  # Have to include value for fuselage length due to skin friction def. Making it this small for flying wing
        d = fuse_D  # Same as above. Minute diameter of fuselage

        # Calculate wetted areas
        th = np.tan((wing_root_chord-wing_tip_chord)/(wing_span/2))
        Cx = wing_root_chord + (wing_span/2 - fuse_D/2)*np.arctan(th)
        SwExp = 0.5*(wing_span-fuse_D)*(Cx + wing_tip_chord)
        SwWing = SwExp*(1.977+0.52*wing_root_xtc) # Wing wetted area
        Swf = fuse_L*fuse_D*(1.977+0.52*(fuse_D/fuse_L)) # Fuselage wetted area for bwb
        if Tail_type == 'Conventional' or Tail_type == 'T-tail' or Tail_type == 'H-tail':
            SwVertail = 2.003*S_Vertail
            SwHtail = 2.003*S_Htail
            cd0 = parasitic_drag_tail(rho, v, mu, wing_root_tc, 0.08, 0.08, wing_mac, tail_root_chord, Vertail_root_chord, wing_le_sweep, l, d, k, wing_area, mach, wing_root_xtc,
                                SwWing, SwHtail, SwVertail, Swf)
        elif Tail_type == 'Boom-mounted_inverted_V':
            SwVtail = 2.003*S_Vtail # VTail wetted area
            cd0 = parasitic_drag_boom(rho, v, mu, wing_root_tc, b, wing_mac ,wing_le_sweep, tail_root_chord, 0.08, l, d, lb, k, wing_area, mach, wing_root_xtc, SwWing, SwVtail, Swf)
        elif Tail_type == 'Inverted_V' or Tail_type == 'V-tail':
            SwVtail = 2.003*S_Vtail # VTail wetted area
            cd0 = parasitic_drag_Vtail(rho, v, mu, wing_root_tc, 0.08, wing_mac, Vtail_mac ,wing_le_sweep, l, d, k, wing_area, mach, wing_root_xtc, SwWing, SwVtail, Swf)
        else:
            Swwl = 2.003*S_Vertail
            cd0 = parasitic_drag_wing(rho, v, mu, wing_root_tc, 0.08, wing_mac, winglet_rudd_mac, wing_le_sweep, wing_root_xtc, mach, l, d, k, wing_area, SwWing, Swwl, Swf)
        
        # Calculate parasitic drag due to propulsors
        # cd0_prop = CASE.PropN[0]*parasitic_drag_nacelle(rho, v, 0.3, mu, k, 15, wing_area)

        # Calculate the parasitic drag due to the winglets
        if Winglet == 'yes':
            cd0_winglet = parasitic_drag_winglet(rho, v, winglet_mac, mu, k, wing_root_xtc, mach, wing_area, swl)

        # AVL can only handle 25 cases at once, so let's create partitions.
        partitions = avl.partitioned_cases(all_cases)
        results = {}
        for partition in partitions:
            session = avl.Session(geometry=aircraft, cases=partition)
            #session.show_geometry()
            results.update(
                session.run_all_cases())  # on some computers this keeps writing "Table values missing. Replaced with NaN" to the console
        Cnb = []
        Cma = []
        CLa = []
        for x in results:
            cLa = results[x]['StabilityDerivatives']['CLa']
            if 'Cma' in results[x]['StabilityDerivatives'].keys():
                cma = results[x]['StabilityDerivatives']['Cma']
            else:
                cma = 0
                converged = 1
                pitch_check = 0
                print('Error: Cma not found as a value in Stability Derivatives')
            if 'Cnb' in results[x]['StabilityDerivatives'].keys():
                cnb = results[x]['StabilityDerivatives']['Cnb']
            else:
                cnb = 0
                converged = 1
                yaw_check = -2
                if x == 'sweep-29':
                    print('Error: Cnb not found as a value in Stability Derivatives')
            Cnb.append(cnb)
            Cma.append(cma)
            CLa.append(cLa)

        # ind1 = alphas.index(-3)
        # ind2 = alphas.index(7)
        # meanCnb = np.mean(Cnb[ind1:ind2])
        meanCnb = Cnb[10]
        # print(Cnb)
        # print(meanCnb)

        # if i == 5:
        #     step = 0.1
        # if i == 50:
        #     Scale = 1
        #     step = 0.01
        #     print('Reducing Winglet/Fin Area Step size')
        # elif i == 100:
        #     Scale = 0.1
        #     step = 0.001
        #     print('Reducing Winglet/Fin Area Step size')
        # elif i == 300:
        #     Scale = 0.01
        #     step = 0.001
        #     print('Reducing Winglet/Fin Area Step size')
        # i += 1
        #
        # if meanCnb >= 0.095:
        #     swl -= min([step, Scale * (meanCnb - (YawRange[1] + YawMid) * 0.5)])
        # elif meanCnb <= 0.065:
        #     swl += min([step, Scale * (0.5 * (YawRange[0] + YawMid) - meanCnb)])
        # else:
        #     converged = 1
        #     yaw_check = 1

        # Vertical and Horizontal tail sizer
        YawRange = [0.03, 0.2] # Found new range for Yaw coefficient at https://www.sciencedirect.com/topics/engineering/yawing-moment
        YawMid = 0.5 * (YawRange[0] + YawRange[1])
        grad_descent['S_Vertail'].append(S_Vertail)
        grad_descent['Cnb'].append(meanCnb)
        # SMrange = [0.05,0.5]
        # SMmid = 0.5*(SMrange[0] + SMrange[1])
        # grad_ascent['S_Htail'].append(S_Htail)
        # SM = -Cma[10] / CLa[10]
        # grad_ascent['SM'].append(SM)
        if neg_val == 0:
        # if SM >= SMrange[1]:
        #     S_Htail -= min([step, abs(Scale * (SM - (SMrange[1] + SMmid) * 0.5))])
        #     if S_Htail <= 0:
        #         S_Htail = step
        #     CASE.Ht_S = S_Htail
        # elif SM <= SMrange[0]:
        #     S_Htail += min([step, abs(Scale * (0.5 * (SMrange[0] + SMmid) - SM))])
        #     CASE.Ht_S = S_Htail
            if meanCnb >= YawRange[1]:
                S_Vertail -= min([step, abs(Scale * (meanCnb - (YawRange[1] + YawMid) * 0.5))])
                if S_Vertail <= 0:
                    S_Vertail = step
                CASE.Vt_S = S_Vertail
            elif meanCnb <= YawRange[0]:
                S_Vertail += min([step, abs(Scale * (0.5 * (YawRange[0] + YawMid) - meanCnb))])
                CASE.Vt_S = S_Vertail
        elif meanCnb >= YawRange[1] or meanCnb <= YawRange[0]:
            xs = grad_descent['S_Vertail'][-2:]
            ys = grad_descent['Cnb'][-2:]
            if (xs[1]-xs[0]) == 0:
                xs[1] = 1
                xs[0] = 0
                converged = 1
                print("Tail sizing failure")
                yaw_check = 0
            slope = (ys[1]-ys[0])/(xs[1]-xs[0])
            if slope == 0 or xs[1] == 0:
                slope = 1
                converged = 1
                print("Tail sizing failure")
                yaw_check = 0
            S_Vertail = (YawMid-ys[1])/slope + xs[1]
            CASE.Vt_S = S_Vertail
        # elif SM >= SMrange[1] or SM <= SMrange[0]:
        #     xss = grad_ascent['S_Htail'][-2:]
        #     yss = grad_ascent['SM'][-2:]
        #     if xss[1]-xss[0] 
        #     slopes = (yss[1]-yss[0])/(xss[1]-xss[0])
        #     S_Htail = (SMmid-yss[1])/slopes + xss[1]
        #     CASE.Ht_S = S_Htail
        i += 1
        
        b_tail_check = 0
        if b_tail > 0.75*b: #If the tail becomes too big, break out of the loop
            converged = 1
            print('Failure: Tail span greater than Wing span')
            yaw_check = 0
            b_tail_check = 1

        # if SM < 0:
        #     converged = 1
        #     print('Failure: Negative Static Margin')
        #     struc = {}
        #     yaw_check = -1
        #     pitch_check = -1

        if S_Htail < 0  or S_Vertail < 0:
            converged = 1
            struc = {}
            yaw_check = -1
            pitch_check = -1
            S_tail_check = -1

        if YawRange[0] <= meanCnb <= YawRange[1]:# and SMrange [0] <= SM <= SMrange[1]:
            converged = 1
            yaw_check = 1
            pitch_check = 1
            S_tail_check = 1

        if b_tail > Dshell * 2:
            converged = 1
            yaw_check = 0
            print('Failure: Tail Span Exceeds Constraint')
        if i > 5:
            converged = 1
            yaw_check = -1
            print('Failure: Tail sizing did not converge within 5 iterations')
    # Pull out needed information from results
    # Currently making a drag polar and stability and control derivatives
    CL = []
    CD = []
    cl = []
    AoAs = []
    CLa = []
    # CLq = []
    # CLde = []
    # CYb = []
    # CYp = []
    # CYr = []
    # CYdr = []
    # Clb = []
    # Clp = []
    # Clr = []
    # Clda = []
    # Cldr = []
    Cma = []
    # Cmq = []
    # Cmde = []
    Cnb = []
    # Cnp = []
    # Cnr = []
    # Cnda = []
    # Cndr = []
    struc_y = []
    struc_V = []
    struc_M = []
    check = 0
    for x in results:
        cL = results[x]['Totals']['CLtot']
        if Winglet == 'yes':
            cD = results[x]['Totals']['CDtot'] + cd0 + cd0_winglet + cd0_prop
            cd0_tot = cd0 + cd0_winglet
        else:
            cD = results[x]['Totals']['CDtot'] + cd0 + cd0_prop
            cd0_tot = cd0
        
        # Drag due to the varying # of propulsors is available 
        # but not yet added because there is not yet a way to quantify the benefit of multiple propulsors
        # But, to add drag due to propulsors, just add the term cd0_prop to the variable cD
        cls = results[x]['StripForces']['wing']['cl']
        aoas = results[x]['Totals']['Alpha']
        AoAs.append(aoas)
        cl.append(cls)
        CL.append(cL)
        CD.append(cD)

        # if 'CLelevator' in results[x]['StabilityDerivatives']:
        #     cLde = results[x]['StabilityDerivatives']['CLelevator']
        #     cmde = results[x]['StabilityDerivatives']['Cmelevator']
        #     CLde.append(cLde)
        #     Cmde.append(cmde)
        # if 'CLaileron' in results[x]['StabilityDerivatives']:
        #     clda = results[x]['StabilityDerivatives']['Claileron']
        #     cnda = results[x]['StabilityDerivatives']['Cnaileron']
        #     Clda.append(clda)
        #     Cnda.append(cnda)
        # if 'CLrudder' in results[x]['StabilityDerivatives']:
        #     cYdr = results[x]['StabilityDerivatives']['CYrudder']
        #     cldr = results[x]['StabilityDerivatives']['Clrudder']
        #     cndr = results[x]['StabilityDerivatives']['Cnrudder']
        #     CYdr.append(cYdr)
        #     Cldr.append(cldr)
        #     Cndr.append(cndr)

        cLa = results[x]['StabilityDerivatives']['CLa']
        # cLq = results[x]['StabilityDerivatives']['CLq']
        # cYb = results[x]['StabilityDerivatives']['CYb']
        # cYp = results[x]['StabilityDerivatives']['CYp']
        # cYr = results[x]['StabilityDerivatives']['CYr']
        # clb = results[x]['StabilityDerivatives']['Clb']
        # clp = results[x]['StabilityDerivatives']['Clp']
        # clr = results[x]['StabilityDerivatives']['Clr']
        if 'Cma' in results[x]['StabilityDerivatives'].keys():
            cma = results[x]['StabilityDerivatives']['Cma']
            pitch_check = 1
        else:
            cma = 0
            converged = 1
            pitch_check = 0
            # print('Error: Cma not found as a value in Stability Derivatives')
        # cmq = results[x]['StabilityDerivatives']['Cmq']
        if 'Cnb' in results[x]['StabilityDerivatives'].keys():
            cnb = results[x]['StabilityDerivatives']['Cnb']
        else:
            cnb = 0
        # cnp = results[x]['StabilityDerivatives']['Cnp']
        # cnr = results[x]['StabilityDerivatives']['Cnr']

        Sref = results[x]['Totals']['Sref']
        bref = results[x]['Totals']['Bref']
        q = 0.5 * rho * v ** 2

        w_y = [val * (bref / 2) for val in results[x]['StripShearMoments']['wing']['2Y/Bref']]
        w_Vz = [val * (q * Sref) for val in results[x]['StripShearMoments']['wing']['Vz/(q*Sref)']]
        w_Mx = [val * (q * bref * Sref) for val in results[x]['StripShearMoments']['wing']['Mx/(q*Bref*Sref)']]

        CLa.append(cLa)
        # CLq.append(cLq)
        # CYb.append(cYb)
        # CYp.append(cYp)
        # CYr.append(cYr)
        # Clb.append(clb)
        # Clp.append(clp)
        # Clr.append(clr)
        Cma.append(cma)
        # Cmq.append(cmq)
        Cnb.append(cnb)
        # Cnp.append(cnp)
        # Cnr.append(cnr)

        if check == 0:
            struc_y.append(w_y)
            struc_V.append(w_Vz)
            struc_M.append(w_Mx)
        elif struc_M[0] < w_Mx[0]:
            struc_M = w_Mx

    # meanCLa = np.mean(CLa[ind1:ind2])
    # meanCma = np.mean(Cma[ind1:ind2])
    # SM = -meanCma/meanCLa # Static Margin average over linear region
    SM = -Cma[10] / CLa[10]  # Static Margin at AoA = 0
    npt = SM * wing_mac + CGx
    dp = {'AOA': AoAs, 'CL': CL, 'CD': CD, 'cl': cl, 'Cma': Cma, 'CLa': CLa}
    # stab = {'AOA': alphas, 'CLa': CLa, 'CLq': CLq, 'CLde': CLde, 'CYb': CYb, 'CYp': CYp, 'CYr': CYr,
    #         'Clb': Clb, 'Clp': Clp, 'Clr': Clr, 'Clda': Clda, 'Cma': Cma, 'Cmq': Cmq, 'Cmde': Cmde,
    #         'Cnb': Cnb, 'Cnp': Cnp, 'Cnr': Cnr, 'Cnda': Cnda
    #         }
    #session.show_geometry()
    struc = {'station': struc_y, 'ShearF': struc_V, 'BendM': struc_M}
    return (dp, S_Vtail, tail_tip_le_pnt.x + tail_root_chord * 0.666, SM, npt, struc, yaw_check, pitch_check,b_tail_check, S_tail_check,cd0_tot)