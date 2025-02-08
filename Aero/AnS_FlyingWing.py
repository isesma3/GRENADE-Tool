import os

LOCAL_DIR = os.path.dirname(__file__)
ROOT_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))

import pandas as pd
from math import radians, tan
from math import pi as PI 
import avlwrapper as avl
import numpy as np
from parasitic_drag_func import *


def flywingaero(CASE, mach, rho, mu, v, g0, CGx, Dshell):
    # Wing Inputs
    wing_aspect_ratio = CASE.Wing_AR[0]
    wing_area = CASE.Wing_S[0]
    wing_span = np.sqrt(wing_aspect_ratio * wing_area)
    wing_taper = CASE.Wing_TR[0]
    wing_le_sweep = radians(CASE.Wing_SWP[0])
    wing_dihedral = radians(CASE.Wing_dhl[0])
    wing_root_le_pnt = avl.Point(0, 0, 0)
    wing_root_tc = CASE.Wing_r_tc[0]
    wing_root_xc = CASE.Wing_r_xc[0]
    wing_root_xtc = CASE.Wing_r_xtc[0]
    wing_tip_tc = CASE.Wing_t_tc[0]
    wing_tip_xc = CASE.Wing_t_xc[0]
    wing_tip_xtc = CASE.Wing_t_xtc[0]
    # Winglet/Fin Inputs
    swl = CASE.Wing_fin_S[0]
    winglet_taper = CASE.Wing_fin_TR[0]

    # Fuselage Inputs
    fuse_length = CASE.Fuse_L[0]
    fuse_depth = CASE.Fuse_d[0]

    Swf = PI * fuse_depth * fuse_length  # Estiamted fuselage wetted area

    # Wing Calculations

    if not (CASE.NACA0[0]):
        af0 = os.path.join(ROOT_DIR, 'Airfoils', CASE.Foil0[0] + '.dat')
    if not (CASE.NACA1[0]):
        af1 = os.path.join(ROOT_DIR, 'Airfoils', CASE.Foil1[0] + '.dat')

    wing_root_chord = 2 * wing_span / (wing_aspect_ratio * (1 + wing_taper))
    wing_tip_chord = wing_root_chord * wing_taper

    wing_tip_le_pnt = avl.Point(x=0.5 * wing_span * tan(wing_le_sweep),
                                y=0.5 * wing_span,
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

    # ## Winglet Surface
    # winglets will have rudders.
    converged = 0
    i = 0
    Scale = 5
    step = 0.5
    neg_val = 0
    cnt = 0
    grad_descent = {'swl': [], 'Cnb': []}
    while converged == 0:
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
        
        winglet_root_chord = wing_tip_chord
        winglet_tip_chord = winglet_taper * winglet_root_chord
        winglet_span = winglet_area / (0.5 * (winglet_root_chord - winglet_tip_chord) + winglet_tip_chord)
        # winglet_aspect_ratio = winglet_span ** 2 / winglet_area
        winglet_le_sweep = np.arctan((winglet_root_chord - winglet_tip_chord)/winglet_span)

        winglet_root_le_pnt = avl.Point(x=wing_tip_le_pnt.x, y=wing_tip_le_pnt.y, z=wing_tip_le_pnt.z)

        rudder = avl.Control(name='rudder',
                             gain=1,
                             x_hinge=0.7,
                             duplicate_sign=-1)

        winglet_tip_le_pnt = avl.Point(x=winglet_root_le_pnt.x + winglet_span * tan(winglet_le_sweep),
                                       y=winglet_root_le_pnt.y,
                                       z=winglet_root_le_pnt.z + winglet_span)

        root_section_wl = avl.Section(leading_edge_point=winglet_root_le_pnt,
                                      chord=winglet_root_chord,
                                      airfoil=avl.NacaAirfoil('0010'),
                                      controls=[rudder])
        tip_section_wl = avl.Section(leading_edge_point=winglet_tip_le_pnt,
                                     chord=winglet_tip_chord,
                                     airfoil=avl.NacaAirfoil('0008'),
                                     controls=[rudder])
        # y_duplicate=0.0 duplicates the winglet over a XZ-plane at Y=0.0
        winglet = avl.Surface(name='winglet',
                              n_chordwise=10,
                              chord_spacing=avl.Spacing.cosine,
                              n_spanwise=10,
                              span_spacing=avl.Spacing.cosine,
                              y_duplicate=0.0,
                              component=1,
                              sections=[root_section_wl, tip_section_wl])

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

        # Currently this is setting the referece point to the 1/4 chord of the MAC
        # We may want to change that to CG as per AVL documentation

        aircraft = avl.Aircraft(name='aircraft',
                                reference_area=wing_area,
                                reference_chord=wing_mac,
                                reference_span=wing_span,
                                reference_point=ref_pnt,
                                mach=mach,
                                surfaces=[wing, winglet])

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
        l = fuse_length  # Have to include value for fuselage length due to skin friction def. Making it this small for flying wing
        d = fuse_depth  # Same as above. Minute diameter of fuselage
        # Define path to the TitanAtm output

        # Depending on if we want to consider the root thickness and chord or tip thickness and chord, we can comment out whatever is necessary
        # There is no explicit way of integrating the pd from root to tip. We might have to apply some interpolation.
        cd0 = wing_parasitic_drag(rho, v, mu, wing_root_tc, wing_mac, wing_le_sweep, l, d, k, wing_area, mach, wing_root_xtc,
                             2.1 * (wing_area + 2 * winglet_area), Swf)

        # AVL can only handle 25 cases at once, so let's create partitions.

        partitions = avl.partitioned_cases(all_cases)
        results = {}
        for partition in partitions:
            session = avl.Session(geometry=aircraft, cases=partition)
            results.update(
                session.run_all_cases())  # on some computers this keeps writing "Table values missing. Replaced with NaN" to the console

        Cnb = []
        for x in results:
            if 'Cnb' in results[x]['StabilityDerivatives'].keys():
                cnb = results[x]['StabilityDerivatives']['Cnb']
            else:
                cnb = 0
                converged = 1
                yaw_check = -2
                if x == 'sweep-29':
                    print('Error: Cnb not found as a value in Stability Derivatives')
            Cnb.append(cnb)

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

        YawRange = [0.065, 0.095]
        YawMid = 0.5 * (YawRange[0] + YawRange[1])
        grad_descent['swl'].append(swl)
        grad_descent['Cnb'].append(meanCnb)
        if i == 0 or neg_val == 1:
            if meanCnb >= 0.095:
                swl -= min([step, Scale * (meanCnb - (YawRange[1] + YawMid) * 0.5)])
            elif meanCnb <= 0.065:
                swl += min([step, Scale * (0.5 * (YawRange[0] + YawMid) - meanCnb)])
        else:
            xs = grad_descent['swl'][-2:]
            ys = grad_descent['Cnb'][-2:]
            slope = (ys[1]-ys[0])/(xs[1]-xs[0])
            swl = (YawMid-ys[1])/slope + xs[1]
        i += 1

        if YawRange[0] <= meanCnb <= YawRange[1]:
            converged = 1
            yaw_check = 1

        if winglet_span > Dshell * 2:
            converged = 1
            yaw_check = 0
            print('Failure: Winglet Span Exceeds Constraint')
        if i > 45:
            converged = 1
            yaw_check = -1
            # print('Failure: Winglet sizing did not converge within 45 iterations')
    # Pull out needed information from results
    # Currently making a drag polar and stability and control derivatives
    CL = []
    CD = []
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
        cD = results[x]['Totals']['CDtot'] + cd0

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
    dp = {'AOA': alphas, 'CL': CL, 'CD': CD}
    # stab = {'AOA': alphas, 'CLa': CLa, 'CLq': CLq, 'CLde': CLde, 'CYb': CYb, 'CYp': CYp, 'CYr': CYr,
    #         'Clb': Clb, 'Clp': Clp, 'Clr': Clr, 'Clda': Clda, 'Cma': Cma, 'Cmq': Cmq, 'Cmde': Cmde,
    #         'Cnb': Cnb, 'Cnp': Cnp, 'Cnr': Cnr, 'Cnda': Cnda
    #         }
    # session.show_geometry()
    struc = {'station': struc_y, 'ShearF': struc_V, 'BendM': struc_M}
    return (dp, swl, winglet_root_le_pnt.x + winglet_root_chord * 0.666, SM, npt, struc, yaw_check, pitch_check)
