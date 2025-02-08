import os
LOCAL_DIR = os.path.dirname(__file__)
ROOT_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))

import sys
sys.path.append(ROOT_DIR)

import pandas as pd
from math import radians, tan
import avlwrapper as avl
from parasitic_drag_func import *
from Environment.TitanAtm import *


def flywingaero(b, s, tr, sweep, t, dihedral, swl, arwl, trwl, mach, rho, mu, v, g0, CGx):

    # Wing Inputs
    wing_span = b
    wing_area = s
    wing_aspect_ratio = wing_span ** 2 / wing_area
    wing_taper = tr
    wing_le_sweep = radians(sweep)
    wing_dihedral = radians(dihedral)
    wing_root_le_pnt = avl.Point(0, 0, 0)

    # Wing Calculations

    wing_root_chord = 2 * wing_span / (wing_aspect_ratio * (1 + wing_taper))
    wing_tip_chord = wing_root_chord * wing_taper

    wing_tip_le_pnt = avl.Point(x=0.5 * wing_span * tan(wing_le_sweep),
                                y=0.5 * wing_span,
                                z=0.5 * wing_span * tan(wing_dihedral))

    root_section = avl.Section(leading_edge_point=wing_root_le_pnt,
                               chord=wing_root_chord,
                               airfoil=avl.NacaAirfoil('2414'))
    tip_section = avl.Section(leading_edge_point=wing_tip_le_pnt,
                              chord=wing_tip_chord,
                              airfoil=avl.NacaAirfoil('2410'))

    # y_duplicate=0.0 duplicates the wing over a XZ-plane at Y=0.0
    wing = avl.Surface(name='wing',
                       n_chordwise=10,
                       chord_spacing=avl.Spacing.cosine,
                       n_spanwise=10,
                       span_spacing=avl.Spacing.cosine,
                       y_duplicate=0.0,
                       sections=[root_section, tip_section])

    # ## Winglet Surface
    # winglets will have rudders elevator.
    winglet_aspect_ratio = arwl
    winglet_taper = trwl
    converged = 0
    while converged == 0:
        winglet_area = swl
        winglet_span = np.sqrt(winglet_aspect_ratio * winglet_area)
        winglet_root_chord = 2 * winglet_span / (winglet_aspect_ratio * (1 + winglet_taper))
        winglet_tip_chord = winglet_root_chord * winglet_taper
        winglet_le_sweep = np.arctan(winglet_span / (winglet_root_chord - winglet_tip_chord))

        wingletx = wing_tip_le_pnt.x + wing_tip_chord - winglet_root_chord
        winglet_root_le_pnt = avl.Point(x=wingletx, y=wing_tip_le_pnt.y, z=wing_tip_le_pnt.z)

        winglet_tip_le_pnt = avl.Point(x=winglet_root_le_pnt.x + winglet_span / tan(winglet_le_sweep),
                                       y=winglet_root_le_pnt.y,
                                       z=winglet_root_le_pnt.z + winglet_span)

        root_section_wl = avl.Section(leading_edge_point=winglet_root_le_pnt,
                                      chord=winglet_root_chord,
                                      airfoil=avl.NacaAirfoil('0010'))
        tip_section_wl = avl.Section(leading_edge_point=winglet_tip_le_pnt,
                                     chord=winglet_tip_chord,
                                     airfoil=avl.NacaAirfoil('0008'))

        # y_duplicate=0.0 duplicates the winglet over a XZ-plane at Y=0.0
        winglet = avl.Surface(name='winglet',
                              n_chordwise=10,
                              chord_spacing=avl.Spacing.cosine,
                              n_spanwise=10,
                              span_spacing=avl.Spacing.cosine,
                              y_duplicate=0.0,
                              sections=[root_section_wl, tip_section_wl])

        # ## Creating the geometry object
        # We're going to use the wing area, mean aerodynamic chord and span
        # as the references which AVL will use to normalise the results.

        # Case Inputs
        mach = mach
        rho = rho
        g0 = g0

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

        # ref_pnt = avl.Point(CGx, y=le_pnt.y, z=le_pnt.z)
        ref_pnt = avl.Point(x=le_pnt.x + 0.25 * wing_mac,
                            y=le_pnt.y, z=le_pnt.z)

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

        #Raymer defines Lambda_m as the sweep of the max thickness line
        x_c = 0.4     #Max camber location based on NACA 2412/2410 airfoil
        k = 0.052e-5   #Value for carbon-fiber
        l = 0.0001     #Have to include value for fuselage length due to skin friction def. Making it this small for flying wing
        d = 0.0000001   #Same as above. Minute diameter of fuselage
        root_thickness = 0.12*wing_root_chord     #Thickness based on NACA 2412
        tip_thickness = 0.1*wing_tip_chord        #Thickness based on NACA 2410
        #Define path to the TitanAtm output
        df = pd.read_csv(ROOT_DIR + '/Flight_Requirements/Flight_Requirements.csv')

        #DEpending on if we want to consider the root thickness and chord or tip thickness and chord, we can comment out whatever is necessary
        #There is no explicit way of integrating the pd from root to tip. We might have to apply some interpolation.
        root_pd = wing_parasitic_drag(rho, v, mu, t, wing_root_chord, wing_le_sweep, wing_mac, d, k, wing_area, mach, x_c, 2.1*wing_area)
        tip_pd = wing_parasitic_drag(rho, v, mu, t, wing_tip_chord, wing_le_sweep, wing_mac, d, k, wing_area, mach, x_c, 2.1*wing_area)
        # print(root_pd,tip_pd)
        cd0 = np.mean([root_pd, tip_pd])


        # AVL can only handle 25 cases at once, so let's create partitions.

        partitions = avl.partitioned_cases(all_cases)
        results = {}
        for partition in partitions:
            session = avl.Session(geometry=aircraft, cases=partition)
            results.update(session.run_all_cases())
            # show_treffz(session)
        Cnb = []
        for x in results:
            cnb = results[x]['StabilityDerivatives']['Cnb']
            Cnb.append(cnb)

        ind1 = alphas.index(-3)
        ind2 = alphas.index(7)
        # meanCnb = np.mean(Cnb[ind1:ind2])
        meanCnb = Cnb[10]
        # print(Cnb)
        # print(meanCnb)

        if meanCnb >= 0.95:
            swl -= 0.005
        elif meanCnb <= 0.065:
            swl += 0.005
        else:
            converged = 1
    # Pull out needed information from results
    # Currently making a drag polar and stability and control derivatives
    CL = []
    CD = []
    CLa = []
    CLq = []
    CLde = []
    CYb = []
    CYp = []
    CYr = []
    Clb = []
    Clp = []
    Clr = []
    Clda = []
    Cma = []
    Cmq = []
    Cmde = []
    Cnb = []
    Cnp = []
    Cnr = []
    Cnda = []
    struc_y = []
    struc_V = []
    struc_M = []
    check = 0
    for x in results:
        cL = results[x]['Totals']['CLtot']
        cD = results[x]['Totals']['CDtot'] + cd0

        CL.append(cL)
        CD.append(cD)

        if 'CLelevator' in results[x]['StabilityDerivatives']:
            cLde = results[x]['StabilityDerivatives']['CLelevator']
            cmde = results[x]['StabilityDerivatives']['Cmelevator']
            CLde.append(cLde)
            Cmde.append(cmde)
        if 'CLaileron' in results[x]['StabilityDerivatives']:
            clda = results[x]['StabilityDerivatives']['Claileron']
            cnda = results[x]['StabilityDerivatives']['Cnaileron']
            Clda.append(clda)
            Cnda.append(cnda)

        cLa = results[x]['StabilityDerivatives']['CLa']
        cLq = results[x]['StabilityDerivatives']['CLq']
        cYb = results[x]['StabilityDerivatives']['CYb']
        cYp = results[x]['StabilityDerivatives']['CYp']
        cYr = results[x]['StabilityDerivatives']['CYr']
        clb = results[x]['StabilityDerivatives']['Clb']
        clp = results[x]['StabilityDerivatives']['Clp']
        clr = results[x]['StabilityDerivatives']['Clr']
        cma = results[x]['StabilityDerivatives']['Cma']
        cmq = results[x]['StabilityDerivatives']['Cmq']
        cnb = results[x]['StabilityDerivatives']['Cnb']
        cnp = results[x]['StabilityDerivatives']['Cnp']
        cnr = results[x]['StabilityDerivatives']['Cnr']

        Sref = results[x]['Totals']['Sref']
        bref = results[x]['Totals']['Bref']
        q = 0.5 * rho * v ** 2

        w_y = [val * (bref / 2) for val in results[x]['StripShearMoments']['wing']['2Y/Bref']]
        w_Vz = [val * (q * Sref) for val in results[x]['StripShearMoments']['wing']['Vz/(q*Sref)']]
        w_Mx = [val * (q * bref * Sref) for val in results[x]['StripShearMoments']['wing']['Mx/(q*Bref*Sref)']]

        CLa.append(cLa)
        CLq.append(cLq)
        CYb.append(cYb)
        CYp.append(cYp)
        CYr.append(cYr)
        Clb.append(clb)
        Clp.append(clp)
        Clr.append(clr)
        Cma.append(cma)
        Cmq.append(cmq)
        Cnb.append(cnb)
        Cnp.append(cnp)
        Cnr.append(cnr)

        if check == 0:
            struc_y.append(w_y)
            struc_V.append(w_Vz)
            struc_M.append(w_Mx)
        elif struc_M[0] < w_Mx[0]:
            struc_M = w_Mx

    meanCLa = np.mean(CLa[ind1:ind2])
    meanCma = np.mean(Cma[ind1:ind2])
    SM = -meanCma/meanCLa
    dp = {'AOA': alphas, 'CL': CL, 'CD': CD}
    stab = {'AOA': alphas, 'CLa': CLa, 'CLq': CLq, 'CLde': CLde, 'CYb': CYb, 'CYp': CYp, 'CYr': CYr,
            'Clb': Clb, 'Clp': Clp, 'Clr': Clr, 'Clda': Clda, 'Cma': Cma, 'Cmq': Cmq, 'Cmde': Cmde,
            'Cnb': Cnb, 'Cnp': Cnp, 'Cnr': Cnr, 'Cnda': Cnda
            }
    session.show_geometry()
    struc = {'station': struc_y, 'ShearF': struc_V, 'BendM': struc_M}
    return (dp, swl, SM, struc)



