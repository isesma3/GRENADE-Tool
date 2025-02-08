import json
import avlwrapper as avl
from math import radians, tan

# Wing Inputs
wing_span = 104
wing_area = 1233.8
wing_aspect_ratio = wing_span ** 2 / wing_area
wing_taper = 1
wing_le_sweep = radians(0)
wing_dihedral = radians(0)
wing_root_le_pnt = avl.Point(2.5, 0, 0)
aspan = 0.25  # fraction of wing with aileron

# Wing Calculations

wing_root_chord = 2 * wing_span / (wing_aspect_ratio * (1 + wing_taper))
wing_tip_chord = wing_root_chord * wing_taper
wing_ail_chord = wing_root_chord * (wing_taper + (1 - wing_taper) * aspan)

aileron = avl.Control(name='aileron',
                      gain=1.0,
                      x_hinge=0.7,
                      duplicate_sign=-1)

wing_tip_le_pnt = avl.Point(x=0.5 * wing_span * tan(wing_le_sweep) + wing_root_le_pnt[0],
                            y=0.5 * wing_span + wing_root_le_pnt[1],
                            z=0.5 * wing_span * tan(wing_dihedral) + wing_root_le_pnt[2])

wing_ail_le_pnt = avl.Point(x=0.5 * (1 - aspan) * wing_span * tan(wing_le_sweep) + wing_root_le_pnt[0],
                            y=0.5 * (1 - aspan) * wing_span + wing_root_le_pnt[1],
                            z=0.5 * (1 - aspan) * wing_span * tan(wing_dihedral) + wing_root_le_pnt[2])

root_section = avl.Section(leading_edge_point=wing_root_le_pnt,
                           chord=wing_root_chord,
                           airfoil=avl.NacaAirfoil('2414'))
ail_section = avl.Section(leading_edge_point=wing_ail_le_pnt,
                          chord=wing_ail_chord,
                          airfoil=avl.NacaAirfoil('2411'),
                          controls=[aileron])
tip_section = avl.Section(leading_edge_point=wing_tip_le_pnt,
                          chord=wing_tip_chord,
                          airfoil=avl.NacaAirfoil('2410'),
                          controls=[aileron])

# y_duplicate=0.0 duplicates the wing over a XZ-plane at Y=0.0
wing = avl.Surface(name='wing',
                   n_chordwise=10,
                   chord_spacing=avl.Spacing.cosine,
                   n_spanwise=10,
                   span_spacing=avl.Spacing.cosine,
                   y_duplicate=0.0,
                   sections=[root_section, ail_section, tip_section])

# ## The tail surface
# The tail surface will have an elevator.
# The elevator will be created as a `Control` object which will be given to the sections.


# HTail inputs
ht_span = 22
ht_area = 154
ht_taper = 1
ht_sweep = radians(0)
ht_dihedral = radians(0)
ht_root_le_pnt = avl.Point(39, 0, 0)

# Htail Calculations

ht_root_chord = 2 * ht_area / (ht_span * (1 + ht_taper))
ht_tip_chord = ht_root_chord * ht_taper

elevator = avl.Control(name='elevator',
                       gain=1.0,
                       x_hinge=0.7,
                       duplicate_sign=1)

ht_tip_le_pnt = avl.Point(x=ht_root_le_pnt.x + 0.5 * ht_span * tan(ht_sweep),
                          y=0.5 * ht_span,
                          z=ht_root_le_pnt.z + 0.5 * ht_span * tan(ht_dihedral))

root_section = avl.Section(leading_edge_point=ht_root_le_pnt,
                           chord=ht_root_chord,
                           airfoil=avl.NacaAirfoil('0012'),
                           controls=[elevator])
tip_section = avl.Section(leading_edge_point=ht_tip_le_pnt,
                          chord=ht_tip_chord,
                          airfoil=avl.NacaAirfoil('0012'),
                          controls=[elevator])
horizontal_tail = avl.Surface(name='horizontal_tail',
                              n_chordwise=10,
                              chord_spacing=avl.Spacing.cosine,
                              n_spanwise=10,
                              span_spacing=avl.Spacing.cosine,
                              y_duplicate=0.0,
                              sections=[root_section, tip_section])

# ## The Vtail surface
# The tail surface will have a rudder.
# The rudder will be created as a `Control` object which will be given to the sections.

# VTail Inputs
vt_span = 9.5
vt_area = 66.5
vt_taper = 1
vt_sweep = radians(0)
vt_root_le_pnt = avl.Point(39, 8, 0)

# VTail Calculations

vt_root_chord = 2 * vt_area / (vt_span * (1 + vt_taper))
vt_tip_chord = vt_root_chord * vt_taper

rudder = avl.Control(name='rudder',
                     gain=-1.0,
                     x_hinge=0.7,
                     duplicate_sign=-1)

vt_tip_le_pnt = avl.Point(x=vt_root_le_pnt.x + vt_span * tan(vt_sweep),
                          y=vt_root_le_pnt.y,
                          z=vt_root_le_pnt.z + vt_span)

root_section = avl.Section(leading_edge_point=vt_root_le_pnt,
                           chord=vt_root_chord,
                           airfoil=avl.NacaAirfoil('0012'),
                           controls=[rudder])
tip_section = avl.Section(leading_edge_point=vt_tip_le_pnt,
                          chord=vt_tip_chord,
                          airfoil=avl.NacaAirfoil('0012'),
                          controls=[rudder])
vertical_tail = avl.Surface(name='vertial_tail',
                            n_chordwise=10,
                            chord_spacing=avl.Spacing.cosine,
                            n_spanwise=10,
                            span_spacing=avl.Spacing.cosine,
                            y_duplicate=1,
                            sections=[root_section, tip_section])

# ## Creating the geometry object
# We're going to use the wing area, mean aerodynamic chord and span as the references which AVL will use the normalise the results.

# Case Inputs
mach = 0.3
rho = 2
g0 = 9.81

# Case Calculations
wing_mac = ((2 * wing_root_chord / 3) *
            (1 + wing_taper + wing_taper ** 2) /
            (1 + wing_taper))

wing_area = wing_span ** 2 / wing_aspect_ratio


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
                        surfaces=[wing, horizontal_tail, vertical_tail])

# Set case parameters

base_case = avl.Case(name='sweep', mach=mach, density=rho, gravity=g0)

# Create sweep case to cover AoA range

alphas = list(range(-10, 20, 1))
trim_param = avl.Parameter(name='elevator', setting='Cm', value=0.0)
elevators = list(range(-15, 16, 3))
# all_cases = avl.create_sweep_cases(base_case=base_case,
#                                    parameters=[{'name': 'alpha',
#                                                 'values': alphas},
#                                                {'name': 'elevator',
#                                                 'values': trim_param}])
all_cases = avl.create_sweep_cases(base_case=base_case,
                                   parameters=[{'name': 'alpha',
                                                'values': alphas}])
session = avl.Session(geometry=aircraft, cases=all_cases)

# AVL can only handle 25 cases at once, so let's create partitions.

partitions = avl.partitioned_cases(all_cases)
results = {}
for partition in partitions:
    session = avl.Session(geometry=aircraft, cases=partition)
    results.update(session.run_all_cases())
    # show_treffz(session)

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
CYdr = []
Clb = []
Clp = []
Clr = []
Clda = []
Cldr = []
Cma = []
Cmq = []
Cmde = []
Cnb = []
Cnp = []
Cnr = []
Cnda = []
Cndr = []
for x in results:
    cL = results[x]['Totals']['CLtot']
    cL = results[x]['Totals']['CDtot']

    CL.append(cL)
    CD.append(cL)

    if 'CLelevator' in results[x]['StabilityDerivatives']:
        cLde = results[x]['StabilityDerivatives']['CLelevator']
        cmde = results[x]['StabilityDerivatives']['Cmelevator']
        CLde.append(cLde)
        Cmde.append(cmde)
    if 'CLrudder' in results[x]['StabilityDerivatives']:
        cYdr = results[x]['StabilityDerivatives']['CYrudder']
        cldr = results[x]['StabilityDerivatives']['Clrudder']
        cndr = results[x]['StabilityDerivatives']['Cnrudder']
        CYdr.append(cYdr)
        Cldr.append(cldr)
        Cndr.append(cndr)
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

dp = {'AOA': alphas, 'CL': CL, 'CD': CD}
stab = {'AOA': alphas, 'CLa': CLa, 'CLq': CLq, 'CLde': CLde, 'CYb': CYb, 'CYp': CYp, 'CYr': CYr, 'CYdr': CYdr,
        'Clb': Clb, 'Clp': Clp, 'Clr': Clr, 'Clda': Clda, 'Cldr': Cldr, 'Cma': Cma, 'Cmq': Cmq, 'Cmde': Cmde,
        'Cnb': Cnb, 'Cnp': Cnp, 'Cnr': Cnr, 'Cnda': Cnda, 'Cndr': Cndr
        }
# Write everything to json
# with open('dp3.json', 'w') as f1:
#     f1.write(json.dumps(dp))
#
# with open('stab.json', 'w') as f2:
#     f2.write(json.dumps(stab))

session.show_geometry()