import avlwrapper as avl
from math import radians, tan

def aero(b,s,tr,sweep,dihedral,wingx0,bht,sht,trht,sweepht,dihedralht,htx0,bvt,svt,trvt,sweepvt,vtx0,vty0,mach,rho,g0):
    # Wing Inputs
    wing_span = b
    wing_area = s
    wing_aspect_ratio = wing_span ** 2 / wing_area
    wing_taper = tr
    wing_le_sweep = radians(sweep)
    wing_dihedral = radians(dihedral)
    wing_root_le_pnt = avl.Point(wingx0, 0, 0)

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

    # ## The tail surface
    # The tail surface will have an elevator.
    # The elevator will be created as a `Control` object which will be given to the sections.


    # HTail inputs
    ht_span = bht
    ht_area = sht
    ht_taper = trht
    ht_sweep = radians(sweepht)
    ht_dihedral = radians(dihedralht)
    ht_root_le_pnt = avl.Point(htx0, 0, 0)

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
    vt_span = bvt
    vt_area = svt
    vt_taper = trvt
    vt_sweep = radians(sweepvt)
    vt_root_le_pnt = avl.Point(vtx0, vty0, 0)

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
                                y_duplicate=0.0,
                                sections=[root_section, tip_section])

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

    # AVL can only handle 25 cases at once, so let's create partitions.

    partitions = avl.partitioned_cases(all_cases)
    results = {}
    for partition in partitions:
        session = avl.Session(geometry=aircraft, cases=partition)
        results.update(session.run_all_cases())
        # show_treffz(session)


    # Pull out needed information from results
    # Currently making a drag polar
    CL = []
    CD = []
    for x in results:
        cl = results[x]['Totals']['CLtot']
        cd = results[x]['Totals']['CDtot']
        CL.append(cl)
        CD.append(cd)

    print(results)
    
    dp = {'AOA': alphas, 'CL': CL, 'CD': CD}
    return dp
