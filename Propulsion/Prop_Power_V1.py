def q_calc(rho, v):
    q = 0.5 * rho * v ** 2
    return q

def cl_calc(m, g, q, s):
    cl1 = m * g / q / s
    return cl1

def cruise(rho, v, cd0, k, cl, s, etap):
    q1 = q_calc(rho, v)
    cd = cd0 + k * cl ** 2
    p = v * q1 * cd * s
    pshaft = p / etap
    return pshaft

def cruise_cd(rho, v, cd0, cdi, s, etap):
    q1 = q_calc(rho, v)
    cd = cd0 + cdi
    p = v * q1 * cd * s
    pshaft = p / etap
    return pshaft
