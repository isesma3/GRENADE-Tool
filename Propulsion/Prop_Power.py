def q(rho, v):
    q = 0.5 * rho * v ** 2
    return q


def climb(rho, vstall, cd0, k, cl, m, g0, roc, etap):
    v = 1.5 * vstall  # climb 50% faster than stall speed
    q1 = q(rho, v)
    cd = cd0 + k * cl ** 2
    p = m * g0 * roc + v * q1 * cd
    pshaft = p / etap
    return pshaft


def cruise(rho, v, cd0, k, cl, etap):
    q1 = q(rho, v)
    cd = cd0 + k * cl ** 2
    p = v * q1 * cd
    pshaft = p / etap
    return pshaft
    
def cruise_cd(rho, v, cd0, cl, cdi):
    q1 = q(rho, v)
    cd = cd0 + cdi
    p = v * q1 * cd
    pshaft = p / etap
    return pshaft
    


def accelerate(rho, v, cd0, k, cl, m, a, etap):
    q1 = q(rho, v)
    cd = cd0 + k * cl ** 2
    p = v * q1 * cd + m * a * v
    pshaft = p / etap
    return pshaft


def preq(rhosl, rhomaxalt, vstallsl, vstallmaxalt, vsl, vmaxalt, cd0, k, s, m, g0, roc, a, etap):
    pclimb1 = climb(rhosl, vstallsl, cd0, k, cl(m, g0, q(rhosl, 1.5 * vstallsl), s), m, g0, roc, etap)
    pclimb2 = climb(rhomaxalt, vstallmaxalt, cd0, k, cl(m, g0, q(rhomaxalt, 1.5 * vstallmaxalt), s), m, g0, roc, etap)
    pcruise1 = cruise(rhosl, vsl, cd0, k, cl(m, g0, q(rhosl, vsl), s), etap)
    pcruise2 = cruise(rhomaxalt, vmaxalt, cd0, k, cl(m, g0, q(rhomaxalt, vmaxalt), s), etap)
    pacc1 = accelerate(rhosl, vsl, cd0, k, cl(m, g0, q(rhosl, vsl), s), m, a, etap)
    pacc2 = accelerate(rhomaxalt, vmaxalt, cd0, k, cl(m, g0, q(rhomaxalt, vmaxalt), s), m, a, etap)
    p = max([pclimb1, pclimb2, pcruise1, pcruise2, pacc1, pacc2])
    return p


def cl(m, g, q, s):
    cl1 = m * g / q / s
    return cl1


power = preq(1.225, 1, 30, 25, 50, 60, 0.02, 0.08, 0.3, 200, 9.81, 0.5, 0.1, 0.8)
