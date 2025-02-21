import math

def q_calc(rho, v):
    q = 0.5 * rho * v ** 2
    return q

def cl_calc(m, g, q, s):
    cl1 = m * g / q / s
    return cl1

def cruise_power(rho, v, cd, s, etap):
    q1 = q_calc(rho, v)
    p = v * q1 * cd * s
    pshaft = p / etap
    return pshaft

def cruise_thrust(rho, v, cd, s):
    q1 = q_calc(rho,v)
    t = q1 * cd * s
    return t

def accelerate(rho, v, cd, s, m, a, etap):
    q1 = q_calc(rho, v)
    p = v * q1 * cd * s + m * a * v
    pshaft = p / etap
    return pshaft

def accelerate_thrust(rho, v, cd, s, m, a):
    q1 = q_calc(rho, v)
    t = v * q1 * cd * s + m * a
    return t

def climb(rho, v, cd, s, m, g0, roc, etap):
    q1 = q_calc(rho, v)
    p = m * g0 * roc + v * q1 * cd * s
    pshaft = p / etap
    return pshaft

def climb_thrust(rho, v, cd, s, m, g0, roc):
    q1 = q_calc(rho,v)
    gamma = math.asin(roc/v)
    t = q1 * cd * s + m * g0 * math.sin(gamma)
    return t

def cruaccclim(rho, v, cd, s, m, g0, roc, a, etap1):
    cruise = cruise_power(rho, v, cd, s, etap1)
    acc = accelerate(rho, v, cd, s, m, a, etap1)
    clim = climb(rho, v, cd, s, m, g0, roc, etap1)
    return cruise, acc, clim

def Prequired(inp_low, inp_cruise, inp_high, cd, S, mtot, g0, etap1, etap2):
    low_cruise = cruise_power(inp_low[0], inp_low[1], cd, S, etap1)
    low_acc = accelerate(inp_low[0], inp_low[1], cd, S, mtot, inp_low[3], etap1)
    low_clim = climb(inp_low[0], inp_low[1], cd, S, mtot, g0, inp_low[2], etap1)

    cruise_cruise = cruise_power(inp_cruise[0], inp_cruise[1], cd, S, etap1)
    cruise_acc = accelerate(inp_cruise[0], inp_cruise[1], cd, S, mtot, inp_cruise[3], etap1)
    cruise_clim = climb(inp_cruise[0], inp_cruise[1], cd, S, mtot, g0, inp_cruise[2], etap1)

    high_cruise = cruise_power(inp_high[0], inp_high[1], cd, S, etap1)
    high_acc = accelerate(inp_high[0], inp_high[1], cd, S, mtot, inp_high[3], etap1)
    high_clim = climb(inp_high[0], inp_high[1], cd, S, mtot, g0, inp_high[2], etap1)

    lst = [low_cruise, low_acc, low_clim, cruise_cruise, cruise_acc, cruise_clim, high_cruise, high_acc, high_clim]
    ind = lst.index(max(lst))
    Label = ['LowCr','LowAcc','LowClb','CruiseCr','CruiseAcc','CruiseClb','HighCr','HighAcc','HighClb']

    if ind % 3 == 0:
        lst[0] = lst[0] * etap1 / etap2 / 0.95
        lst[3] = lst[3] * etap1 / etap2 / 0.95
        lst[6] = lst[6] * etap1 / etap2 / 0.95
    elif ind % 3 == 1:
        lst[1] = lst[1] * etap1 / etap2 / 0.95
        lst[4] = lst[4] * etap1 / etap2 / 0.95
        lst[7] = lst[7] * etap1 / etap2 / 0.95
    else:
        lst[2] = lst[2] * etap1 / etap2 / 0.95
        lst[5] = lst[5] * etap1 / etap2 / 0.95
        lst[8] = lst[8] * etap1 / etap2 / 0.95
    lst[ind] = lst[ind] * 0.95
    mP = max(lst)
    return mP,Label[ind]

def Trequired(inp_low, inp_cruise, inp_high, cd, S, mtot, g0):
    #low_cruise = cruise_power(inp_low[0], inp_low[1], cd, S, etap1)
    low_cruise = cruise_thrust(inp_low[0], inp_low[1], cd, S)
    #low_acc = accelerate(inp_low[0], inp_low[1], cd, S, mtot, inp_low[3], etap1)
    low_acc = accelerate_thrust(inp_low[0], inp_low[1], cd, S, mtot, inp_low[3])
    #low_clim = climb(inp_low[0], inp_low[1], cd, S, mtot, g0, inp_low[2], etap1)
    low_clim = climb_thrust(inp_low[0], inp_low[1], cd, S, mtot, g0, inp_low[2])

    #cruise_cruise = cruise_power(inp_cruise[0], inp_cruise[1], cd, S, etap1)
    cruise_cruise = cruise_thrust(inp_cruise[0], inp_cruise[1], cd, S)

    #cruise_acc = accelerate(inp_cruise[0], inp_cruise[1], cd, S, mtot, inp_cruise[3], etap1)
    cruise_acc = accelerate_thrust(inp_cruise[0], inp_cruise[1], cd, S, mtot, inp_cruise[3])

    #cruise_clim = climb(inp_cruise[0], inp_cruise[1], cd, S, mtot, g0, inp_cruise[2], etap1)
    cruise_clim = climb_thrust(inp_cruise[0], inp_cruise[1], cd, S, mtot, g0, inp_cruise[2])

    high_cruise = cruise_thrust(inp_high[0], inp_high[1], cd, S)
    #high_cruise = cruise_power(inp_high[0], inp_high[1], cd, S, etap1)

    high_acc = accelerate_thrust(inp_high[0], inp_high[1], cd, S, mtot, inp_high[3])
    high_clim = climb_thrust(inp_high[0], inp_high[1], cd, S, mtot, g0, inp_high[2])

    lst = [low_cruise, low_acc, low_clim, cruise_cruise, cruise_acc, cruise_clim, high_cruise, high_acc, high_clim]
    ind = lst.index(max(lst))
    Label = ['LowCr','LowAcc','LowClb','CruiseCr','CruiseAcc','CruiseClb','HighCr','HighAcc','HighClb']

    if ind % 3 == 0:
        lst[0] = lst[0]
        lst[3] = lst[3] 
        lst[6] = lst[6]
    elif ind % 3 == 1:
        lst[1] = lst[1] 
        lst[4] = lst[4] 
        lst[7] = lst[7] 
    else:
        lst[2] = lst[2] 
        lst[5] = lst[5] 
        lst[8] = lst[8]
    lst[ind] = lst[ind]
    mP = max(lst)
    return mP,Label[ind]

