#Thermal Equations
#Created by Jacob Webb
#Contributors: Jacob Webb, Yassine Fouchal
#Last Updated: Feb 19, 2023
# T=∜(S(α/ε)(A_p/A_r )/σ) # Temperture due to solar/cosmic radiation
# Q=(k/s)A(deltaT) # Conduction
# q''=h_bar(T_inf-T_body) #convection equation, where h or h_bar (W/m^2-K) is 25-250 for subsonic flow
# qdot''=-k(T2-T1)/(x2-x1) #heat flux for conduction along thickness x
# Qdot=mdot*c_p(T_out-T_in)
# 1=absorptivity+reflectance+transmittance #absorptivity = alpha, reflectance = rho, transmittance = tau
# q'' = emissivity*σ*T^4 #Radiation heat flux Eq 1
# q'' = absorptivity*σ*T^4 #Radiation heat flux Eq 2
# qdot_net''=emissivity*σ*T_surface^4-absorbtivity*σ*T_surroundings^4 #Radiation heat flux Eq 3
# qdot_net''=emissivity*σ*(T_surface^4-T_surroundings^4) # If surface is assumed to be gray
# T_o^n+qdot_conv''(2*deltat/(rho*c_p*deltax))=(1+(2k*deltat/(rho*c_p*(deltax)^2)))*T_o^(n+1)-(2k*deltat/(rho*c_p*(deltax)^2))*T_1^(n+1)# Finite Difference Equations for the 1-D Transient in-depth Energy Equation

def solar_radiation(sigma,solarConstant,absorbtivity,emissivity,A_p,A_r):
    temperature = (solarConstant*(absorbtivity/emissivity)*(A_p/A_r)/sigma)**(1/4)



    return temperature

def conduction_heat_transfer(Q, k, s, deltaT):
    return Q / (k/s) / deltaT

def convection_heat_transfer(q, h_bar, T_inf, T_body):
    return q / h_bar + T_body - T_inf

def conduction_heat_flux(qdot, k, T2, T1, x2, x1):
    return qdot * (x2 - x1) / (-k * (T2 - T1))

def heat_transfer_rate(Qdot, mdot, c_p, T_out, T_in):
    return Qdot / (mdot * c_p) - T_in + T_out

def radiation_heat_flux_eq1(q, emissivity, T):
    sigma = 5.67e-8  # Stefan-Boltzmann constant
    return q / (emissivity * sigma * T ** 4)

def radiation_heat_flux_eq2(q, absorptivity, T):
    sigma = 5.67e-8  # Stefan-Boltzmann constant
    return q / (absorptivity * sigma * T ** 4)

def radiation_heat_flux_eq3(qdot_net, emissivity, T_surface, T_surroundings):
    sigma = 5.67e-8  # Stefan-Boltzmann constant
    return qdot_net / (emissivity * sigma * (T_surface ** 4 - T_surroundings ** 4))

def finite_difference_equation(T_o_n, qdot_conv, deltat, rho, c_p, deltax, k, T_1):
    n=1
    return (1 + 2 * k * deltat / (rho * c_p * deltax ** 2)) * T_o_n + qdot_conv * (2 * deltat / (rho * c_p * deltax)) - (2 * k * deltat / (rho * c_p * deltax ** 2)) * T_1**(n+1)

