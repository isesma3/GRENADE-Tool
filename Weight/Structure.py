# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# File Name: Structure.py                                 
# Creat Date: February 16, 2023                        
# Contributors (In-Order): Daniel J. Moore                    
# Last Edited: February 20, 2023                    
# Last Edited By: Daniel Moore                          
# For: Aerospace Systems
#  Design Lab @ Georgia Tech     
# Description: 
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Functions:                                          
#   NACA_4Digit_Coord()     - NACA 4 digit surface coordinates
#   Airfoil_File(FOIL,x)    - return surface coordinate
#   SkinPanel()             - Beam bending structural weight estimate
# 
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Version Sumamry
#   1 - Initial Commit add panel node generation
#   2 - Add Structure function
#   3 - Update function to steamline parameter processing and incorporate wing sweep and varying chord
#        - (current version)
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Additional Notes
#
#
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~




# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Import Libraries
import os
LOCAL_DIR = os.path.dirname(__file__)
ROOT_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))

import pandas as pd
from scipy.interpolate import interp1d
import numpy as np
import math
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def station_c(cr,ct,b,y):
    '''
    Inputs:
        cr  - root chord length
        ct  - tip chord length
        b   - wingspan
        y   - distance spanwise from root
    Outputs:
        ci  - chord at semispan station
    '''
    ci = (2/b)*(ct-cr)*y+cr
    return ci

def station_dx(cr,ct,b,y,swp,f):
    '''
    Inputs:
        cr  - root chord length
        ct  - tip chord length
        b   - wingspan
        y   - distance spanwise from root
        swp - wing sweep [deg]
        f   - fraction of chord where sweep measured
    Outputs:
        dx  - longitudinal shift in leading edge position
    '''
    dx = np.multiply(f*(cr-ct),1-(2/b)*y) + np.tan(math.pi*swp/180.)*y
    return dx

def NACA_4Digit_Coord(Foil,x,ULonly:bool):
    '''
    Inputs:
        Foil - four digit airfoil name (ex: 2412, 0008,...)
        x - x-axis coordinates to evaluate at, as fraction of chord
    Outputs:
        coord   - array of x and y pairs defining airfoil [x_input, camber line, thickness distribution, x_upper, x_lower, y_upper, y_lower]
    '''
    D = [int(d) for d in str(Foil)] # Get individual airfoil digits
    # NACA 2412 Airfoil Properties
    m = D[0]/100    # camber
    p = D[1]/10     # max camber position
    t = int(str(D[2])+str(D[3]))/100    # max t/c
    
    # Evaluate Foil thickness and camber line
    yt = 5*t*(0.2969*np.sqrt(x)-0.1267*x-0.3523*np.square(x) + 0.2843*np.power(x,3) - 0.1022*np.power(x,4)) # Thickness distribution, with zero thickness trailing edge
    yc = np.zeros(x.shape)
    theta = np.zeros(x.shape)
    i = 0
    for xi in x:
        if xi <= p:
            if p == 0:
                theta[i] = math.pi/2
                yc[i] = 0
            else:
                yc[i] = (m/np.square(p))*(2*p*xi - np.square(xi))
                theta[i] = np.arctan((2*m/np.square(p)) * (p-xi))
        else:
            yc[i] = (m/np.square(1-p))*((1-2*p)+(2*p*xi) - np.square(xi))
            if p == 1:
                theta[i] = math.pi/2
            else:
                theta[i] = np.arctan((2*m/np.square(1-p))*(p-xi))
        i += 1

    # Upper and Lower Surface Coordinates
    xU = x - np.multiply(yt,np.sin(theta))
    xL = x + np.multiply(yt,np.sin(theta))
    yU = yc + np.multiply(yt,np.cos(theta))
    yL = yc - np.multiply(yt,np.cos(theta))

    # Return Coordinates
    if ULonly:
        coord = np.column_stack((xU,xL,yU,yL))
    else:
        coord = np.column_stack((x,yc,yt,xU,xL,yU,yL))
    return coord

def Airfoil_File(FOIL,x):
    '''
    Inputs:
        FOIL    - name of airfoil dat file (Selig format), must be placed in ./Aero/Airfoils/
        x       - x-axis coordinates to evaluate at, as fraction of chord
    Outputs:
        coord   - array of airfoil upper and lower surface coordinates [x_upper, x_lower, y_upper, y_lower]
    '''
    lines = np.loadtxt(os.path.join(ROOT_DIR,'Aero/Airfoils/'+FOIL+'.dat'),skiprows=1)
    #coord = np.column_stack((x,yc,yt,xU,xL,yU,yL))
    min = np.argmin(lines, axis=0)
    # Normalize on 0 to 1
    minv = np.amin(lines, axis=0)[0]
    maxv = np.amax(lines, axis=0)[0]
    i = 0
    for xi in np.transpose(lines)[0]:
        lines[i][0] = (xi-minv)*(1/(maxv-minv))
        i += 1

    # Assume no trailing edge thickness
    if lines[0][1] != lines[-1][1]:
        lines[0][1] = np.mean([lines[0][1],lines[-1][1]])
        lines[-1][1] = np.mean([lines[0][1],lines[-1][1]])
    # Evaluate upper and lower surface coordinates
    Lower = interp1d(np.transpose(lines[int(min[0]):])[0],np.transpose(lines[int(min[0]):])[1], fill_value="extrapolate")
    Upper = interp1d(np.transpose(lines[:min[0]+1])[0],np.transpose(lines[:min[0]+1])[1], fill_value="extrapolate")
    yU = Upper(x)
    yL = Lower(x)

    # Return upper and lower surface coordinates
    coord = np.column_stack((x,x,yU,yL))
    return coord

def Airfoil_Properties(FOIL,NACA):
    dx = np.linspace(0,1,1000)
    if NACA:
        a = NACA_4Digit_Coord(FOIL,dx,False)
        camber = interp1d(a[:,0],a[:,1],'cubic')
        thickness = interp1d(a[:,0],a[:,2],'cubic')
    else:
        a = Airfoil_File(FOIL,dx) # Return airfoil upper and lower surface
        camber = interp1d(a[:,0],np.mean(a[:,2:],axis=1),'cubic')
        thickness = interp1d(a[:,0],a[:,-2]-a[:,-1],'cubic')
    c_x = camber(dx)
    t_x = thickness(dx)
    # Evaluate max camber and t/c
    cam = np.amax(c_x)
    tc = np.amax(t_x)
    # Find indices for maximums
    it = np.where(tc == t_x)[0][0]
    ic = np.where(cam == c_x)[0][0]
    # Locate Maximum positions
    xtc = dx[it]
    xc = dx[ic]

    return tc,xtc,cam,xc

def SkinPanel(Data,Struc):
    '''
    Inputs:
        Data    - Dataframe of input parameters
        sturc   - Dataframe of structure inputs (stations, shear forces, bending moments)
    Outputs:
        Val     - array [total mass [kg], bending mass [kg], shear mass [kg], bending thickness [m], shearing thickness [m]]

    Assumptions:
        - Upper and Lower contour skin thicknesses the same
        - Shear material doesn't contribute to centroid/moment of intertia
        - Wing Sweep measured at leading edge
    '''
    # Import Parameters
    Nlim = Data.Nz[0]         # Limit Load Factor for Design
    st_z = Struc.struc_st.to_numpy()    # Load positions along semispan
    F_z = Struc.struc_F.to_numpy()*Nlim      # Shear forces along semispan
    M_z = Struc.struc_M.to_numpy()*Nlim      # Bending moments along semispan
    b = Data.Wing_b[0]      # Span [m]
    cr = Data.Wing_cr[0]     # Root chord [m]
    ct = Data.Wing_ct[0]     # Tip chord [m]
    swp = Data.Wing_Sweep[0]  # Wing Sweep [deg]
    Panels = Data.Panels[0] # Number of panels on wing
    sig_ult = Data.Ult_n[0]   # Ultimate normal stress [Pa]
    tau_ult = Data.Ult_s[0]   # Ultimate shear stress [Pa]
    NACA4 = Data.NACA0[0]    # Boolean, True if root airfoil 4 digit NACA
    NACA42 = Data.NACA1[0]   # Boolean, True if tip airfoil 4 digit NACA
    Airfoil = Data.Foil0[0]  # Either NACA4 digit name or name of Selig coordinate .dat file for root
    Airfoil2 = Data.Foil1[0] # Either NACA4 digit name or name of Selig coordinate .dat file for root
    FOS = Data.FOS[0]       # Factor of Safety
    rho_bend = Data.rho_b[0] # Density of material under bending load
    rho_shear = Data.rho_s[0] # Density of material under shear load
    g = Data.g[0]               #gravitational acceleration

    # Initialize Arrays
    t_bend = np.zeros(st_z.shape)
    ts = np.zeros(st_z.shape)
    mBend = np.zeros(st_z.shape)
    mShear = np.zeros(st_z.shape)
    mnet = np.zeros(st_z.shape)
    x_webU = np.zeros(st_z.shape)
    x_webL = np.zeros(st_z.shape)
    Fvec = np.zeros((st_z.shape[0],3))
    cm = np.zeros((st_z.shape[0],3))

    tol = 0.001
    k = 0
    mguess = 1
    conv = 1
    Nx = 0
    Ny = 1
    while conv > tol and k <= 1000:
        for i,mloc in enumerate(mnet):
            Fvec[i] =  np.array([-mloc*g*Nx + np.sum(-mnet[i+1:]*g*Nx),F_z[i],0])
        j = 0
        for pos_z in st_z:
            if pos_z == st_z[-1]:
                db = 0
                M = [-(M_z[j] - 0.5*mnet[j]*g*Ny*db),(-0.5*mnet[j]*g*db) + 0,0]
            else:
                db = (st_z[j+1]-pos_z)
                M = [-(M_z[j] - 0.5*mnet[j]*g*Ny*db),(-0.5*mnet[j]*g*db) + np.sum(np.multiply(Fvec[j+1:,0],st_z[j+1:]-pos_z)),0]
            F = Fvec[j]
            c = station_c(cr,ct,b,pos_z)
            dx_c = station_dx(cr,ct,b,pos_z,swp,0)
            pFM = [c/4 + dx_c,0,pos_z]


            # Panel Node Coordinates
            cnt = round(Panels/2)+1
            x_vec = np.linspace(0,1,cnt)


            if NACA4:
                coord = NACA_4Digit_Coord(Airfoil,x_vec,True)
            else:
                coord = Airfoil_File(Airfoil,x_vec)
            if NACA42:
                coord2 = NACA_4Digit_Coord(Airfoil2,x_vec,True)
            else:
                coord2 = Airfoil_File(Airfoil2,x_vec)
            frac = (2/b)*pos_z
            coord = np.add(np.multiply(1-frac,coord),np.multiply(frac,coord2))  # Interpolate airfoil
            coord = c*coord # scale airfoil by local chord
            xU_n = np.transpose(coord[:,0]) + dx_c
            xL_n = np.transpose(coord[:,1]) + dx_c
            yU_n = np.transpose(coord[:,2])
            yL_n = np.transpose(coord[:,3])

            # Evaluate the length of the web, b
            bU = np.sqrt(np.square(xU_n[1:]-xU_n[:-1]) + np.square(yU_n[1:]-yU_n[:-1]))
            bL = np.sqrt(np.square(xL_n[1:]-xL_n[:-1]) + np.square(yL_n[1:]-yL_n[:-1]))

            betaU = np.arctan(np.divide((yU_n[1:]-yU_n[:-1]),(xU_n[1:]-xU_n[:-1])))     # The sign of this angle may be incorrect, but subsequent calculations make this a moot point
            betaL = np.arctan(np.divide((yL_n[1:]-yL_n[:-1]),(xL_n[1:]-xL_n[:-1])))
            # Panel Midpoint
            xmU = (xU_n[1:]+xU_n[:-1])/2
            xmL = (xL_n[1:]+xL_n[:-1])/2
            ymU = (yU_n[1:]+yU_n[:-1])/2
            ymL = (yL_n[1:]+yL_n[:-1])/2

            # Evaluate centroid coord and define x', y', etc.
            xc = (np.sum(np.multiply(bU,xmU)) + np.sum(np.multiply(bL,xmL)))/(np.sum(bU)+np.sum(bL))
            yc = (np.sum(np.multiply(bU,ymU)) + np.sum(np.multiply(bL,ymL)))/(np.sum(bU)+np.sum(bL))
            zc = 0

            xU_np = xU_n - xc
            yU_np = yU_n - yc
            xL_np = xL_n - xc
            yL_np = yL_n - yc

            xmU_p = (xU_np[1:]+xU_np[:-1])/2
            xmL_p = (xL_np[1:]+xL_np[:-1])/2
            ymU_p = (yU_np[1:]+yU_np[:-1])/2
            ymL_p = (yL_np[1:]+yL_np[:-1])/2

            # Calculate Normalized Moments of Inertia (I/thickness)
            Ixxt_U = (1/12)*np.multiply(np.power(bU,3),np.square(np.sin(betaU))) + np.multiply(bU,np.square(ymU_p))
            Ixxt_L = (1/12)*np.multiply(np.power(bL,3),np.square(np.sin(betaL))) + np.multiply(bL,np.square(ymL_p))

            Iyyt_U = (1/12)*np.multiply(np.power(bU,3),np.square(np.cos(betaU))) + np.multiply(bU,np.square(xmU_p))
            Iyyt_L = (1/12)*np.multiply(np.power(bL,3),np.square(np.cos(betaL))) + np.multiply(bL,np.square(xmL_p))

            Ixyt_U = (1/24)*np.multiply(np.power(bU,3),np.sin(2*betaU)) + np.multiply(bU,np.multiply(xmU_p,ymU_p))
            Ixyt_L = (1/24)*np.multiply(np.power(bL,3),np.sin(2*betaL)) + np.multiply(bL,np.multiply(xmL_p,ymL_p))

            Ixxt = np.sum(Ixxt_U) + np.sum(Ixxt_L)
            Iyyt = np.sum(Iyyt_U) + np.sum(Iyyt_L)
            Ixyt = np.sum(Ixyt_U) + np.sum(Ixyt_L)

            # Calculate Moments about Centroid
            Mx_p = M[0] + F[1] * (zc-pFM[2]) - F[2] * (yc-pFM[1])
            My_p = M[1] - F[0] * (zc-pFM[2]) - F[2] * (xc-pFM[0])
            Mz_p = M[2] + F[0] * (yc-pFM[1]) - F[1] * (xc-pFM[0])

            # Calculate Normalized Axial Stress for Each Panel
            sig_zt = np.zeros(cnt-1)
            for i in range(0,cnt-1):
                sig_zt[i] = (Mx_p*(Iyyt*ymU_p[i] - Ixyt*xmU_p[i])/(Ixxt*Iyyt-np.square(Ixyt))) + (My_p*(Ixxt*xmU_p[i] - Ixyt*ymU_p[i])/(Ixxt*Iyyt-np.square(Ixyt))) + F[2]/np.sum(bU[i] + bL[i])

            # Calculate Bending Skin Thickness
            t_bend[j] = np.amax(np.abs(sig_zt))*(FOS/sig_ult)

            # Calculate Shear Material Thickness
            iU = np.argmax(yU_n)
            iL = np.argmax(yL_n)
            h_wb = np.max(yU_n) - np.max(yL_n)
            x_webU[j] = xU_n[iU]
            x_webL[j] = xL_n[iL]
            if pos_z == st_z[-1]:
                Lwb = 0
                dz = 0
            else:
                Lwb = 1
                dz = 1
            if j != 0:
                mShear[j-1] = mShear[j-1]*np.sqrt(np.square(np.mean([x_webU[j],x_webL[j]]) - np.mean([x_webU[j-1],x_webL[j-1]])) + np.square(pos_z-st_z[j-1]))
                mBend[j-1] = mBend[j-1]*(pos_z-st_z[j-1])
            S = F
            I = np.multiply([Ixxt,Iyyt,Ixyt],t_bend[j])
            if I[0] == 0 and I[1] == 0 and I[2] == 0:
                q = 0
            else:
                q = ((S[0]*I[0]-S[1]*I[2])/(I[0]*I[1]-np.square(I[2])))*np.sum(t_bend[j]*np.multiply(bU,xmU_p)) - ((S[1]*I[1]-S[0]*I[2])/(I[0]*I[1]-np.square(I[2])))*np.sum(t_bend[j]*np.multiply(bU,ymU_p))

            ts[j] = np.abs(q)*FOS/tau_ult

            # Calculate Bending Material and Shear Material Weight
            mBend[j] = rho_bend*t_bend[j]*dz*(np.sum(bU) + np.sum(bL))
            mShear[j] = rho_shear * ts[j] * Lwb * h_wb
            mnet[j] = mBend[j] + mShear[j]

            # Specify rough center of mass [m]
            if mBend[j] == 0 and mShear [j] == 0:
                cm[j][0] = (sum(xmU) + sum(xmL))/(len(xmU) + len(xmL))
                cm[j][1] = (sum(ymU) + sum(ymL))/(len(ymU) + len(ymL))
            else:
                cm[j][0] = (mBend[j] * (sum(xmU) + sum(xmL))/(len(xmU) + len(xmL)) + mShear[j] * 0.5*(x_webU[j] + x_webL[j]+ 2*xc))/(mBend[j] + mShear[j])
                cm[j][1] = (mBend[j] * (sum(ymU) + sum(ymL))/(len(ymU) + len(ymL)) + mShear[j] * 0.5*(yU_n[iU] +   yL_n[iL]+ 2*yc))/(mBend[j] + mShear[j])
            cm[j][2] = pos_z
            j += 1
        conv = abs(mguess - np.sum(mnet))
        mguess = np.sum(mnet)
        k += 1
    cg = [0,0,0]
    cg[0] = np.sum(np.multiply(0.5*np.add(mBend[:-1] + mShear[:-1],mBend[1:] + mShear[1:]),0.5*np.add(cm[:-1,0],cm[1:,0])))/np.sum(0.5*np.add(mBend[:-1] + mShear[:-1],mBend[1:] + mShear[1:]))
    #cg[2] = np.sum(np.multiply(0.5*np.add(mBend[:-1] + mShear[:-1],mBend[1:] + mShear[1:]),0.5*np.add(cm[:-1][1],cm[1:][1])))/np.sum(0.5*np.add(mBend[:-1] + mShear[:-1],mBend[1:] + mShear[1:]))
    cg[1] = np.sum(np.multiply(0.5*np.add(mBend[:-1] + mShear[:-1],mBend[1:] + mShear[1:]),0.5*np.add(cm[:-1,2],cm[1:,2])))/np.sum(0.5*np.add(mBend[:-1] + mShear[:-1],mBend[1:] + mShear[1:]))
    Val = [2*sum(mnet),cg]
    return Val





###### Test Code #######
#coord = NACA_4Digit_Coord(2412,np.linspace(0,1,num=11))
#print(coord)
#lines = Airfoil_File("e342",np.linspace(0,1,num=33))
#print(lines)