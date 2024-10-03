import numpy as np  
import time 
from math import *   

L_Ctl = -1   
R_Ctl = 1   
Cmd_scale = 20.0          
kcontrol = 1              # command: 1.5 for running, 2 for climbing  


def rad2deg(rad):
    deg = rad*180.0/pi
    return deg

def deg2rad(deg):
    rad = deg*pi/180.0
    return rad

def saturate(Cmd,sat):
    if Cmd>sat: 
        Cmd=sat
    if Cmd<-sat:
        Cmd=-sat
    return Cmd   

def impedance_control(
    Kp=0.0, 
    Kd=0.0, 
    ref_pos=0.0, ref_vel=0.0, 
    real_pos=0.0, real_vel=0.0,  
    tau_ff=0.0  
):  
    Cmd_tau = 0.0  
    Cmd_tau = (ref_pos - real_pos) * Kp + (ref_vel - real_vel) * Kd + tau_ff  

    return Cmd_tau      

def smooth(old_value=None, value=None):  
    smoothed_value = 0.7 * old_value + 0.3 * value   
    return smoothed_value   