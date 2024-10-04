import ReadIMU_old as ReadIMU
import time
from DNN_old import DNN
import datetime
import numpy as np
import kqExoskeletonIO as kqio
import datetime as dt
import RPi.GPIO as GPIO
from math import *

def SendCmdTorque(cmd1, cmd2):
    Ant.Cmd.Loop_L  = kqio.TOR_LOOP
    Ant.Cmd.Loop_R  = kqio.TOR_LOOP
    Ant.Cmd.Value_L = cmd1
    Ant.Cmd.Value_R = cmd2
    
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

ComPort = '/dev/ttyUSB0'
imu = ReadIMU.READIMU(ComPort)
dnn = DNN(18,128,64,2)

start = time.time()
now   = 0
t_pr1 = 0
t_pr2 = 0
t_pr3 = 0

L_Cmd = 0
R_Cmd = 0
pk    = 0

date  = time.localtime(time.time())
dateh = date.tm_hour
datem = date.tm_min
dates = date.tm_sec

def write_csv(V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12,V13):
    with open(str(dateh)+str(datem)+str(dates)+".csv", "a") as log:
        log.write("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}\n".format(str(V1),str(V2),str(V3),str(V4),str(V5),str(V6),str(V7),str(V8),str(V9),str(V10),str(V11),str(V12),str(V13)))

with open(str(dateh)+str(datem)+str(dates)+".csv", "a") as log:
    log.write("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}\n".format("t",
                                                                                "L_Cmd","R_Cmd",
                                                                                "L_tau","R_tau",
                                                                                "L_IMU_angle","R_IMU_angle",
                                                                                "L_IMU_vel","R_IMU_vel",
                                                                                "L_encoder", "R_encoder",
                                                                                "L_encoder_vel", "R_encoder_vel"))


print("Initializing the comunication with the Exoskeleton")
GetSec = kqio.GetSec
Ant = kqio.AntCH("/dev/ttyAMA0") # This is the comport that connects the Raspberry Pi 4 to the LEO
Ant.Cmd.CmdMode  = kqio.CMD_SERVO_OVERRIDE
StartSec         = GetSec()
UpdateSec        = StartSec
UpdateState      = Ant.ComState
UpdateSuccessSec = StartSec
AntConnected     = (Ant.ComState == 1)

ComTotalCnt = 1
ComErrorCnt = 0
print("Succesful initialization")

print("Getting the initial angular position values for the encoders")
# The follwing sign definition follow the rigth hand side rule assuming that the rotation axis is pointing outside of the exoskeleton (for each motor)
StartHipAngle_L = -rad2deg(Ant.Data.HipAngle_L)
StartHipAngle_R = rad2deg(Ant.Data.HipAngle_R)


while(AntConnected):
    
    if UpdateState == 1:
        UpdateSuccessSec = GetSec()

        now = (time.time()-start)
        imu.read()
        imu.decode()
        
        L_tau = Ant.Data.HipTor_L
        R_tau = Ant.Data.HipTor_R
        
        L_encoder     = -rad2deg(Ant.Data.HipAngle_L) - StartHipAngle_L
        R_encoder     = rad2deg(Ant.Data.HipAngle_R) - StartHipAngle_R
        L_encoder_vel = -rad2deg(Ant.Data.HipSpeed_L)
        R_encoder_vel = rad2deg(Ant.Data.HipSpeed_R)

        L_IMU_angle = imu.XIMUL
        R_IMU_angle = imu.XIMUR
        L_IMU_vel   = imu.XVIMUL
        R_IMU_vel   = imu.XVIMUR

        if (now - t_pr3 > 3): # Time to reset peak torque printed to terminal
            t_pr3 = now
            pk = 0

        if (now - t_pr1 > 0.025):
            t_pr1 = now
            kp = 10
            kd = 400

            dnn.generate_assistance(L_IMU_angle, R_IMU_angle, L_IMU_vel, R_IMU_vel,kp,kd)

            kcontrol = 0.05 # 1.5 para running. 2 para climbing.
            
            L_Cmd     = dnn.hip_torque_L*kcontrol
            R_Cmd     = dnn.hip_torque_R*kcontrol
            L_Cmd_sat = saturate(L_Cmd,6)
            R_Cmd_sat = saturate(R_Cmd,6)
                
            if(L_Cmd>pk or R_Cmd>pk):
                if(R_Cmd>L_Cmd):
                    pk=R_Cmd
                if(L_Cmd>R_Cmd):
                    pk=L_Cmd
            
            SendCmdTorque(L_Cmd_sat, R_Cmd_sat)
            
            print(f" L_IMU: {L_IMU_angle:^8.3f} | R_IMU: {R_IMU_angle:^8.3f} | L_CMD: {L_Cmd_sat:^8.3f} | R_CMD: {R_Cmd_sat:^8.3f} | Peak: {pk:^8.3f} ")
            
            # Save to CSV 
            write_csv(now,
                      L_Cmd,R_Cmd,
                      L_tau,R_tau,
                      L_IMU_angle,R_IMU_angle,
                      L_IMU_vel,R_IMU_vel,
                      L_encoder, R_encoder,
                      L_encoder_vel, R_encoder_vel)
            
    if UpdateSec - StartSec > 5:
        Ant.Disconnect()
        print('Run Finish')
        break
    
    elif UpdateState == -1:
        print('Com Error')
        break
    else:
        ComErrorCnt += 1
        if GetSec() - UpdateSuccessSec > 0.3:
            print('Error: Com Lost')
            Ant.Cmd.CmdMode = kqio.CMD_OBS_ONLY
            Ant.Disconnect()
            break
    UpdateState = Ant.Update()  
    ComTotalCnt += 1
