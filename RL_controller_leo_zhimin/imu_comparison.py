import time   
from math import *   
import kqExoskeletonIO as kqio   
import ReadIMU as ReadIMU   
from DNN_torch import DNN   
from utils import *   
from ReadIMUserial import READIMU     
import socket   


#########################################
# IMU settings  
######################################### 
ser_port_left = "/dev/ttyUSB0"       
ser_baudrate = 115200          
ser_timeout = 0.01                         

ser_port_right = "/dev/ttyUSB1"        
ser_baudrate = 115200                  
ser_timeout = 0.01              

imu_read_left = READIMU(ser_port_left)       
imu_read_right = READIMU(ser_port_right)       
#########################################
# IMU settings  
#########################################   

#########################################
# GUI socket settings 
#########################################  
server_ip = '10.154.28.205'     
server_port = 23456    
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)   
##########################################
##########################################   

ComPort = '/dev/serial0'  
imu = ReadIMU.READIMU(ComPort)      

ctl_mode     = 1           # 1 for with IMU 0 for without IMU   
nn_mode      = 1  
kcontrol     = 0.3         # 1.5 para running. 2 para climbing.  
max_cmd      = 5.0         
torque_scale = 40.0        

# reference position 
pos_ampl     = 30 
pos_fre      = 0.5   
# reference position      

# network setup  
dnn = DNN(18, 128, 64, 2)        
# network setup   

now   = 0
t_pr1 = 0
t_pr2 = 0
t_pr3 = 0

L_Cmd = 0
R_Cmd = 0
pk    = 0
kp    = 10   
kd    = 400    

root_path = '../data/'  
date      = time.localtime(time.time()) 
dateh     = date.tm_hour  
datem     = date.tm_min    
dates     = date.tm_sec    

# def write_csv(V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12,V13): 
#     with open(root_path + str(dateh)+str(datem)+str(dates)+".csv", "a") as log:
#         log.write("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}\n".format(str(V1),str(V2),str(V3),str(V4),str(V5),str(V6),str(V7),str(V8),str(V9),str(V10),str(V11),str(V12),str(V13)))

# with open(root_path + str(dateh)+str(datem)+str(dates)+".csv", "a") as log:
#     log.write("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}\n".format(
#         "t",
#         "L_Cmd", "R_Cmd", 
#         "L_tau", "R_tau",   
#         "L_IMU_angle", "R_IMU_angle",
#         "L_IMU_vel", "R_IMU_vel",
#         "L_encoder", "R_encoder",
#         "L_encoder_vel", "R_encoder_vel" 
#     ))  

def write_csv(V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12,V13): 
    with open(root_path + "imu_comparison.csv", "a") as log:
        log.write("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}\n".format(str(V1),str(V2),str(V3),str(V4),str(V5),str(V6),str(V7),str(V8),str(V9),str(V10),str(V11),str(V12),str(V13)))

with open(root_path + "imu_comparison.csv", "a") as log:
    log.write("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}\n".format(
        "t",
        "L_IMU_angle", "R_IMU_angle",
        "L_IMU_vel", "R_IMU_vel",
        "L_encoder", "R_encoder",
        "L_encoder_vel", "R_encoder_vel", 
        "L_Cmd", "R_Cmd",   
        "L_tau", "R_tau"     
    ))  
    
for i in range(200):    
    imu_read_left.read()      
    imu_read_right.read()     
    initial_left = imu_read_left.AngleX 
    initial_right = imu_read_right.AngleX   
input("Finish initial setup for encoders and press enter:")    
    
start = time.time()    

while True:     
    now = (time.time() - start)    
    
    imu_read_left.read()      
    imu_read_right.read()      
    
    L_IMU_angle   = imu_read_left.AngleX - initial_left      
    R_IMU_angle   = imu_read_right.AngleX - initial_left      
    L_IMU_vel     = imu_read_left.AngleVelX                   
    R_IMU_vel     = imu_read_right.AngleVelX     
    
    # # ---------------------------------------------------
    imu.read()        
    imu.decode()        
    L_encoder       = imu.XIMUL
    R_encoder       = imu.XIMUR
    L_encoder_vel   = imu.XVIMUL
    R_encoder_vel   = imu.XVIMUR   
    
    # L_encoder       = 0.0 
    # R_encoder       = 0.0
    # L_encoder_vel   = 0.0 
    # R_encoder_vel   = 0.0    
    
    # data = f"{L_IMU_angle:.1f}" + "," + f"{R_IMU_angle:.1f}" + "," + f"{L_Cmd_sat:.1f}" + "," + f"{R_Cmd_sat:.1f}"  
    # client_socket.sendto(data.encode(), (server_ip, server_port))  
    
    L_tau       = 0.0       
    R_tau       = 0.0      
    
    
    print(f" Time: {now:^8.3f}, L_IMU: {L_IMU_angle:^8.3f} | R_IMU: {R_IMU_angle:^8.3f} | L_IMU_vel: {L_IMU_vel:^8.3f} | R_IMU_vel: {R_IMU_vel:^8.3f} | L_CMD: {L_Cmd:^8.3f} | R_CMD: {R_Cmd:^8.3f} | Peak: {pk:^8.3f} ")
    
    write_csv(
        now, 
        L_IMU_angle, R_IMU_angle,
        L_IMU_vel, R_IMU_vel,
        L_encoder, R_encoder, 
        L_encoder_vel, R_encoder_vel, 
        L_Cmd, R_Cmd,  
        L_tau, R_tau  
    )   