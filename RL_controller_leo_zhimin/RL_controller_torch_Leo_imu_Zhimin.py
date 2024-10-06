import time   
from math import *   
import kqExoskeletonIO as kqio   
import ReadIMU as ReadIMU   
from DNN_torch import DNN   
from utils import *   
import socket 
import zmq  

# # 定义服务器的IP地址和端口号
# server_ip = '10.154.28.205'     
# server_port = 45678    
# client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    

# 创建一个 ZeroMQ 上下文  
context = zmq.Context()   
# 创建 REQ（请求）套接字  
socket = context.socket(zmq.REQ)
# 连接到服务器的 IP 地址和端口，假设服务器的 IP 地址是 192.168.1.10
server_address = "tcp://10.154.28.205:7794"    
# server_address = "tcp://192.168.12.112:7794"  
socket.connect(server_address)    


ctl_mode     = 1           
nn_mode      = 1  
kcontrol     = 0.1          
max_cmd      = 5.0         
torque_scale = 40.0   

#########################################
# IMU settings  
######################################### 
# ComPort = '/dev/ttyUSB0'  
ComPort = '/dev/ttyS0' 
# ComPort = '/dev/serial0'
imu = ReadIMU.READIMU(ComPort)       

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

date  = time.localtime(time.time())
dateh = date.tm_hour
datem = date.tm_min  
dates = date.tm_sec    

# def write_csv(V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12,V13):
#     with open(str(dateh)+str(datem)+str(dates)+".csv", "a") as log:
#         log.write("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}\n".format(str(V1),str(V2),str(V3),str(V4),str(V5),str(V6),str(V7),str(V8),str(V9),str(V10),str(V11),str(V12),str(V13)))


# with open(str(dateh)+str(datem)+str(dates)+".csv", "a") as log:
#     log.write("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}\n".format(
#         "t",
#         "L_Cmd", "R_Cmd",
#         "L_tau", "R_tau",
#         "L_IMU_angle", "R_IMU_angle",
#         "L_IMU_vel", "R_IMU_vel",
#         "L_encoder", "R_encoder",
#         "L_encoder_vel", "R_encoder_vel" 
#     ))   

# print("Initializing the comunication with the Exoskeleton")
# GetSec           = kqio.GetSec  
# # Ant              = kqio.AntCH("/dev/ttyAMA0")                # This is the comport that connects the Raspberry Pi 4 to the LEO
# Ant              = kqio.AntCH("/dev/ttyS0")  
# Ant.Cmd.CmdMode  = kqio.CMD_SERVO_OVERRIDE  
# StartSec         = GetSec() 
# UpdateSec        = StartSec 
# UpdateState      = Ant.ComState 
# UpdateSuccessSec = StartSec  
# AntConnected     = (Ant.ComState == 1)   

ComTotalCnt = 1   
# ComErrorCnt = 0   
print("Succesful initialization")    

# input("Getting initial angular position values for encoders and press enter")  
# # The follwing sign definition follow the rigth hand side rule assuming that the rotation axis is pointing outside of the exoskeleton (for each motor)
# StartHipAngle_L = rad2deg(Ant.Data.HipAngle_L)      
# StartHipAngle_R = rad2deg(Ant.Data.HipAngle_R)     

start = time.time()    
while True:   
    UpdateSuccessSec = time.time()  
    CurrentSec = UpdateSuccessSec - start       

    imu.read()    
    
    L_IMU_angle = imu.XIMUL    
    R_IMU_angle = imu.XIMUR    
    L_IMU_vel   = imu.XVIMUL    
    R_IMU_vel   = imu.XVIMUR      
    
    # print(f" Time: {UpdateSuccessSec:^8.3f}")    
    
    # L_tau = Ant.Data.HipTor_L    
    # R_tau = Ant.Data.HipTor_R     
    
    # L_encoder     = rad2deg(Ant.Data.HipAngle_L) - StartHipAngle_L
    # R_encoder     = rad2deg(Ant.Data.HipAngle_R) - StartHipAngle_R
    # L_encoder_vel = rad2deg(Ant.Data.HipSpeed_L)    
    # R_encoder_vel = rad2deg(Ant.Data.HipSpeed_R)   

    # if ctl_mode == 0:   
    #     L_IMU_angle = imu.XIMUL  
    #     R_IMU_angle = imu.XIMUR  
    #     L_IMU_vel   = imu.XVIMUL
    #     R_IMU_vel   = imu.XVIMUR   
    # else: 
    #     L_IMU_angle = -1 * L_encoder 
    #     R_IMU_angle = -1 * R_encoder  
    #     L_IMU_vel   = -1 * L_encoder_vel  
    #     R_IMU_vel   = -1 * R_encoder_vel      
    
    kp = 10
    kd = 400   
    dnn.generate_assistance(L_IMU_angle, R_IMU_angle, L_IMU_vel, R_IMU_vel, kp, kd)    
    
    L_Cmd     = dnn.hip_torque_L/torque_scale * kcontrol  
    R_Cmd     = dnn.hip_torque_R/torque_scale * kcontrol  
    
    # L_Cmd = 1 
    # R_Cmd = 1   

    L_Cmd_sat = saturate(L_Cmd, max_cmd)     
    R_Cmd_sat = saturate(R_Cmd, max_cmd)     
        
    # if(L_Cmd>pk or R_Cmd>pk):   
    #     if(R_Cmd>L_Cmd):  
    #         pk=R_Cmd  
    #     if(L_Cmd>R_Cmd):  
    #         pk=L_Cmd   
    
    # send torque cmd  
    # SendCmdTorque(L_Cmd_sat, R_Cmd_sat)      
    # SendCmdTorque(0.0, 0.0)     
    
    print(f" Time: {CurrentSec:^8.3f}, L_IMU: {L_IMU_angle:^8.3f} | R_IMU: {R_IMU_angle:^8.3f} | L_CMD: {L_Cmd_sat:^8.3f} | R_CMD: {R_Cmd_sat:^8.3f}")
    
    # if ComTotalCnt%5 == 0:  
    #     print(f" Time: {CurrentSec:^8.3f}, L_IMU: {L_IMU_angle:^8.3f} | R_IMU: {R_IMU_angle:^8.3f} | L_CMD: {L_Cmd_sat:^8.3f} | R_CMD: {R_Cmd_sat:^8.3f}")
    #     data = f"{L_IMU_angle:.1f}" + "," + f"{R_IMU_angle:.1f}" + "," + f"{L_Cmd_sat:.1f}" + "," + f"{R_Cmd_sat:.1f}"  
    #     socket.send(data.encode())       
    #     response = socket.recv_string()        
        # print(f"收到服务器响应: {response}")        
    
    # # Save to CSV 
    # write_csv(
    #     now, 
    #     L_IMU_angle, R_IMU_angle,
    #     L_IMU_vel, R_IMU_vel,
    #     L_encoder, R_encoder, 
    #     L_encoder_vel, R_encoder_vel, 
    #     L_Cmd, R_Cmd,
    #     L_tau, R_tau 
    # ) 
            
    # if UpdateSec - StartSec > 5:
    #     Ant.Disconnect()  
    #     print('Run Finish')  
    #     break  
    # elif UpdateState == -1:  
    #     print('Com Error')
    #     break 
    # else: 
    #     ComErrorCnt += 1 
    #     if GetSec() - UpdateSuccessSec > 0.3: 
    #         print('Error: Com Lost') 
    #         Ant.Cmd.CmdMode = kqio.CMD_OBS_ONLY 
    #         Ant.Disconnect() 
    #         break 

    # UpdateState = Ant.Update()   
    ComTotalCnt += 1    