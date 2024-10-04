import ReadIMU as ReadIMU   
from ReadIM948Serial import READIMU    
from DNN_torch import DNN    
import datetime    
import time   
import numpy as np     
import csv                          


# Initialize IMU and DNN
# ComPort = '/dev/ttyUSB0'  

ser_port_left = "/dev/ttyUSB2"       
ser_baudrate = 115200          
ser_timeout = 0.01                         

ser_port_right = "/dev/ttyUSB3"      
ser_baudrate = 115200          
ser_timeout = 0.01            

imu_read_left = READIMU(ser_port_left)      
imu_read_right = READIMU(ser_port_right)    

for i in range(200):  
    imu_read_left.read()  
    imu_read_right.read()   
    initial_left = imu_read_left.AngleX 
    initial_right = imu_read_right.AngleX   
    print("++++++++++++++++++++++++++++")  
        
# # connect to teensy 
# ComPort = '/dev/serial0'    
# imu = ReadIMU.READIMU(ComPort)     

start = time.time()   

dnn = DNN(18, 128, 64, 2)  # depends on training network 
now = 0  
t_pr1 = 0   
t_pr2 = 0   
t_pr3 = 0   
pk = 0   
counter = 0     
L_Cmd = 0   
R_Cmd = 0   

L_Ctl = -1  
R_Ctl = 1   
Cmd_scale = 20.0  
dt = 0.01 

L_IMU_angle     = 0.0 
R_IMU_angle     = 0.0 
L_IMU_angle_pre = 0.0 
R_IMU_angle_pre = 0.0 

# command: 1.5 for running, 2 for climbing  
kcontrol = 1        
output = np.array([])    

date = time.localtime(time.time())  
date_year = date.tm_year
date_month = date.tm_mon
date_day = date.tm_mday
date_hour = date.tm_hour
date_minute = date.tm_min
date_second = date.tm_sec  


# Create filename with format {Year}{Month}{Day}-{Hour}{Minute}{Second}.csv
csv_filename = f"{date_year:04}{date_month:02}{date_day:02}-{date_hour:02}{date_minute:02}{date_second:02}.csv"
with open(csv_filename, 'a', newline='') as csvfile:
    fieldnames = ['L_IMU_Ang', 'R_IMU_Ang', 'L_IMU_Vel', 'R_IMU_Vel', 'L_Cmd', 'R_Cmd', 'Peak', 'Time']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    # Write the header only if the file is empty
    csvfile.seek(0, 2)   
    if csvfile.tell() == 0:    
        writer.writeheader()   

    while True:
        now = (time.time() - start)  
        
        # imu.read()    
        # imu.decode()    
        # print("count :", counter)      

        # counter = counter + 1 
        
        # L_IMU_angle = imu.XIMUL 
        # R_IMU_angle = imu.XIMUR 
        # L_IMU_vel   = imu.XVIMUL 
        # R_IMU_vel   = imu.XVIMUR   
        
        imu_read_left.read()  
        imu_read_right.read()   
    
        L_IMU_angle = imu_read_left.AngleX - initial_left
        R_IMU_angle = imu_read_right.AngleX - initial_left
        
        L_IMU_vel   = (L_IMU_angle - L_IMU_angle_pre)/dt
        R_IMU_vel   = (R_IMU_angle - R_IMU_angle_pre)/dt  
        
        L_IMU_angle_pre = L_IMU_angle 
        R_IMU_angle_pre = R_IMU_angle  
        
        # print(f"Time after reading IMU = {now:^8.3f}")  
        
        # if (now - t_pr3 > 3):  # Time to reset peak torque printed to terminal
        #     t_pr3 = now
        #     pk = 0   

        # if (now - t_pr1 > 0.001):    
        
        t_pr1 = now
        kp = 10  
        kd = 400   
        
        # print(f"Time when running NN = {now:^8.3f}")  
        dnn.generate_assistance(L_IMU_angle, R_IMU_angle, L_IMU_vel, R_IMU_vel, kp, kd)  

        L_Cmd = L_Ctl * dnn.hip_torque_L * kcontrol
        R_Cmd = R_Ctl * dnn.hip_torque_R * kcontrol   

        if (L_Cmd > pk or R_Cmd > pk):
            if (R_Cmd > L_Cmd):
                pk = R_Cmd
            if (L_Cmd > R_Cmd):
                pk = L_Cmd   
        
        print(f"| now: {now:^8.3f} | L_IMU_Ang: {L_IMU_angle:^8.3f} | R_IMU_Ang: {R_IMU_angle:^8.3f} | L_IMU_Vel: {L_IMU_vel:^8.3f} | R_IMU_Vel: {R_IMU_vel:^8.3f} | L_Cmd: {L_Cmd:^8.3f} | R_Cmd: {R_Cmd:^8.3f} | Peak: {pk:^8.3f} |")

        # B1_int16 = int(imu.ToUint(L_Cmd/Cmd_scale, -20, 20, 16))    
        # B2_int16 = int(imu.ToUint(R_Cmd/Cmd_scale, -20, 20, 16))     

        # b1 = (B1_int16 >> 8 & 0x00ff)
        # b2 = (B1_int16 & 0x00FF)
        # b3 = (B2_int16 >> 8 & 0x00ff)
        # b4 = (B2_int16 & 0x00FF) 

        # imu.send(b1, b2, b3, b4)   

        data = {
            'L_IMU_Ang': L_IMU_angle,
            'R_IMU_Ang': R_IMU_angle,
            'L_IMU_Vel': L_IMU_vel,
            'R_IMU_Vel': R_IMU_vel,
            'L_Cmd': L_Cmd/Cmd_scale,  
            'R_Cmd': R_Cmd/Cmd_scale,  
            'Peak': pk,
            'Time': now
        }
        writer.writerow(data)
        csvfile.flush()  # Ensure data is written to file 