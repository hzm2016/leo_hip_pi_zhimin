import ReadIMU as ReadIMU
import time
from DNN import DNN
import datetime
import numpy as np

#ComPort='/dev/ttyUSB0'
ComPort='com8' # Look at the Device Manager
imu = ReadIMU.READIMU(ComPort)
start = time.time()
dnn = DNN(18,128,64,2)
now = 0
t_pr1 = 0
t_pr2 = 0
t_pr3 = 0
pk=0

B1_int16=0XFFFFFFFF
B2_int16=0XFFFFFFFF
b1=0XFFFFFFFF
b2=0XFFFFFFFF
b3=0XFFFFFFFF
b4=0XFFFFFFFF

date = time.localtime(time.time())

dateh=date.tm_hour
datem=date.tm_min
dates=date.tm_sec
'''
def fake_torque(t):
    L=9.0*np.cos(t)
    R=9.0*np.sin(t)
    return L, R

def sat_fc(Cmd):
    if Cmd>15: 
        Cmd=15
    if Cmd<-15:
        Cmd=-15
    return Cmd


def write_csv(V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12,V13):
    with open(str(dateh)+str(datem)+str(dates)+".csv", "a") as log:
        log.write("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}\n".format(str(V1),str(V2),str(V3),str(V4),str(V5),str(V6),str(V7),str(V8),str(V9),str(V10),str(V11),str(V12),str(V13)))

with open(str(dateh)+str(datem)+str(dates)+".csv", "a") as log:
    log.write("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}\n".format("t","L_tor","R_tor",
                                                                                "L_Cmd","R_Cmd","0", 
                                                                                "0","L_IMU_angle",
                                                                                "R_IMU_angle",
                                                                                "L_IMU_vel",
                                                                                "R_IMU_vel",
                                                                                "Heart_Rate", 
                                                                                "ACC_X"))

'''
while(True):

    now=(time.time()-start)
    imu.read()
    imu.decode()

    L_IMU_angle=imu.XIMUL
    R_IMU_angle=imu.XIMUR
    L_IMU_vel=imu.XVIMUL
    R_IMU_vel=imu.XVIMUR

    if (now - t_pr2 > 0.1):
        t_pr2 = now
        print(" R_IMU: " + str(R_IMU_angle)+" pk: " + str(pk)+" L_Cmd: " + str(L_Cmd)+"  R_Cmd: " + str(R_Cmd))

    if (now - t_pr3 > 3): # Time to reset peak torque printed to terminal
        t_pr3 = now
        pk=0

    if (now - t_pr1 > 0.025):
        t_pr1 = now
        kp=10
        kd=400

        #kp=2.0
        #skd=0.6


        dnn.generate_assistance(L_IMU_angle, R_IMU_angle, L_IMU_vel, R_IMU_vel,kp,kd)

        kcontrol=1.5 # 1.5 para running. 2 para climbing.
        
        L_Cmd = -dnn.hip_torque_L*1.0*kcontrol
        R_Cmd = dnn.hip_torque_R*1.0*kcontrol

        '''L_Cmd= sat_fc(L_Cmd)
        R_Cmd= sat_fc(R_Cmd)'''
            
        if(L_Cmd>pk or R_Cmd>pk):
            if(R_Cmd>L_Cmd):
                pk=R_Cmd
            if(L_Cmd>R_Cmd):
                pk=L_Cmd
        
      
        '''   
        if(L_Cmd>13 or R_Cmd>13 or L_Cmd<-13 or R_Cmd<-13):
            L_Cmd = dnn.hip_torque_L*10.0*(0.0)
            R_Cmd = dnn.hip_torque_R*10.0*(0.0)
            '''
                
            

        # Save to CSV 
        '''
        write_csv(now,0,0,
                  L_Cmd,R_Cmd,0,0,
                  L_IMU_angle,R_IMU_angle,L_IMU_vel,R_IMU_vel,0,0)
        
        #L_Cmd, R_Cmd = fake_torque(now)'''


        
        B1_int16=int(imu.ToUint(L_Cmd, -20, 20, 16))
        B2_int16=int(imu.ToUint(R_Cmd, -20, 20, 16))

        b1=(B1_int16 >> 8 & 0x00ff)
        b2=(B1_int16 & 0x00FF)
        b3=(B2_int16 >> 8 & 0x00ff)
        b4=(B2_int16 & 0x00FF)

        imu.send(b1,b2,b3,b4)




        #print(" t: " + str(now))


    

    '''
    self.XIMUL=self.ToFloat(self.L_XIMU_int16,-180,180,16)
    self.XIMUR=self.ToFloat(self.R_XIMU_int16,-180,180,16)
    self.XVIMUL=self.ToFloat(self.L_XVIMU_int16,-800,800,16)
    self.XVIMUR=self.ToFloat(self.R_XVIMU_int16,-800,800,16)
    '''