import serial
from ast import Try
from time import sleep
import numpy as np
from numpy import array  
import gatt  
import time   

from argparse import ArgumentParser  
from array import array   
import socket 
import sys   

import threading    
import csv    
import zmq


context = zmq.Context()   
socket = context.socket(zmq.REQ)  # REQ 表示请求模式
socket.connect("tcp://127.0.0.1:5555")    


L_IMU_angle = 0   
R_IMU_angle = 0   
L_IMU_vel   = 0    
R_IMU_vel   = 0     

L_encoder     = 0   
R_encoder     = 0   
L_encoder_vel = 0    
R_encoder_vel = 0    

index = 0 


class AnyDevice(gatt.Device):   
    def __init__(self, manager, mac_address): 
        super(AnyDevice, self).__init__(manager=manager, mac_address=mac_address)
        self.sock_pc = None
        self.parse_imu_flage = False 

    def connect_succeeded(self):
        super().connect_succeeded()
        print("[%s] Connected" % (self.mac_address))  

    def connect_failed(self, error): 
        super().connect_failed(error)  
        print("[%s] Connection failed: %s" % (self.mac_address, str(error)))

    def disconnect_succeeded(self):   
        super().disconnect_succeeded()  
        print("[%s] Disconnected" % (self.mac_address))  

    def services_resolved(self):  
        super().services_resolved()  

        print("[%s] Resolved services" % (self.mac_address))  
        for service in self.services:
            print("[%s]\tService [%s]" % (self.mac_address, service.uuid))
            for characteristic in service.characteristics:
                print("[%s]\t\tCharacteristic [%s]" % (self.mac_address, characteristic.uuid))
               
        # 保持连接
        lzchar1 = next(
            c for c in service.characteristics
            if c.uuid == '0000ae01-0000-1000-8000-00805f9b34fb'.lower())
        lzchar1.write_value(')'.encode()) # 发送十六进制的0x29，让设备保持连接
        
        # 尝试采用蓝牙高速通信特性 0x46  
        lzchar1.write_value(bytes([0x46]))   

        # GPIO 上拉
        #lzchar1.write_value(bytes([0x27,0x10]))

        # 参数设置
        isCompassOn = 0        #1=使用磁场融合姿态，0=不使用
        barometerFilter = 2
        Cmd_ReportTag = 0x044 # 功能订阅标识
        params = bytearray([0x00 for i in range(0,11)])
        params[0] = 0x12
        params[1] = 5       #静止状态加速度阀值
        params[2] = 255     #静止归零速度(单位cm/s) 0:不归零 255:立即归零
        params[3] = 0       #动态归零速度(单位cm/s) 0:不归零
        params[4] = ((barometerFilter&3)<<1) | (isCompassOn&1);   
        params[5] = 200      #数据主动上报的传输帧率[取值0-250HZ], 0表示0.5HZ
        params[6] = 1       #陀螺仪滤波系数[取值0-2],数值越大越平稳但实时性越差
        params[7] = 3       #加速计滤波系数[取值0-4],数值越大越平稳但实时性越差
        params[8] = 5       #磁力计滤波系数[取值0-9],数值越大越平稳但实时性越差
        params[9] = Cmd_ReportTag&0xff
        params[10] = (Cmd_ReportTag>>8)&0xff
        lzchar1.write_value(params)

        # 主动上报 0x19 
        lzchar1.write_value(bytes([0x19]))  
        
        #lzchar1.write_value(bytes([0x51,0xAA,0xBB])) # 用总圈数代替欧拉角传输 并清零圈数 0x51
        #lzchar1.write_value(bytes([0x51,0x00,0x00])) # 输出欧拉角 0x51

        lzchar2 = next(
            c for c in service.characteristics
            if c.uuid == '0000ae02-0000-1000-8000-00805f9b34fb'.lower())  
        
        lzchar2.enable_notifications()

    def descriptor_read_value_failed(self, descriptor, error):
        print('descriptor_value_failed')

    def characteristic_write_value_succeeded(self, characteristic):
        super().characteristic_write_value_succeeded(characteristic)
        print("[%s] wr ok" % (self.mac_address))

    def characteristic_write_value_failed(self, characteristic, error):
        super().characteristic_write_value_failed(characteristic, error)
        print("[%s] wr err %s" % (self.mac_address, error))
    
    def characteristic_enable_notifications_succeeded(self, characteristic):
        super().characteristic_enable_notifications_succeeded(characteristic)
        print("[%s] notify ok" % (self.mac_address))

    def characteristic_enable_notifications_failed(self, characteristic, error):
        super().characteristic_enable_notifications_failed(characteristic, error)
        print("[%s] notify err. %s" % (self.mac_address, error))

    def characteristic_value_updated(self, characteristic, value):
        print("Lzchar:", value.hex()) 
        print("value.size",len(value))  
        
        if characteristic.uuid == '0000ae02-0000-1000-8000-00805f9b34fb'.lower():
            if self.parse_imu_flage:
                self.parse_imu(value)

            if self.sock_pc is not None:
                print("send blue source data") 
                self.sock_pc.sendall(value) 
                
    def parse_imu(self, buf):  
        global L_IMU_angle, R_IMU_angle, L_IMU_vel, R_IMU_vel, index  
        
        scaleAccel       = 0.00478515625      # 加速度 [-16g~+16g]    9.8*16/32768
        scaleQuat        = 0.000030517578125  # 四元数 [-1~+1]         1/32768
        scaleAngle       = 0.0054931640625    # 角度   [-180~+180]     180/32768
        scaleAngleSpeed  = 0.06103515625      # 角速度 [-2000~+2000]    2000/32768
        scaleMag         = 0.15106201171875   # 磁场 [-4950~+4950]   4950/32768
        scaleTemperature = 0.01               # 温度
        scaleAirPressure = 0.0002384185791    # 气压 [-2000~+2000]    2000/8388608
        scaleHeight      = 0.0010728836       # 高度 [-9000~+9000]    9000/8388608

        imu_dat = array('f',[0.0 for i in range(0, 34)])    

        if buf[0] == 0x11:  
            ctl = (buf[2] << 8) | buf[1]   
            print(" subscribe tag: 0x%04x"%ctl)   
            print(" ms: ", ((buf[6]<<24) | (buf[5]<<16) | (buf[4]<<8) | (buf[3]<<0)))

            L =7 # 从第7字节开始根据 订阅标识tag来解析剩下的数据
            if ((ctl & 0x0001) != 0):
                tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2 
                # print("\taX: %.3f"%tmpX); # x加速度aX
                tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2 
                # print("\taY: %.3f"%tmpY); # y加速度aY
                tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
                # print("\taZ: %.3f"%tmpZ); # z加速度aZ

                imu_dat[0] = float(tmpX)
                imu_dat[1] = float(tmpY)
                imu_dat[2] = float(tmpZ)
            
            print(" ")
            if ((ctl & 0x0002) != 0):
                tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
                # print("\tAX: %.3f"%tmpX) # x加速度AX
                tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
                # print("\tAY: %.3f"%tmpY) # y加速度AY
                tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
                # print("\tAZ: %.3f"%tmpZ) # z加速度AZ

                imu_dat[3] = float(tmpX)
                imu_dat[4] = float(tmpY)
                imu_dat[5] = float(tmpZ)

            print(" ")
            if ((ctl & 0x0004) != 0):
                tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2 
                # print("\tGX: %.3f"%tmpX) # x角速度GX
                tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2 
                # print("\tGY: %.3f"%tmpY) # y角速度GY
                tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2
                # print("\tGZ: %.3f"%tmpZ) # z角速度GZ

                imu_dat[6] = float(tmpX)
                imu_dat[7] = float(tmpY)
                imu_dat[8] = float(tmpZ)    
                
                L_IMU_vel = tmpX   
                R_IMU_vel = tmpX    
                print("L_IMU_vel, R_IMU_vel :", L_IMU_vel, R_IMU_vel)   
                
            print(" ")  
            if ((ctl & 0x0040) != 0):  
                tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
                # print("\tangleX: %.3f"%tmpX); # x角度
                tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
                # print("\tangleY: %.3f"%tmpY); # y角度
                tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
                # print("\tangleZ: %.3f"%tmpZ); # z角度  

                imu_dat[19] = float(tmpX)    
                imu_dat[20] = float(tmpY)    
                imu_dat[21] = float(tmpZ)    
                
                L_IMU_angle = tmpX    
                R_IMU_angle = tmpX    
                print("L_IMU_angle, R_IMU_angle :", L_IMU_angle, R_IMU_angle)   
                
                # 发送消息并等待响应  
                socket.send_string("Hello, Server!")  
                message = socket.recv_string()  
                print(f"收到响应: {message}")     
                print("index :", index)  
                index += 1     
            
            # print(" ")
            # if ((ctl & 0x0008) != 0):
            #     tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
            #     print("\tCX: %.3f"%tmpX); # x磁场CX
            #     tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
            #     print("\tCY: %.3f"%tmpY); # y磁场CY
            #     tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
            #     print("\tCZ: %.3f"%tmpZ); # z磁场CZ

            #     imu_dat[9] = float(tmpX)  
            #     imu_dat[10] = float(tmpY)  
            #     imu_dat[11] = float(tmpZ)     
            
            # print(" ")
            # if ((ctl & 0x0010) != 0):
            #     tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleTemperature; L += 2
            #     print("\ttemperature: %.2f"%tmpX) # 温度

            #     tmpU32 = np.uint32(((np.uint32(buf[L+2]) << 16) | (np.uint32(buf[L+1]) << 8) | np.uint32(buf[L])))
            #     if ((tmpU32 & 0x800000) == 0x800000): # 若24位数的最高位为1则该数值为负数，需转为32位负数，直接补上ff即可
            #         tmpU32 = (tmpU32 | 0xff000000)      
            #     tmpY = np.int32(tmpU32) * scaleAirPressure; L += 3
            #     print("\tairPressure: %.3f"%tmpY); # 气压

            #     tmpU32 = np.uint32((np.uint32(buf[L+2]) << 16) | (np.uint32(buf[L+1]) << 8) | np.uint32(buf[L]))
            #     if ((tmpU32 & 0x800000) == 0x800000): # 若24位数的最高位为1则该数值为负数，需转为32位负数，直接补上ff即可
            #         tmpU32 = (tmpU32 | 0xff000000)
            #     tmpZ = np.int32(tmpU32) * scaleHeight; L += 3 
            #     print("\theight: %.3f"%tmpZ); # 高度

            #     imu_dat[12] = float(tmpX)
            #     imu_dat[13] = float(tmpY)
            #     imu_dat[14] = float(tmpZ)

            # print(" ")
            # if ((ctl & 0x0020) != 0):
            #     tmpAbs = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            #     print("\tw: %.3f"%tmpAbs); # w
            #     tmpX =   np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            #     print("\tx: %.3f"%tmpX); # x
            #     tmpY =   np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            #     print("\ty: %.3f"%tmpY); # y
            #     tmpZ =   np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
            #     print("\tz: %.3f"%tmpZ); # z

            #     imu_dat[15] = float(tmpAbs)
            #     imu_dat[16] = float(tmpX)
            #     imu_dat[17] = float(tmpY)
            #     imu_dat[18] = float(tmpZ)  

            # print(" ")
            # if ((ctl & 0x0080) != 0):
            #     tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) / 1000.0; L += 2
            #     print("\toffsetX: %.3f"%tmpX); # x坐标
            #     tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) / 1000.0; L += 2
            #     print("\toffsetY: %.3f"%tmpY); # y坐标
            #     tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) / 1000.0; L += 2
            #     print("\toffsetZ: %.3f"%tmpZ); # z坐标

            #     imu_dat[22] = float(tmpX)
            #     imu_dat[23] = float(tmpY)   
            #     imu_dat[24] = float(tmpZ)   

            # print(" ")
            # if ((ctl & 0x0100) != 0):
            #     tmpU32 = ((buf[L+3]<<24) | (buf[L+2]<<16) | (buf[L+1]<<8) | (buf[L]<<0)); L += 4
            #     print("\tsteps: %u"%tmpU32); # 计步数
            #     tmpU8 = buf[L]; L += 1
            #     if (tmpU8 & 0x01):# 是否在走路
            #         print("\t walking yes")
            #         imu_dat[25] = 100
            #     else:
            #         print("\t walking no")
            #         imu_dat[25] = 0
            #     if (tmpU8 & 0x02):# 是否在跑步
            #         print("\t running yes")
            #         imu_dat[26] = 100
            #     else:
            #         print("\t running no")
            #         imu_dat[26] = 0
            #     if (tmpU8 & 0x04):# 是否在骑车
            #         print("\t biking yes")
            #         imu_dat[27] = 100
            #     else:
            #         print("\t biking no")
            #         imu_dat[27] = 0
            #     if (tmpU8 & 0x08):# 是否在开车
            #         print("\t driving yes")
            #         imu_dat[28] = 100
            #     else:
            #         print("\t driving no")
            #         imu_dat[28] = 0

            # print(" ")  
            # if ((ctl & 0x0200) != 0):  
            #     tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            #     print("\tasX: %.3f"%tmpX); # x加速度asX
            #     tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            #     print("\tasY: %.3f"%tmpY); # y加速度asY
            #     tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
            #     print("\tasZ: %.3f"%tmpZ); # z加速度asZ
            
            #     imu_dat[29] = float(tmpX) 
            #     imu_dat[30] = float(tmpY)  
            #     imu_dat[31] = float(tmpZ)   
                
            # print(" ")   
            # if ((ctl & 0x0400) != 0):  
            #     tmpU16 = ((buf[L+1]<<8) | (buf[L]<<0)); L += 2
            #     print("\tadc: %u"%tmpU16); # adc测量到的电压值，单位为mv
            #     imu_dat[32] = float(tmpU16)  

            # print(" ")
            # if ((ctl & 0x0800) != 0):
            #     tmpU8 = buf[L]; L += 1
            #     print("\t GPIO1  M:%X, N:%X"%((tmpU8>>4)&0x0f, (tmpU8)&0x0f))
            #     imu_dat[33] = float(tmpU8)  

        else:
            print("[error] data head not define")


class READIMU(object):
    def __init__(self, ComPort) -> None:
        self.ComPort = ComPort
        
        self.AngleX_Left = 0  
        self.AngleX_Right = 0  
        self.AngleVelX_Left = 0  
        self.AngleVelX_Right = 0     
        
        #Serial Variables --------
        self.buffer = 0x00    
        self.buffer_len = 0x00    
        #Serial Begin -------------
        
        # self.Serial_IMU = serial.Serial(ComPort, 230400, timeout=0.02, parity=serial.PARITY_NONE)
        #self.Serial_IMU = serial.Serial(ComPort, 115200, timeout=0.007, parity=serial.PARITY_NONE)
        self.Serial_IMU = serial.Serial(ComPort, 115200, timeout=0.001, parity=serial.PARITY_NONE)  

        print('Serial Open Success')
        #Serial END---------------

    def read(self):  
        self.buffer = self.Serial_IMU.read(11)    
        self.buffer_len = len(self.buffer)   
        print("length: ", self.buffer_len, self.buffer[0], self.buffer[1], self.buffer[10])  

    def send(self,b1,b2,b3,b4):  
        self.Serial_IMU.write(bytearray([0x40, 0x41, 0x42, 
                               b1, b2, b3, b4, 0x43]))
    
    def printHEX(self, Vprint):
        print([hex(x) for x in Vprint])

    def ToUint(self, x, x_min, x_max, nbits):
        span = x_max - x_min

        if (x < x_min):
            x = x_min

        if (x > x_max):
            x = x_max
        toUint=((x - x_min) * ((float)((1 << nbits) - 1) / span))
        return toUint  

    def ToFloat(self,x_int, x_min, x_max, nbits):
        span = x_max - x_min
        offset_value = x_min
        toFloat= x_int * span / float((((1 << nbits) - 1))) + offset_value
        return toFloat  

    def decode(self):  
        #if len(self.buffer)==7 and self.buffer[0]==0x3a and self.buffer[1]==0xc4 :
        if len(self.buffer)==11 and self.buffer[0]==0x31 and self.buffer[1]==0x32  and self.buffer[10]==0x33:
            self.L_XIMU_int16=(self.buffer[2] << 8) | (self.buffer[3])
            self.R_XIMU_int16=(self.buffer[4] << 8) | (self.buffer[5])
            self.L_XVIMU_int16=(self.buffer[6] << 8) | (self.buffer[7])
            self.R_XVIMU_int16=(self.buffer[8] << 8) | (self.buffer[9])


            self.XIMUL=self.ToFloat(self.L_XIMU_int16,-180,180,16)
            self.XIMUR=self.ToFloat(self.R_XIMU_int16,-180,180,16)
            self.XVIMUL=self.ToFloat(self.L_XVIMU_int16,-800,800,16)
            self.XVIMUR=self.ToFloat(self.R_XVIMU_int16,-800,800,16)

            '''    self.XIMU=((self.buffer[6] << 24) | (self.buffer[5]) << 16| 
            (self.buffer[4] << 8) | (self.buffer[3]))'''
            
            #print(hex(self.buffer))
        else:
            # input()  
            print("----------------------------------------")
            print("----------------------------------------")
            print("----------------------------------------")
            print("----------------------------------------") 
            print("----------------------------------------")  
            print("----------------------------------------")
            print("----------------------------------------")
            print("----------------------------------------")
            print("----------------------------------------") 
            print("----------------------------------------")
            self.Serial_IMU.reset_input_buffer()   
            self.Serial_IMU.reset_output_buffer()   
            

def read_data():     
    global L_IMU_angle, R_IMU_angle, L_IMU_vel, R_IMU_vel, L_encoder, R_encoder, L_encoder_vel, R_encoder_vel 
    
    host = None    
    port = 6666  
    sock = None       
    if host is not None:    
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((host, port))
        except:
            print("Could not make a connection to the server")
            input("Press enter to quit")
            sys.exit(0)  

    print("Connecting bluetooth ...")   
    manager = gatt.DeviceManager(adapter_name='hci0')    
    device = AnyDevice(manager=manager, mac_address="E9:65:F2:E3:6E:58")      
    device.sock_pc = sock   
    if host is None:  
        device.parse_imu_flage = True  

    device.connect()     
    manager.run()    
    

def render_data():   
    global L_IMU_angle, R_IMU_angle, L_IMU_vel, R_IMU_vel  
    start = time.time()    
    csv_filename = "../data/bluetooth_data_comparison.csv"      
    
    server_ip = '10.154.28.205'   
    server_port = 34567  
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)   
    
    with open(csv_filename, 'a', newline='') as csvfile:
        fieldnames = ['L_IMU_Ang', 'R_IMU_Ang', 'L_IMU_Vel', 'R_IMU_Vel', 'L_Cmd', 'R_Cmd', 'Peak', 'Time']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)   
        
        csvfile.seek(0, 2)    
        if csvfile.tell() == 0:    
            writer.writeheader()     
            
        while True:  
            now = (time.time() - start)   

            L_Cmd = 0 
            R_Cmd = 0  
            
            data = {
                'L_IMU_Ang': L_IMU_angle,
                'R_IMU_Ang': R_IMU_angle,
                'L_IMU_Vel': L_IMU_vel,
                'R_IMU_Vel': R_IMU_vel,
                'L_Cmd': L_Cmd,  
                'R_Cmd': R_Cmd,    
                'Peak': 0.0,
                'Time': now
            }   
            
            writer.writerow(data)     
            csvfile.flush()         
            

def save_data():    
    global L_IMU_angle, R_IMU_angle, L_IMU_vel, R_IMU_vel, L_encoder, R_encoder, L_encoder_vel, R_encoder_vel 
    
    def write_csv(V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12,V13):
        with open("../data/bluetooth_data_comparison.csv", "a") as log:
            log.write("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}\n".format(str(V1),str(V2),str(V3),str(V4),str(V5),str(V6),str(V7),str(V8),str(V9),str(V10),str(V11),str(V12),str(V13)))

    # ComPort = '/dev/serial0'  
    # imu = READIMU(ComPort)        
    
    # 定义服务器的IP地址和端口号
    server_ip = '10.154.28.205'    # 服务器的IP地址
    server_port = 45678  

    # 创建UDP客户端Socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  
    
    start = time.time()  
    # csv_filename = "../data/bluetooth_data_comparison_new.csv"    
    
    with open("../data/bluetooth_data_comparison.csv", "w") as log:
        log.write("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12}\n".format(
            "L_IMU_angle", 
            "R_IMU_angle",
            "L_IMU_vel", 
            "R_IMU_vel",
            "L_encoder", 
            "R_encoder",
            "L_encoder_vel", 
            "R_encoder_vel", 
            "L_Cmd", 
            "R_Cmd", 
            "L_torque", 
            "R_torque", 
            "Time"     
        ))    
            
    while True:  
        now = (time.time() - start)    
        
        # imu.read()  
        # imu.decode()     
        
        # L_encoder       = imu.XIMUL
        # R_encoder       = imu.XIMUR
        # L_encoder_vel   = imu.XVIMUL
        # R_encoder_vel   = imu.XVIMUR    
        
        print("L_IMU_angle :", L_IMU_angle, "R_IMU_angle :", R_IMU_angle)    
        
        data = {
            'L_IMU_angle': L_IMU_angle,   
            'R_IMU_angle': R_IMU_angle,   
            'L_IMU_vel': L_IMU_vel,   
            'R_IMU_vel': R_IMU_vel,   
            'L_encoder': L_encoder,   
            'R_encoder': R_encoder,  
            'L_encoder_vel': L_encoder_vel,      
            'R_encoder_vel': R_encoder_vel,    
            "L_Cmd": 0.0,   
            "R_Cmd": 0.0,   
            "L_torque": 0.0,     
            "R_torque": 0.0,  
            'Time': now  
        }   
        
        data = f"{L_IMU_angle:.1f}" + "," + f"{R_IMU_angle:.1f}" + "," + f"{L_IMU_vel:.1f}" + "," + f"{R_IMU_vel:.1f}"  
        client_socket.sendto(data.encode(), (server_ip, server_port))   

        # # Save to CSV 
        # write_csv(
        #     L_IMU_angle, 
        #     R_IMU_angle,  
        #     L_IMU_vel, 
        #     R_IMU_vel, 
        #     L_encoder, 
        #     R_encoder, 
        #     L_encoder_vel, 
        #     R_encoder_vel, 
        #     0.0, 0.0, 
        #     0.0, 0.0, 
        #     now  
        # )   


if __name__ == "__main__":  
    arg_parser = ArgumentParser(description="GATT Connect Demo")
    args = arg_parser.parse_args()

    thread_reading = threading.Thread(target=read_data)    
    # thread_save    = threading.Thread(target=save_data)     
     
    thread_reading.start()     
    # thread_save.start()    
    
    thread_reading.join()     
    # thread_save.join()   