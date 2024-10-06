import numpy as np     
import serial    
import time    
import csv    
from utils import *     
import socket   
import zmq    



class READIMU(object):  
    def __init__(self, ComPort) -> None:  
        self.ComPort = ComPort   
        
        self.AngleX    = 0
        self.AngleVelX = 0    
        self.AngleY    = 0
        self.AngleVelY = 0  
        
        self.CmdPacket_Begin       = 0x49   # 起始码
        self.CmdPacket_End         = 0x4D     # 结束码
        self.CmdPacketMaxDatSizeRx = 73  # 模块发来的数据包的数据体最大长度
        
        self.CS = 0    
        self.i = 0           
        self.RxIndex = 0    
        self.buf = bytearray(5 + self.CmdPacketMaxDatSizeRx) # 接收包缓存
        self.cmdLen = 0                                      # 长度
        
        #------------ Serial Begin -------------
        self.Serial_IMU = serial.Serial(ComPort, 460800, timeout=0.001, parity=serial.PARITY_NONE)    
        # self.Serial_IMU = serial.Serial(ComPort, 230400, timeout=0.02, parity=serial.PARITY_NONE)      
        # self.Serial_IMU = serial.Serial(ComPort, 115200, timeout=0.007, parity=serial.PARITY_NONE)    
        # self.Serial_IMU = serial.Serial(ComPort, 115200, timeout=0.001, parity=serial.PARITY_NONE)   
        
        self.info_set()  
        print('Serial Open Success')   
        
        self.imu_buffer = np.memmap("imu_data.dat", dtype='float32', mode='r+', shape=(4,))    
        
    def info_set(self):    
        params = [0] * 11        # 数组
        
        isCompassOn = 0          # 1=开启磁场融合姿态 0=关闭磁场融合姿态
        barometerFilter = 2      # 气压计的滤波等级[取值0-3]
        Cmd_ReportTag = 0x044    # 功能订阅标识   
        
        params[0] = 0x12
        params[1] = 5            # 静止状态加速度阀值
        params[2] = 255          # 静止归零速度(单位cm/s) 0:不归零 255:立即归零
        params[3] = 0            # 动态归零速度(单位cm/s) 0:不归零
        params[4] = ((barometerFilter & 3) << 1) | (isCompassOn & 1)
        params[5] = 100          # 数据主动上报的传输帧率[取值0-250HZ], 0表示0.5HZ
        params[6] = 1            # 陀螺仪滤波系数[取值0-2],数值越大越平稳但实时性越差
        params[7] = 3            # 加速计滤波系数[取值0-4],数值越大越平稳但实时性越差
        params[8] = 5            # 磁力计滤波系数[取值0-9],数值越大越平稳但实时性越差
        params[9] = Cmd_ReportTag & 0xff
        params[10] = (Cmd_ReportTag >> 8) & 0xff
        
        self.Cmd_PackAndTx(params, len(params)) # 发送指令给传感器
        time.sleep(0.5)

        # 2.唤醒传感器
        for index in range(5):  
            self.Cmd_PackAndTx([0x03], 1)     
            time.sleep(0.1)     

            # 3.开启主动上报
            self.Cmd_PackAndTx([0x19], 1)    
            time.sleep(0.1)      
    
    def Cmd_PackAndTx(self, pDat, DLen):    
        if DLen == 0 or DLen > 19:
            return -1  # 非法参数

        # 构建发送包缓存，包括50字节的前导码
        self.buf = bytearray([0x00]*46) + bytearray([0x00, 0xff, 0x00, 0xff,  0x49, 0xFF, DLen]) + bytearray(pDat[:DLen])

        # 计算校验和，从地址码开始到数据体结束
        self.CS = sum(self.buf[51:51+DLen+2]) & 0xFF  # 取低8位
        self.buf.append(self.CS)    
        self.buf.append(0x4D)                         # 添加结束码  

        # 发送数据
        self.Serial_IMU.write(self.buf)     
        return 0
    
    def Cmd_GetPkt(self, byte):   
        self.CS += byte # 边收数据边计算校验码，校验码为地址码开始(包含地址码)到校验码之前的数据的和

        if self.RxIndex == 0: # 起始码  
            if byte == self.CmdPacket_Begin:     
                self.i = 0
                self.buf[self.i] = self.CmdPacket_Begin
                self.i += 1
                self.CS = 0 # 下个字节开始计算校验码
                self.RxIndex = 1   
        elif self.RxIndex == 1: # 数据体的地址码
            self.buf[self.i] = byte
            self.i += 1
            if byte == 255: # 255是广播地址，模块作为从机，它的地址不可会出现255
                self.RxIndex = 0
            else:
                self.RxIndex += 1  
        elif self.RxIndex == 2: # 数据体的长度
            self.buf[self.i] = byte   
            self.i += 1
            if byte > self.CmdPacketMaxDatSizeRx or byte == 0:  # 长度无效
                self.RxIndex = 0
            else:
                self.RxIndex += 1  
                self.cmdLen = byte    
        elif self.RxIndex == 3: # 获取数据体的数据
            self.buf[self.i] = byte
            self.i += 1
            if self.i >= self.cmdLen + 3: # 已收完数据体
                self.RxIndex += 1
        elif self.RxIndex == 4: # 对比 效验码
            self.CS -= byte
            if (self.CS&0xFF) == byte: # 校验正确
                self.buf[self.i] = byte
                self.i += 1
                self.RxIndex += 1  
            else:   
                self.RxIndex = 0  
        elif self.RxIndex == 5: # 结束码
            self.RxIndex = 0
            if byte == self.CmdPacket_End: # 捕获到完整包
                self.buf[self.i] = byte
                self.i += 1  
                self.Cmd_RxUnpack(self.buf[3:self.i-2], self.i-5)    # 处理数据包的数据体
        else:
            self.RxIndex = 0  
            
    def Cmd_RxUnpack(self, buf, DLen):    
        scaleAccel       = 0.00478515625
        scaleQuat        = 0.000030517578125
        scaleAngle       = 0.0054931640625
        scaleAngleSpeed  = 0.06103515625
        scaleMag         = 0.15106201171875
        scaleTemperature = 0.01
        scaleAirPressure = 0.0002384185791
        scaleHeight      = 0.0010728836   
    
        if buf[0] == 0x11:  
            ctl = (buf[2] << 8) | buf[1]  
            print("\n subscribe tag: 0x%04x"%ctl)
            print(" ms: ", ((buf[6]<<24) | (buf[5]<<16) | (buf[4]<<8) | (buf[3]<<0)))

            L = 7   # 从第7字节开始根据 订阅标识tag来解析剩下的数据
            #############################################

            if ((ctl & 0x0004) != 0):
                self.AngleVelX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2 
                print("\tGX: %.3f"%self.AngleVelX)    
                self.AngleVelY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2 
                tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2
                print("\tGZ: %.3f"%tmpZ)   
                # self.AngleVelX = tmpX  
                # vel_y = tmpY
                # vel_z = tmpZ   

            if ((ctl & 0x0040) != 0):
                self.AngleX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
                print("\tangleX: %.3f"%self.AngleX) 
                self.AngleY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
                # print("\tangleY: %.3f"%tmpY)   
                tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
                # print("\tangleZ: %.3f"%tmpZ)   
                # self.AngleX = tmpX    
                # angle_y = tmpY   
                # angle_z = tmpZ     
        else:
            print("------data head not define")    
        self.imu_buffer[0] = imu_read_left.AngleX 
        self.imu_buffer[1] = imu_read_left.AngleY  
        self.imu_buffer[2] = imu_read_left.AngleVelX 
        self.imu_buffer[3] = imu_read_left.AngleVelY   
        self.imu_buffer.flush()  

    def read(self):  
        data = self.Serial_IMU.read(1)       
        buf_len = len(data)   
        if buf_len > 0:              
            self.Cmd_GetPkt(data[0])     

    def ToUint(self, x, x_min, x_max, nbits):  
        span = x_max - x_min  
        if (x < x_min):
            x = x_min  
        if (x > x_max):
            x = x_max   
        toUint=((x - x_min) * ((float)((1 << nbits) - 1) / span))
        return toUint     

    def ToFloat(self, x_int, x_min, x_max, nbits):  
        span = x_max - x_min
        offset_value = x_min
        toFloat= x_int * span / float((((1 << nbits) - 1))) + offset_value
        return toFloat    

    def Reset_buffer(self):   
        self.Serial_IMU.reset_input_buffer()
        self.Serial_IMU.reset_output_buffer()    
            

if __name__ == "__main__":  
    # server_ip = '10.154.28.205'   
    # server_port = 45678  
    # client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)   
    
    # context = zmq.Context()  
    # client_socket = context.socket(zmq.REQ)     
    # server_address = "tcp://192.168.12.112:7794" 
    # client_socket.connect(server_address)   

    ser_port_left = "/dev/ttyUSB3"      
    ser_baudrate = 115200          
    ser_timeout = 0.001         
    imu_read_left = READIMU(ser_port_left)     
        
    # ser_port_right = "/dev/ttyUSB1"      
    # ser_baudrate = 115200          
    # ser_timeout = 0.001            
    # imu_read_right = READIMU(ser_port_right)     
    
    L_IMU_angle = 0.0   
    L_IMU_angle_pre = 0.0   
    L_IMU_vel   = 0.0  
    L_IMU_vel_pre   = 0.0  
    R_IMU_angle = 0.0 
    R_IMU_angle_pre = 0.0   
    R_IMU_vel   = 0.0     
    R_IMU_vel_pre   = 0.0     
    dt = 0.01 
    
    pos_ampl = 50   
    pos_freq = 1   
    
    # for i in range(100):  
    #     imu_read_left.read() 
    #     imu_read_right.read()  
    #     initial_left = imu_read_left.AngleX 
    #     initial_right = imu_read_right.AngleX   
    #     print("++++++++++++++++++++++++++++")   
    
    # csv_filename = "../data/serial_data_reading.csv"  
    # with open(csv_filename, 'a', newline='') as csvfile:  
    #     fieldnames = ['L_IMU_Ang', 'R_IMU_Ang', 'L_IMU_Vel', 'R_IMU_Vel', 'L_Cmd', 'R_Cmd', 'Peak', 'Time']
    #     writer = csv.DictWriter(csvfile, fieldnames=fieldnames)  

    #     # # Write the header only if the file is empty
    #     csvfile.seek(0, 2)
    #     if csvfile.tell() == 0:
    #         writer.writeheader()   
            
    start = time.time()  
     
    count = 0 
    while True:  
        now = time.time() - start   
        
        # L_IMU_angle = pos_ampl * sin(2 * np.pi * pos_freq * now)    
        # R_IMU_angle = pos_ampl * sin(2 * np.pi * pos_freq * now)    
        
        imu_read_left.read()    
        # imu_read_right.read()         
        
        # L_IMU_angle = imu_read_left.AngleX     
        # R_IMU_angle = imu_read_left.AngleY      
        
        # imu_buffer.flush()   
        
        # # L_IMU_angle = imu_read_left.AngleX - initial_left   
        # # R_IMU_angle = imu_read_right.AngleX - initial_right     
        # # L_IMU_vel   = (L_IMU_angle - L_IMU_angle_pre)/dt  
        # # R_IMU_vel   = (R_IMU_angle - R_IMU_angle_pre)/dt    
        # L_IMU_vel   = imu_read_left.AngleVelX 
        # R_IMU_vel   = imu_read_right.AngleVelX   
            
        # L_IMU_vel   = smooth(old_value=L_IMU_vel_pre, value=L_IMU_vel)    
        # R_IMU_vel   = smooth(old_value=R_IMU_vel_pre, value=R_IMU_vel)    
            
        # data = {
        #     'L_IMU_Ang': L_IMU_angle, 
        #     'R_IMU_Ang': R_IMU_angle,  
        #     'L_IMU_Vel': L_IMU_vel,  
        #     'R_IMU_Vel': R_IMU_vel,    
        #     'L_Cmd': 0.0,    
        #     'R_Cmd': 0.0,    
        #     'Peak': 0.0,              
        #     'Time': now    
        # }
        
        # L_IMU_angle_pre = L_IMU_angle    
        # R_IMU_angle_pre = R_IMU_angle    
        # L_IMU_vel_pre = L_IMU_vel   
        # R_IMU_vel_pre = R_IMU_vel   
        print("now :", now)
        print("L_IMU_angle, R_IMU_angle :", L_IMU_angle, R_IMU_angle)    
        # if count%5==0:
        #     render_data = f"{L_IMU_angle:.1f}" + "," + f"{R_IMU_angle:.1f}" + "," + f"{L_IMU_vel:.1f}" + "," + f"{R_IMU_vel:.1f}"  
        #     client_socket.send(render_data.encode())     
        #     response_data = client_socket.recv_string()    
        #     all_list = [item.strip() for item in response_data.split(",")]    
        #     print("received data: ", all_list[0])     
        
        # pos_ampl = float(all_list[0])    
        # pos_freq = float(all_list[1])           
        
        # if count%500 == 0: 
        #     imu_read_left.Reset_buffer() 
             
        # writer.writerow(data)         
        # csvfile.flush()         
        count += 1 