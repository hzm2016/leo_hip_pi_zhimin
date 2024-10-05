import zmq
import numpy as np 
import time 

# 创建一个 ZeroMQ 上下文
context = zmq.Context()  

# 创建 REQ（请求）套接字
socket = context.socket(zmq.REQ)

# 连接到服务器的 IP 地址和端口，假设服务器的 IP 地址是 192.168.1.10
# server_address = "tcp://10.154.28.205:7794"  
server_address = "tcp://192.168.12.112:7794"  

socket.connect(server_address)  

L_IMU_angle = 100.0 
R_IMU_angle = 100.0  
L_IMU_vel   = 500.0  
R_IMU_vel   = 500.0  

while True:  
    now = time.time()  
    # L_IMU_angle = pos_ampl * sin(2 * np.pi * pos_freq * now)    
    # R_IMU_angle = pos_ampl * sin(2 * np.pi * pos_freq * now)    
    
    imu_buffer = np.memmap("RL_controller_leo_zhimin/imu_data.dat", dtype='float32', mode='r+',shape=(4,), offset=4)     
    L_IMU_angle = imu_buffer[0] 
    R_IMU_angle = imu_buffer[1] 
    L_IMU_vel   = imu_buffer[2] 
    R_IMU_vel   = imu_buffer[3]   
    
    # render_data = f"{L_IMU_angle:.1f}" + "," + f"{R_IMU_angle:.1f}" + "," + f"{L_IMU_vel:.1f}" + "," + f"{R_IMU_vel:.1f}"  
    # # client_socket.sendto(render_data.encode(), (server_ip, server_port))  
    # socket.send(render_data.encode())    

    # # 接收响应
    # response = socket.recv_string()   
    # print(f"收到服务器响应: {response}")    
    
    print("now :", now)  
    print("angles :", L_IMU_angle, R_IMU_angle)  