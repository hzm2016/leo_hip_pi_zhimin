import zmq
import numpy as np 
import time 

# 创建一个 ZeroMQ 上下文
context = zmq.Context()  
# 创建 REQ（请求）套接字
socket = context.socket(zmq.REQ)
# 连接到服务器的 IP 地址和端口，假设服务器的 IP 地址是 192.168.1.10
server_address = "tcp://10.154.28.205:7794"    
# server_address = "tcp://192.168.12.112:7794"  
socket.connect(server_address)  
print("Connection is created!!!")    

# # 定义服务器的IP地址和端口号
# server_ip = '10.154.28.205'     
# server_port = 23456  
# client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  

# 创建一个 ZeroMQ 上下文
# context = zmq.Context()   
# socket = context.socket(zmq.REQ)  
# server_address = "tcp://10.154.28.205:7794"     
# socket.connect(server_address)   


L_IMU_angle = 0.0 
R_IMU_angle = 0.0  
L_IMU_vel   = 0.0  
R_IMU_vel   = 0.0  

imu_buffer = np.memmap("RL_controller_leo_zhimin/imu_data.dat", dtype='float32', mode='r+',shape=(4,), offset=4)     
while True:  
    now = time.time()  
    
    L_IMU_angle = imu_buffer[0] 
    R_IMU_angle = imu_buffer[1] 
    L_IMU_vel   = imu_buffer[2] 
    R_IMU_vel   = imu_buffer[3]   
    
    render_data = f"{L_IMU_angle:.1f}" + "," + f"{R_IMU_angle:.1f}" + "," + f"{L_IMU_vel:.1f}" + "," + f"{R_IMU_vel:.1f}"   
    socket.send(render_data.encode())    

    response = socket.recv_string()   
    
    print("now :", now)  
    print("angles :", L_IMU_angle, R_IMU_angle)  