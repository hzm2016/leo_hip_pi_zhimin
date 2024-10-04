import zmq

# 创建一个 ZeroMQ 上下文
context = zmq.Context()  

# 创建 REQ（请求）套接字
socket = context.socket(zmq.REQ)

# 连接到服务器的 IP 地址和端口，假设服务器的 IP 地址是 192.168.1.10
server_address = "tcp://10.154.28.205:7779"  
socket.connect(server_address)  

L_IMU_angle = 100.0 
R_IMU_angle = 100.0  
L_IMU_vel   = 500.0  
R_IMU_vel   = 500.0  

while True:  
    # 发送消息   
    # message = "你好，服务器！"   
    # print(f"发送消息: {message}")   
    # socket.send_string(message)   
    
    render_data = f"{L_IMU_angle:.1f}" + "," + f"{R_IMU_angle:.1f}" + "," + f"{L_IMU_vel:.1f}" + "," + f"{R_IMU_vel:.1f}"  
    # client_socket.sendto(render_data.encode(), (server_ip, server_port))  
    socket.send(render_data.encode())    

    # 接收响应
    response = socket.recv_string()   
    print(f"收到服务器响应: {response}")    