# udp_client.py
import socket

# 定义服务器的IP地址和端口号
server_ip = '10.154.28.205'    # 服务器的IP地址
server_port = 23456  

# 创建UDP客户端Socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 不断发送消息
while True:
    # message = input("Enter message to send (type 'exit' to quit): ")  
    
    data = "12,34,56,89"   
    client_socket.sendto(data.encode(), (server_ip, server_port))
    
    print("data :", data)
    # if message.lower() == 'exit':
    #     print("Client is shutting down...")
    #     break

client_socket.close()  