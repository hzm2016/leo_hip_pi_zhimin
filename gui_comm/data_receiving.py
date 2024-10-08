import zmq    

context = zmq.Context()    
socket = context.socket(zmq.REP)     
socket.bind("tcp://127.0.0.1:5555")     
count = 0 
print("count :", count)  
while True:
    message = socket.recv_string()
    print(f"收到消息: {message}")
    socket.send_string("消息已收到")   
    
    count += 1 
    print("count :", count)   