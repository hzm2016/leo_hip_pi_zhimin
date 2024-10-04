## 板端程序
### 蓝牙连接imu && socket转发到pc

### 使用方法
启动蓝牙：确保蓝牙已经启动
在X3派中，每次开机后需要手动启动一次蓝牙
sudo /usr/bin/startbt6212.sh

连接imu：
  python3 imu_node.py 传感器的MAC地址 转发的目标ip(可不输入ip)
  python3 imu_node.py 7B:1D:DE:EA:17:49 127.0.0.1

上面ip地址是根据目标PC的ip地址来输入的，也可以不输入ip地址启动，
imu的mac地址可以用手机nrfconnect搜索到，或者用我们电脑端配套上位机软件搜索看到

### 依赖安装
  bluez
  libbluetooth-dev
  gatt
对应指令如下，可能不同系统，指令会有差别
  sudo apt-get install bluez
  sudo apt-get install libbluetooth-dev
  sudo pip install gatt


First IMU MAC address :  
