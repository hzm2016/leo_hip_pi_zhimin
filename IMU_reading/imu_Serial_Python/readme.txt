需安装pyserial库 和numpy 库
	可用以下指令安装pyserial
		pip install pyserial
	可用以下指令安装numpy
		pip install numpy

		
源码里的第7行 ser_port = "COM3"     #此处需要替换为对应使用的串口号，windows系统写成COMx，若是linux则要根据所用系统进行调整如写成/dev/ttyUSBx或/dev/ttySx
源码里的第7行 ser_baudrate = 115200 #此处需要替换为正确的串口波特率
执行 imupy.py 即可
