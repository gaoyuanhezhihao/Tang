手动控制模式：
1.双击Calibrate.py文件，在命令行中需要输入串口号，这里是com5
	然后会弹出手动控制界面

远程控制模式：
1.启动SocketControl_MPU6050.py, 输入向下串口号：com6。再输入MPU6050所用的串口号：com7
	向下控制程序启动。可以启动Client_Remote.py来模拟图像识别进程发过来的命令。's'代表停止，
	'f'代表停止，'b'代表向后。如果需要左转弯，先输入'l'，再输入需要转的角度（比如输入90）
	如果需要右转弯，先输入'r'，再输入需要转的角度（比如输入90）