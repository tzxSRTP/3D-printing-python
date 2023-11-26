import serial
import time
import camera
import cv2
import numpy as np

#初始化部分
class Arduino:
	def __init__(self) ->None:
		port = "COM6"#注意修改对应端口
		baudrate = 9600
		self.ser = serial.Serial(port, baudrate)
		
		# return 1

		#停止
	def stop(self):
		self.ser.write('222'.encode())
		time.sleep(2)
		self.ser.write('222'.encode())
		return 1
	
	#启动直线传送带
	def start_conveyor(self):	
		self.ser.write('000'.encode())#环形传送带启动，系统开始工作
		time.sleep(1)
		self.ser.write('000'.encode())#环形传送带启动，系统开始工作
		time.sleep(1)
		self.ser.write('000'.encode())#环形传送带启动，系统开始工作
		time.sleep(1)
		self.ser.write('111'.encode())

	
	#准备涂胶，告诉舵机开始转，笔头出料
	def servo_turn_on(self):
		self.ser.write('333'.encode())
		return 1

	#涂胶结束，告诉舵机反向转，笔头停止出料
	def servo_turn_off(self):
		self.ser.write('444'.encode())
		return 1

	#送到环形传送带，开始拍照
	def detect_object_for_camera(self):
		self.str1 = self.ser.readline()
		self.str1 = bytes.decode(self.str1)
		if self.str1[0]=='7':#抵达环形传送带
			print('find object')
			return 1
		else:
			return 0	#未抵达环形传送带
	
	#打印完成，结束拍照
	def finish_print(self):
		self.str1 = self.ser.readline()
		self.str1 = bytes.decode(self.str1)
		if self.str1[0]=='8':#打印完成
			return 1
		else:
			return 0#打印未完成

	#质量检测，num是序号，从0开始；Is_Good是标志位
	def quality_check(self,num, Is_Good):
		if Is_Good:	#不为0是正品
			self.ser.write((str(num)+'55').encode())#正品末位是2
		else:
			self.ser.write((str(num)+'56').encode())#次品末位是0
		return 1
	
if __name__ == '__main__':
	camera=camera.Camera()
	arduino=Arduino()
	time.sleep(1)
	arduino.start_conveyor()
	while True:
		flag= arduino.detect_object_for_camera()
		if flag==1:
			time.sleep(5)
			srcImg=camera.get_last_rbg()
			depth_frame = camera.get_last_depth()  
			x1,x2,y1,y2=400,900,400,600
			roiImg=srcImg[y1:y2+1,x1:x2+1,:]
			cv2.imwrite('./measure/measure0.5s_0.jpg', roiImg)
			np.save('depth_image0.npy',depth_frame )
			
			time.sleep(0.5)
			srcImg=camera.get_last_rbg()
			depth_frame = camera.get_last_depth()
			x1,x2,y1,y2=400,900,400,600
			roiImg=srcImg[y1:y2+1,x1:x2+1,:]
			cv2.imwrite('./measure/measure0.5s_1.jpg', roiImg)
			np.save('depth_image1.npy',depth_frame )
			time.sleep(0.5)
			srcImg=camera.get_last_rbg()
			depth_frame = camera.get_last_depth()  
			x1,x2,y1,y2=400,900,400,600
			roiImg=srcImg[y1:y2+1,x1:x2+1,:]
			cv2.imwrite('./measure/measure0.5s_2.jpg', roiImg)
			np.save('depth_image2.npy',depth_frame )
			time.sleep(0.5)
			srcImg=camera.get_last_rbg()
			depth_frame = camera.get_last_depth()  
			x1,x2,y1,y2=400,900,400,600
			roiImg=srcImg[y1:y2+1,x1:x2+1,:]
			cv2.imwrite('./measure/measure0.5s_3.jpg', roiImg)
			np.save('depth_image3.npy',depth_frame )
			arduino.stop()
			print('detect!')
			exit(-1)
		else:
			continue

	# time.sleep(2.5)
	# arduino.stop()
	# arduino.stop()
	# while True:
    #     # 利用接近传感器检测物体是否进入相机范围
	# 	object_detected=arduino.detect_object_for_camera()
	# 	if not object_detected:
	# 		continue
	# 	else:	
	# 		print('detect')
	# while True:
	# 	arduino.detect_object_for_camera()
	

	# port = "COM4"#注意修改对应端口
	# baudrate = 9600
	# ser = serial.Serial(port, baudrate)

	# ser.write('111'.encode())#直线传送带启动，系统开始工作
	# time.sleep(2)
	# ser.write('111'.encode())#直线传送带启动，系统开始工作


	# port = "COM4"#注意修改对应端口
	# baudrate = 9600
	# ser = serial.Serial(port, baudrate)
	# ser.write('111'.encode())#直线传送带启动，系统开始工作

	# port = "COM4"
	# baudrate = 9600
	# ser = serial.Serial(port, baudrate)
	# ser.write('111'.encode())
	# time.sleep(2)
	# ser.write('111'.encode())


	
	# ser.write('111'.encode())
	# ser.write('666'.encode())

	# port = "COM4"
	# baudrate = 9600
	# ser = serial.Serial(port, baudrate)
	# i=0
	# #收信息
	# while(1):
	# 	print(i)
	# 	ser.write('111'.encode())
	# 	time.sleep(1)
	# 	i=i+1
		
		
		