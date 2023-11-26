# from pykinect2 import PyKinectV2
# from pykinect2 import PyKinectRuntime
# import cv2
# import numpy as np
# import camera
# import serial_with_arduino
# import planner
# import arm_control
# import time
# from PyQt5.QtWidgets import QApplication
# from PyQt5.QtCore import QThread


# def main():
#     #初始化相机
#     my_camera=camera.Camera()
#     if my_camera.kinect:
#         print("Initialize Kinect succeeded!")
#     else:
#         print("Initialize Kinect failed!")
#         return
    
#     # 初始化机械臂
#     arm=arm_control.RobotArm()

#     # 初始化串口（与Arduino通讯）
#     arduino=serial_with_arduino.Arduino()
#     print('init serial_with_arduino')
#     time.sleep(1)
#     # arduino.start_conveyor()#TODO:启动传送带，后续写进用户界面控制

#     while True:
#         # 利用接近传感器检测物体是否进入相机范围
#         # object_detected=arduino.detect_object_for_camera()
#         object_detected=1   #TODO
#         if not object_detected:
#             continue
#         else: # 检测到物体，每个物体只进入一次该分支
#             # 利用机器视觉算法求解世界坐标系下：3D涂胶轮廓（含法向量）、零件三维模型和传送带速度
#             print('detect object')
#             start_time = time.time()

#             # TODO:加功能：检测到回路闭环才继续
#             time.sleep(1)   #TODO:等待物体进入视觉区域,这个要改的话需要修改对应dynamic中l1
#             contour3d,model3d,v_belt=my_camera.get_glue_contour_v2(2) #coutour3d.shape==(3,x),每个点包含xyz和nxnynz
#             while contour3d.any()==None:
#                 contour3d,model3d,v_belt=my_camera.get_glue_contour_v2(2) #coutour3d.shape==(3,x),每个点包含xyz和nxnynz

#             # v_belt=664/17.59   #T传送带速度设置，目前为35%对应速度
#             v_belt=0

#             if (contour3d==None).any():
#                 continue
#             # 在涂胶轮廓上采样,得到静态涂胶点
#             np.savetxt('contour3d.txt', contour3d)
#             point_num=38
#             points_s=planner.sample_on_contour(contour3d,point_num) 
#             # 求解动态涂胶点
#             t0=1.1  #拍照到 机械臂移动到第一个涂胶点的时间差，TODO:需要调试
#             dt=0.2
#             points_d=planner.dynamicalize_v2(points_s,v_belt,t0,dt)#+弧度打印
#             # print('points_dy=',points_d)
#             #涂胶
#             end_time = time.time()
#             arm.glue(points_d,t0,dt)

#             run_time = end_time - start_time
#             print("运行时间：", run_time)

#             # 等待当前物块驶出打印区
#             while True:
#                 result = arduino.finish_print()  # 调用函数获取返回值
#                 time.sleep(5)#TODO
#                 result=1
#                 if result == 1:  # 返回值为1时，继续程序运行
#                     break  # 退出循环
#                 else:
#                     time.sleep(0.3)# 打印未完成，可以添加一些等待时间或其他处理
#                     pass
            

# if __name__ == '__main__':
#     main()








