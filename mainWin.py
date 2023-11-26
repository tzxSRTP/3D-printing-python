import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5 import QtCore, QtGui, QtWidgets

from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime
import cv2
import numpy as np

import camera
import serial_with_arduino  
import planner
import arm_control
import time
# import global_var
from PyQt5.QtCore import QThread

EDGE_TYPE=0 
# PRINT_MODE=0


#后端主程序
class BackendThread(QThread):
    startConveyorSignal = pyqtSignal()
    stopConveyorSignal=pyqtSignal()
    # stopArmSignal=pyqtSignal()
    def run(self):
        #初始化相机
        my_camera=camera.Camera()
        if my_camera.kinect:
            print("Initialize Kinect succeeded!")
        else:
            print("Initialize Kinect failed!")
            return
        
        # 初始化机械臂 #TODO 注意！建议每次先用sokit尝试一下:set_coords(250,140,360,179.9,0,90,500)
        arm=arm_control.RobotArm()
        time.sleep(1)

        while True:
            # if PRINT_MODE==0:#随动打印模式
                # 利用接近传感器检测物体是否进入相机范围
            object_detected=arduino.detect_object_for_camera()
            print("TRUE")
            # object_detected=1   
            if not object_detected:
                continue
            else: # 检测到物体，每个物体只进入一次该分支
                # 利用机器视觉算法求解世界坐标系下：3D涂胶轮廓（含法向量）、零件三维模型和传送带速度
                print('detect object')
                
                # TODO:加功能：检测到回路闭环才继续
                time.sleep(7.5)   #TODO:等待物体进入视觉区域,这个要改的话需要修改对应dynamic中l1
                print('0')
                start_time = time.time()
                contour3d,model3d,v_belt=my_camera.get_glue_contour_v2(EDGE_TYPE) #coutour3d.shape==(3,x),每个点包含xyz和nxnynz
                while contour3d.any()==None:
                    contour3d,model3d,v_belt=my_camera.get_glue_contour_v2(EDGE_TYPE) #coutour3d.shape==(3,x),每个点包含xyz和nxnynz

                v_belt=664/17.59-1.4  #T传送带速度设置，目前为35%对应速度
                # v_belt=0

                if (contour3d==None).any():
                    continue
                # 在涂胶轮廓上采样,得到静态涂胶点
                np.savetxt('contour3d.txt', contour3d)
                point_num=35
                points_s=planner.sample_on_contour(contour3d,point_num) 
                # 求解动态涂胶点
                t0=0.3  #拍照到 机械臂移动到第一个涂胶点的时间差
                dt=0.2
                points_d=planner.dynamicalize_v2(points_s,v_belt,t0,dt)#+弧度打印
                # print('points_dy=',points_d)
                #涂胶
                end_time = time.time()
                run_time = end_time - start_time
                print("运行时间：", run_time)
                arm.glue(points_d,t0,dt)


                # 等待当前物块驶出打印区
                while True:
                    result = arduino.finish_print()  # 调用函数获取返回值
                    # time.sleep(5)#TODO
                    # result=1
                    if result == 1:  # 返回值为1时，继续程序运行
                        break  # 退出循环
                    else:
                        time.sleep(0.3)# 打印未完成，可以添加一些等待时间或其他处理
                        pass
                print('execute here')
            # else:#静态打印模式
            #     print('static mode')
            #     x_init = 110  # 95
            #     y_init = 330  # 300
            #     z_init = 375  # 360
            #     z_draw = 370-2  # 200度的打印笔是353
            #     v_init = 20
            #     v_draw =5  # 15

            #     dx = x_init
            #     dy = y_init
            #     dz = z_draw
            #     path_point = np.load('star.npy')
            #     for point in path_point:
            #         arm.move(point[0]*1000+dx, point[1]*1000+dy, point[2]*1000+dz, np.pi - 0.01, 0, np.pi / 2, v_draw)
            #         time.sleep(0.05)
            #     arm.move(dx, dy,dz, np.pi - 0.01, 0, np.pi / 2, v_init)
            

class MainWindow(QMainWindow):
    def __init__(self,arduino):
        super().__init__()
        self.width = 1280
        self.height = 720

        self.arduino=arduino

        # self.arm=arm    #TODO

        # 设置软件图标
        #self.setWindowIcon(QtGui.QIcon("icon.ico"))
        # 设置主界面标题
        self.setWindowTitle("智能3D打印机器人")
        # 设置固定尺寸
        self.setFixedSize(self.width, self.height)
        # 设置主界面背景色
        # self.setStyleSheet("background-color:rgb(84,82,119)")
        # 2.创建二级菜单栏
        # 2.1 创建多分页窗口
        self.stackedWidget_func = QtWidgets.QStackedWidget(self) # QStackedWidget表示多分页的窗口
        
        # self.stackedWidget_func.setObjectName("stackedWidget_func")
        # self.stackedWidget_func.setGeometry(QtCore.QRect(0, 50, 1280, 50))
        # self.stackedWidget_func.setStyleSheet("QWidget{background-color:rgb(211,240,168);border:none}")
        # self.setCentralWidget(self.stackedWidget_func)
        # # 2.2 创建分页对象，并载入分页
        self.page0=Win0()
        self.stackedWidget_func.addWidget(self.page0)
        self.page1 = Basic()
        self.stackedWidget_func.addWidget(self.page1)
        self.page2 = Qual()
        self.stackedWidget_func.addWidget(self.page2)
        self.page3 = Visual()
        self.stackedWidget_func.addWidget(self.page3)
        
        widget = QWidget(self)
        self.setCentralWidget(self.stackedWidget_func)

        # 连接信号和槽函数
        backend_thread.startConveyorSignal.connect(self.startConveyor)
        backend_thread.stopConveyorSignal.connect(self.stopConveyor)

    def startConveyor(self):
        # 启动传送带的操作
        self.arduino.start_conveyor()
    def stopConveyor(self):
        self.arduino.stop()

#主界面
class Win0(QWidget):
    def __init__(self):
        super().__init__()
        self.width = 1280
        self.height = 720

        self.setAutoFillBackground(True)
        #设置窗口背景
        palette=QPalette()
        pixmap=QPixmap('./images/mainWin.png')
        palette.setBrush(QPalette.Background, QBrush(pixmap))     
        self.setPalette(palette)

        #设置按钮
        self.btn1 = None
        self.btn1 = QPushButton(self)
        self.btn1.setFixedSize(296, 79) 
        self.btn1.move(184,257)
        # 设置按钮的不同状态下的图标
        self.btn1.setStyleSheet('''
            QPushButton{border-image:url(./images/btnMain11.png);}
            QPushButton:hover{border-image:url(./images/btnMain12.png);}
            QPushButton:pressed{border-image:url(./images/btnMain12.png);}
        ''') 
        self.btn2 = None
        self.btn2 = QPushButton(self)
        self.btn2.setFixedSize(296, 79) 
        self.btn2.move(184,382)
        # 设置按钮的不同状态下的图标
        self.btn2.setStyleSheet('''
            QPushButton{border-image:url(./images/btnMain21.png);}
            QPushButton:hover{border-image:url(./images/btnMain22.png);}
            QPushButton:pressed{border-image:url(./images/btnMain22.png);}
        ''') 

        self.btn3 = None
        self.btn3 = QPushButton(self)
        self.btn3.setFixedSize(296, 79) 
        self.btn3.move(184,512)
        # 设置按钮的不同状态下的图标
        self.btn3.setStyleSheet('''
            QPushButton{border-image:url(./images/btnMain31.png);}
            QPushButton:hover{border-image:url(./images/btnMain32.png);}
            QPushButton:pressed{border-image:url(./images/btnMain32.png);}
        ''') 

        self.connectSlots()

    def connectSlots(self):
        self.btn1.clicked.connect(self.slot_btn1)
        self.btn2.clicked.connect(self.slot_btn2)
        self.btn3.clicked.connect(self.slot_btn3)

    def slot_btn1(self):
        window.stackedWidget_func.setCurrentIndex(1)  # 将多页面窗口切换至页面序号0
        # self.stackedWidget_func.setCurrentWidget(self.page1)
        print('click btn1')
    def slot_btn2(self):
        window.stackedWidget_func.setCurrentIndex(2)  # 将多页面窗口切换至页面序号0
        return 1
    def slot_btn3(self):
        window.stackedWidget_func.setCurrentIndex(3)  # 将多页面窗口切换至页面序号0
        return 1

#基础设置界面
class Basic(QWidget):
    def __init__(self):
        super().__init__()
        # self.arm=arm # 保存传递进来的RobotArm对象
        self.setFixedSize(1280,720)
        self.setAutoFillBackground(True)
        palette=QPalette()
        pixmap=QPixmap('./images/back_basic.png')
        palette.setBrush(QPalette.Background, QBrush(pixmap))     
        self.setPalette(palette)

        #设置按钮
        self.btnToQual = None
        self.btnToQual = QPushButton(self)
        self.btnToQual.setFixedSize(190, 57); 
        self.btnToQual.move(546,0)
        # 设置按钮的不同状态下的图标
        self.btnToQual.setStyleSheet('''
            QPushButton{border-image:url(./images/btnToQual1.png);}
            QPushButton:hover{border-image:url(./images/btnToQual2.png);}
            QPushButton:pressed{border-image:url(./images/btnToQual2.png);}
        ''') 

        self.btnToVisual = None
        self.btnToVisual = QPushButton(self)
        self.btnToVisual.setFixedSize(190,57) 
        self.btnToVisual.move(747,0)
        # 设置按钮的不同状态下的图标
        self.btnToVisual.setStyleSheet('''
            QPushButton{border-image:url(./images/btnToVisual1.png);}
            QPushButton:hover{border-image:url(./images/btnToVisual2.png);}
            QPushButton:pressed{border-image:url(./images/btnToVisual2.png);}
        ''') 

        #停止运动按钮
        # self.btnStopArm = None
        # self.btnStopArm = QPushButton(self)
        # self.btnStopArm.setFixedSize(190,57) 
        # self.btnStopArm.move(747,0)
        # # 设置按钮的不同状态下的图标
        # self.btnStopArm.setStyleSheet('''
        #     QPushButton{border-image:url(./images/btnStopArm1.png);}
        #     QPushButton:hover{border-image:url(./images/btnStopArm2.png);}
        #     QPushButton:pressed{border-image:url(./images/btnStopArm2.png);}
        # ''') 

        self.initButtonGroup()
        self.connectSlots()

    def initButtonGroup(self):
        # playout1 = QHBoxLayout()
        self.m_pButtonGroup = QButtonGroup(self)
        self.m_pButtonGroup.setExclusive(True)

        self.btnStart = QRadioButton(self)
        self.btnStart.resize(354, 56)
        self.btnStart.move(791, 174)
        startBtnStyle = "\
        QRadioButton::indicator:unchecked {\
            image: url(./images/btnStart2.png);\
        }\
        QRadioButton::indicator:unchecked:hover {\
            image: url(./images/btnStart3.png);\
        }\
        QRadioButton::indicator:checked {\
            image: url(./images/btnStart1.png);\
        }"
        self.btnStart.setStyleSheet(startBtnStyle)

        # playout1.addWidget(self.btnStart)
        self.m_pButtonGroup.addButton(self.btnStart)

        # 按钮2：关闭传送带
        self.btnStop = QRadioButton(self)
        
        self.btnStop.move(791, 249)
        # btnStop.setGeometry(791, 249,354,56)
        StopBtnStyle = "\
        QRadioButton::indicator:unchecked {\
            image: url(./images/btnStop2.png);\
        }\
        QRadioButton::indicator:unchecked:hover {\
            image: url(./images/btnStop3.png);\
        }\
        QRadioButton::indicator:checked {\
            image: url(./images/btnStop1.png);\
        }"
        self.btnStop.setStyleSheet(StopBtnStyle)
        self.btnStop.resize(354, 56)
        # playout1.addWidget(self.btnStop)
        self.m_pButtonGroup.addButton(self.btnStop)

        

        # 第二组按钮：打印模式选择=============================
        # pLayout2 = QHBoxLayout()
        self.m_pButtonGroup2 = QButtonGroup(self)
        self.m_pButtonGroup2.setExclusive(True)

        # 按钮1：静态打印
        self.btnMatch = QRadioButton(self)
        self.btnMatch.resize(354, 56)
        self.btnMatch.move(791, 492)
        matchBtnStyle = "\
        QRadioButton::indicator:unchecked {\
            image: url(./images/btnMatch2.png);\
        }\
        QRadioButton::indicator:unchecked:hover {\
            image: url(./images/btnMatch3.png);\
        }\
        QRadioButton::indicator:checked {\
            image: url(./images/btnMatch1.png);\
        }"
        self.btnMatch.setStyleSheet(matchBtnStyle)

        # pLayout2.addWidget(self.btnMatch)
        self.m_pButtonGroup2.addButton(self.btnMatch)

        # 按钮2：动态打印
        self.btnFree = QRadioButton(self)
        self.btnFree.move(791, 568)
        freeBtnStyle = "\
        QRadioButton::indicator:unchecked {\
            image: url(./images/btnFree2.png);\
        }\
        QRadioButton::indicator:unchecked:hover {\
            image: url(./images/btnFree3.png);\
        }\
        QRadioButton::indicator:checked {\
            image: url(./images/btnFree1.png);\
        }"
        self.btnFree.setStyleSheet(freeBtnStyle)
        self.btnFree.resize(354, 56)

        # pLayout2.addWidget(self.btnFree)
        self.m_pButtonGroup2.addButton(self.btnFree)

        # playout1.setSpacing(10)
        # playout1.setContentsMargins(10, 10, 10, 10)
        # pLayout2.setSpacing(10)
        # pLayout2.setContentsMargins(10, 10, 10, 10)
        
        # self.setLayout(playout1)
        # self.setLayout(pLayout2)
        

    

    def connectSlots(self):
        self.btnToQual.clicked.connect(self.slot_btnToQual)
        self.btnToVisual.clicked.connect(self.slot_btnToVisual)
        self.btnMatch.clicked.connect(self.modeMatch)
        self.btnFree.clicked.connect(self.modeFree)
        self.btnStart.clicked.connect(self.modeStart)
        self.btnStop.clicked.connect(self.modeStop)
        
        # arm = arm_control.RobotArm()  # 创建RobotArm对象
        # self.btnStopArm.clicked.connect(window.arm.stopArm())
    
    def slot_btnToQual(self):
        window.stackedWidget_func.setCurrentIndex(2)

    def slot_btnToVisual(self):
        window.stackedWidget_func.setCurrentIndex(3)

    def modeMatch(self):#静态
        # global PRINT_MODE
        # PRINT_MODE=1
        return 1
    def modeFree(self):#动态
        # global PRINT_MODE
        # PRINT_MODE=0
        return 1
    def modeStart(self):
        backend_thread.startConveyorSignal.emit()

        print("clk modestart")#TODO:
    def modeStop(self):
        backend_thread.stopConveyorSignal.emit()
        print("clk modestop")
    


class Qual(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(1280,720)
        self.setAutoFillBackground(True)
        palette=QPalette()
        pixmap=QPixmap('./images/back_qual.png')
        palette.setBrush(QPalette.Background, QBrush(pixmap))     
        self.setPalette(palette)

        #设置按钮
        self.btnToBasic = None
        self.btnToBasic = QPushButton(self)
        self.btnToBasic.setFixedSize(190, 57); 
        self.btnToBasic.move(343,0)
        # 设置按钮的不同状态下的图标
        self.btnToBasic.setStyleSheet('''
            QPushButton{border-image:url(./images/btnToBasic1.png);}
            QPushButton:hover{border-image:url(./images/btnToBasic2.png);}
            QPushButton:pressed{border-image:url(./images/btnToBasic2.png);}
        ''') 

        self.btnToVisual = None
        self.btnToVisual = QPushButton(self)
        self.btnToVisual.setFixedSize(190,57) 
        self.btnToVisual.move(747,0)
        # 设置按钮的不同状态下的图标
        self.btnToVisual.setStyleSheet('''
            QPushButton{border-image:url(./images/btnToVisual1.png);}
            QPushButton:hover{border-image:url(./images/btnToVisual2.png);}
            QPushButton:pressed{border-image:url(./images/btnToVisual2.png);}
        ''')
        # self.m_pButtonGroup3 = QButtonGroup(self)
        # self.m_pButtonGroup3.setExclusive(True)
        #腐蚀边界
        self.btnFushi = QPushButton(self)
        self.btnFushi.setFixedSize(172, 172)
        self.btnFushi.move(144, 303)
        self.btnFushi.setStyleSheet('''
            QPushButton{border-image:url(./images/btnFushi1.png);}
            QPushButton:hover{border-image:url(./images/btnFushi2.png);}
            QPushButton:pressed{border-image:url(./images/btnFushi3.png);}
        ''')
        # fushiBtnStyle = "\
        # QRadioButton::indicator:unchecked {\
        #     image: url(./images/btnFushi1.png);\
        # }\
        # QRadioButton::indicator:unchecked:hover {\
        #     image: url(./images/btnFushi2.png);\
        # }\
        # QRadioButton::indicator:checked {\
        #     image: url(./images/btnFushi3.png);\
        # }"
        # self.btnFushi.setStyleSheet(fushiBtnStyle)

        # self.m_pButtonGroup3.addButton(self.btnFushi)
        # 按钮2：五角星标记
        self.btnStar = QPushButton(self)
        self.btnStar.move(375, 303)
        self.btnStar.setStyleSheet('''
            QPushButton{border-image:url(./images/btnStar1.png);}
            QPushButton:hover{border-image:url(./images/btnStar2.png);}
            QPushButton:pressed{border-image:url(./images/btnStar3.png);}
        ''')
        # StopBtnStyle = "\
        # QRadioButton::indicator:unchecked {\
        #     image: url(./images/btnStar1.png);\
        # }\
        # QRadioButton::indicator:unchecked:hover {\
        #     image: url(./images/btnStar2.png);\
        # }\
        # QRadioButton::indicator:checked {\
        #     image: url(./images/btnStar3.png);\
        # }"
        # self.btnStar.setStyleSheet(StopBtnStyle)
        
        self.btnStar.setFixedSize(172,172)
        # playout1.addWidget(self.btnStar)
        # self.m_pButtonGroup3.addButton(self.btnStar)

        self.connectSlots()

    def connectSlots(self):
        self.btnToBasic.clicked.connect(self.slot_btnToBasic)
        self.btnToVisual.clicked.connect(self.slot_btnToVisual)
        self.btnFushi.clicked.connect(self.slot_btnFushi)
        self.btnStar.clicked.connect(self.slot_btnStar)
        # self.timer.timeout.connect(self.updateImages)

    
    def slot_btnToBasic(self):
        window.stackedWidget_func.setCurrentIndex(1)
    def slot_btnToVisual(self):
        window.stackedWidget_func.setCurrentIndex(3)
    def slot_btnFushi(self):
        print('clk btnFushi')
        global EDGE_TYPE
        EDGE_TYPE=0
        # global_var.set_type(0)
        

    def slot_btnStar(self):
        global EDGE_TYPE
        EDGE_TYPE=1
        # global_var.set_type(1)
    
#自定义轨迹界面
class Visual(QWidget):
    def __init__(self, parent=None):
        super(Visual, self).__init__(parent)
        #设置标题
        # self.setWindowTitle("智能3D打印机器人")
        self.setFixedSize(1280,720)
        self.setAutoFillBackground(True)
        pixmap=QPixmap('./images/back_self.png')
        palette=QPalette()
        palette.setBrush(QPalette.Background, QBrush(pixmap))     
        self.setPalette(palette)

        #实例化QPixmap类
        self.pix = QPixmap()
        #起点，终点
        self.lastPoint = QPoint()
        self.endPoint = QPoint()
        #初始化
        self.initUi()
        # 初始化储存点坐标的数组
        self.paint_xy = np.array([], dtype=np.int32).reshape(0, 2)
 
    def initUi(self):
        # self.resize(600, 500)  # 窗口大小设置为600*500，这样可以鼠标拖动缩放
        # self.setFixedSize(900, 512)  # 固定窗口大小，不可缩放
 
        self.pix = QPixmap(800, 512)
        self.pix.fill(Qt.white)
        # 偏移量，保证鼠标的位置和画的线点是重合的
        self.offset = QPoint(442,132)

        btn_clear = QPushButton(self)
        btn_clear.setFixedSize(277, 98); 
        btn_clear.move(83,340)
        # 设置按钮的不同状态下的图标
        btn_clear.setStyleSheet('''
            QPushButton{border-image:url(./images/btnClear1.png);}
            QPushButton:hover{border-image:url(./images/btnClear2.png);}
            QPushButton:pressed{border-image:url(./images/btnClear2.png);}
        ''') 
        btn_clear.clicked.connect(self.clear)
 
        btn_save = QPushButton(self)
        btn_save = QPushButton(self)
        btn_save.setFixedSize(277, 98); 
        btn_save.move(83,182)
        # 设置按钮的不同状态下的图标
        btn_save.setStyleSheet('''
            QPushButton{border-image:url(./images/btnSave1.png);}
            QPushButton:hover{border-image:url(./images/btnSave2.png);}
            QPushButton:pressed{border-image:url(./images/btnSave2.png);}
        ''') 
        btn_save.clicked.connect(self.save)

        btn_set = QPushButton(self)
        btn_set = QPushButton(self)
        btn_set.setFixedSize(277, 98); 
        btn_set.move(83,498)
        # 设置按钮的不同状态下的图标
        btn_set.setStyleSheet('''
            QPushButton{border-image:url(./images/btnSet1.png);}
            QPushButton:hover{border-image:url(./images/btnSet2.png);}
            QPushButton:pressed{border-image:url(./images/btnSet2.png);}
        ''') 
        btn_set.clicked.connect(self.set)

        #设置按钮
        self.btnToBasic = None
        self.btnToBasic = QPushButton(self)
        self.btnToBasic.setFixedSize(190, 57)
        self.btnToBasic.move(343,0)
        # 设置按钮的不同状态下的图标
        self.btnToBasic.setStyleSheet('''
            QPushButton{border-image:url(./images/btnToBasic1.png);}
            QPushButton:hover{border-image:url(./images/btnToBasic2.png);}
            QPushButton:pressed{border-image:url(./images/btnToBasic2.png);}
        ''') 

        #设置按钮
        self.btnToQual = None
        self.btnToQual = QPushButton(self)
        self.btnToQual.setFixedSize(190, 57)
        self.btnToQual.move(546,0)
        # 设置按钮的不同状态下的图标
        self.btnToQual.setStyleSheet('''
            QPushButton{border-image:url(./images/btnToQual1.png);}
            QPushButton:hover{border-image:url(./images/btnToQual2.png);}
            QPushButton:pressed{border-image:url(./images/btnToQual2.png);}
        ''') 

        self.btnToBasic.clicked.connect(self.slot_btnToBasic)
        self.btnToQual.clicked.connect(self.slot_btnToQual)

 
    def clear(self):
        self.pix.fill(Qt.white)
        self.update()
        # 初始化储存点坐标的数组
        self.paint_xy = np.array([], dtype=np.int32).reshape(0, 2)
 
    def save(self):
        self.pix.save("draw.jpg")
        if self.paint_xy.shape[0] > 0:
            # paint_xy=paint_xy/4+np.array([[19],[29]])
            np.savetxt("paint_xy.txt", self.paint_xy, fmt='%d')

    #TODO:设置模式切换
    def set(self):
        global EDGE_TYPE
        EDGE_TYPE=2
        # return 1
 
    def paintEvent(self, event):
        pp = QPainter(self.pix)
        pen_color=Qt.blue
        pen_width=20
        pen=QPen(pen_color, pen_width, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
        pp.setPen(pen)
        # 根据鼠标指针前后两个位置绘制直线
        pp.drawLine(self.lastPoint, self.endPoint)
        # 让前一个坐标值等于后一个坐标值，
        # 这样就能实现画出连续的线
        self.lastPoint = self.endPoint
        painter = QPainter(self)
        #绘制画布到窗口指定位置处
        painter.drawPixmap(442, 132, self.pix)
 
    def mousePressEvent(self, event):
        # 鼠标左键按下
        if event.button() == Qt.LeftButton:
            self.lastPoint = event.pos() - self.offset
            # 上面这里减去一个偏移量，否则鼠标点的位置和线的位置不对齐
            self.endPoint = self.lastPoint
            print(self.endPoint)
 
    def mouseMoveEvent(self, event):
        # 鼠标左键按下的同时移动鼠标
        if event.buttons() and Qt.LeftButton:
            self.endPoint = event.pos() - self.offset
            # 进行重新绘制
            self.update()
            #对x y进行重新换算
            x = self.endPoint.x() / 8 + 15
            y = -(64 - self.endPoint.y() / 8 + 20)
            self.paint_xy = np.vstack((self.paint_xy, [x, y]))
 
    def mouseReleaseEvent(self, event):
        # 鼠标左键释放
        if event.button() == Qt.LeftButton:
            self.endPoint = event.pos() - self.offset
            # 进行重新绘制
            self.update()
            
    def slot_btnToBasic(self):
        window.stackedWidget_func.setCurrentIndex(1)
        # return 1
    def slot_btnToQual(self):
        window.stackedWidget_func.setCurrentIndex(2)
        # return 1
    
# class Visual(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setFixedSize(1280,720)
#         self.setAutoFillBackground(True)
#         palette=QPalette()
#         pixmap=QPixmap('./images/back_visual.png')
#         palette.setBrush(QPalette.Background, QBrush(pixmap))     
#         self.setPalette(palette)

#         #设置按钮
#         self.btnToBasic = None
#         self.btnToBasic = QPushButton(self)
#         self.btnToBasic.setFixedSize(190, 57); 
#         self.btnToBasic.move(343,0)
#         # 设置按钮的不同状态下的图标
#         self.btnToBasic.setStyleSheet('''
#             QPushButton{border-image:url(./images/btnToBasic1.png);}
#             QPushButton:hover{border-image:url(./images/btnToBasic2.png);}
#             QPushButton:pressed{border-image:url(./images/btnToBasic2.png);}
#         ''') 

#         #设置按钮
#         self.btnToQual = None
#         self.btnToQual = QPushButton(self)
#         self.btnToQual.setFixedSize(190, 57); 
#         self.btnToQual.move(546,0)
#         # 设置按钮的不同状态下的图标
#         self.btnToQual.setStyleSheet('''
#             QPushButton{border-image:url(./images/btnToQual1.png);}
#             QPushButton:hover{border-image:url(./images/btnToQual2.png);}
#             QPushButton:pressed{border-image:url(./images/btnToQual2.png);}
#         ''') 

#         my_str = ""
#         self.listResult=[]#TODO:待连接
#         self.listResult.append(0)
#         self.listResult.append(1)
#         self.listResult.append(1)
#         self.glue_count=3
#         self.qlab_group = [None] * 20

#     # 表格纵向差47，横向218
#         start = 0 if self.glue_count < 9 else self.glue_count - 8
#         for i in range(start, self.glue_count):  # 显示序号（i）1~8；list编号0~7
#             col = i - start
#             self.qlab_group[col * 2] = QLabel(self)
#             self.qlab_group[col * 2].setGeometry(162, 250 + 47 * col, 85, 36)
#             my_str = str(i + 1).zfill(5)
#             font = QFont("微软雅黑", 16)
#             self.qlab_group[col * 2].setText(my_str)
#             print("workNo.", my_str)
#             print("i=", i)
#             self.qlab_group[col * 2].setFont(font)
#             self.qlab_group[col * 2].setStyleSheet("QLabel{color:black}")
#             self.qlab_group[col * 2].setAlignment(Qt.AlignCenter)

#             self.qlab_group[col * 2 + 1] = QLabel(self)
#             # self.qlab_group[col * 2 + 1].setFont(font)
#             # result = ""
#             if self.listResult[i] == 0:
#                 self.qlab_group[col * 2 + 1].setText("不 合 格")
#                 self.qlab_group[col * 2 + 1].setStyleSheet("QLabel{color:red}")
#                 self.qlab_group[col * 2 + 1].setAlignment(Qt.AlignCenter)
#             else:
#                 self.qlab_group[col * 2 + 1].setText("合 格")
#                 self.qlab_group[col * 2 + 1].setStyleSheet("QLabel{color:green}")
#                 self.qlab_group[col * 2 + 1].setAlignment(Qt.AlignCenter)

#             self.qlab_group[col * 2 + 1].setFont(font)
#             self.qlab_group[col * 2 + 1].setGeometry(342, 250 + 47 * col, 191, 36)

#         #显示合格率
#         count_qualified = self.listResult.count(1)
#         total_elements = len(self.listResult)
#         ratio = count_qualified  / total_elements
#         ratio = round(ratio * 100, 1)
#         self.qlab_ratio=QLabel(self)
#         self.qlab_ratio.setFont(font)
#         self.qlab_ratio.setStyleSheet("QLabel{color:white}")
#         self.qlab_ratio.setAlignment(Qt.AlignCenter)
#         self.qlab_ratio.setText(str(ratio)+"%")
#         self.qlab_ratio.setGeometry(800,530,200,95)

#         #显示成品照片
#         # self.label1 = QLabel(self)
#         # self.label1.move(705,101)

#         # self.updateImages()  # 更新图片

#         # # 创建一个定时器，每隔一段时间检查文件更新并更新图片
#         # self.timer = QTimer(self)
#         # self.timer.start(1000)  # 设置定时器间隔时间（毫秒）

#         self.connectSlots()

#     # def updateImages(self):
#     #     image1_path = "./images/output.jpg"#TODO:更改为成品图片路径

#     #     pixmap1 = QPixmap(image1_path)
#     #     self.label1.setPixmap(pixmap1.scaledToWidth(394))

#     #     self.connectSlots()

#     def connectSlots(self):
#         self.btnToBasic.clicked.connect(self.slot_btnToBasic)
#         self.btnToQual.clicked.connect(self.slot_btnToQual)

    
#     def slot_btnToBasic(self):
#         window.stackedWidget_func.setCurrentIndex(1)
#     def slot_btnToQual(self):
#         window.stackedWidget_func.setCurrentIndex(2)
    


# if __name__ == '__main__':
#     app = QApplication([])
#     window = MainWindow()
#     window.show()
#     app.exec_()


    



if __name__ == '__main__':
    app = QApplication([])
    arduino=serial_with_arduino.Arduino()

    # 创建并启动后端线程
    backend_thread = BackendThread()
    backend_thread.start()

    # 创建并显示前端界面
    window = MainWindow(arduino)
    window.show()

    # 运行主事件循环
    app.exec_()
