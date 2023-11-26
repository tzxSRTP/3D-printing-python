import numpy as np
import threading
# import socket
import time
from socket import *

PEN=1
PRINT=2

class RobotArm:
    def __init__(self) -> None:
        print("exe:__init__")
        # self.address=('192.168.43.234',5001) #机械臂上树莓派的地址
        # self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        HOST = '192.168.43.234' #or 'localhost'
        PORT = 5001
        BUFSIZ =1024
        ADDR = (HOST,PORT)
        self.conn_socket = socket(AF_INET,SOCK_STREAM)
        self.conn_socket.connect(ADDR)
        #初始位置
        self.stopArm()
        self.init_position=np.array([200,140,360,np.pi-0.01,0,np.pi/2])
        # self.init_position=np.array([200,10,470,np.pi-0.01,0,np.pi/2])
        #set_coords(250,140,360,179.9,0,90,500)

        #等待连接
        time.sleep(1)
        #移动到初始位置
        self.move(self.init_position[0],self.init_position[1],self.init_position[2],self.init_position[3],self.init_position[4],self.init_position[5],40)

    def move(self, x, y, z, rx, ry, rz, v):
        #注：向机械臂发送的单位要求为：xyz为mm，关节角度为°，速度mm/min且为0-2000之间的整数。
        rx*=(180 / np.pi) 
        ry*=(180 / np.pi)
        rz*=(180 / np.pi)
        v=int(v*60)
        mes="set_coords(" + str(x) + "," + str(y) + "," + str(z) + "," + str(rx) + "," + str(ry) + ", " + str(rz) + ", " + str(v) + ")"
        # r=self.sock.sendto(mes.encode('utf-8'), self.address)

        #TODO:为调试，先不发送指令 注释以下部分
        r=self.conn_socket.send(mes.encode())
        if r>=0:
            print("send move:"+str(mes))
        else:
            print("send move failed!")
    
    def stopArm(self):
        mes='task_stop()'
        r=self.conn_socket.send(mes.encode())
        if r>=0:
            print("send msg:stop")
        else:
            print('stop failed!')

    #根据笔的末端位置求解机械臂末端位置x,y,z,为计算正确，固定ry,rz的值
    def calculate_endpoint_position(self, pen_x, pen_y, pen_z, rx, flag):
        if flag==PRINT:
            # # 将欧拉角转换为旋转矩阵R
            R_x = np.array([[1, 0, 0],
                            [0, np.cos(rx), -np.sin(rx)],
                            [0, np.sin(rx), np.cos(rx)]])
            R_y=np.array([[1.0,0.0,0.0],
                        [0.0,1.0,0.0],
                        [0.0,0.0,1.0]])
            R_z=np.array([[0.0,-1.0,0.0],
                        [1.0,0.0,0.0],
                        [0.0,0.0,1.0]])
            
            R = R_z.dot(R_y).dot(R_x)
            R_inv=np.linalg.inv(R)
            pen_pos=np.array([0,-60,75])#笔末端在机械臂末端的坐标
            end_position=np.array([pen_x,pen_y,pen_z])-np.dot(R_inv,pen_pos)
        else:
            end_position=np.array([pen_x,pen_y,pen_z+150])##测得笔尖到末端距离15cm
        return end_position

        #测试不加末端与笔的旋转变换情况
        # return np.array([pen_x,pen_y,pen_z])


    def glue(self,points,t0,dt,pose_given=0):
        if not pose_given:
            self.init_position[2]=210
            current_points=np.hstack([self.init_position[0:3].reshape(3,1), points]) #起始点+所有涂胶点
            next_points=np.hstack([points, self.init_position[0:3].reshape(3,1)]) #所有涂胶点+起始点
            dists=np.linalg.norm(next_points-current_points, axis=0) #相邻点距离
            #计算每一段的速度
            speeds=np.zeros(dists.shape[0])
            speeds[1:-1]=dists[1:-1]/dt
            speeds[0]=dists[0]/t0
            speeds[-1]=dists[-1]/t0
            #逐个点涂胶
            # end_time = time.time()
            for i in range(speeds.shape[0]):
                
                pen_x=next_points[0][i]
                pen_y=next_points[1][i]
                pen_z=next_points[2][i]
                rx=np.pi-0.01
                (x,y,z) = self.calculate_endpoint_position(pen_x, pen_y, pen_z, rx,PEN)

                # speeds[i]=300

                print('x,y,z='+str(x)+' '+str(y)+' '+str(z))
                self.move(x, y, z, rx, 0, np.pi/2, speeds[i])
                #等待移动
                time.sleep(t0+0.02) if i==0 else time.sleep(dt)
        else:
            current_points=np.hstack([self.init_position[0:3].reshape(3,1), points[0:3,:]]) #起始点+所有涂胶点
            next_points=np.hstack([points[0:3,:], self.init_position[0:3].reshape(3,1)]) #所有涂胶点+起始点
            dists=np.linalg.norm(next_points-current_points, axis=0) #相邻点距离
            #计算每一段的速度
            speeds=np.zeros(dists.shape[0])
            speeds[1:-1]=dists[1:-1]/dt
            speeds[0]=dists[0]/t0
            speeds[-1]=dists[-1]/t0
            #逐个点涂胶
            for i in range(speeds.shape[0]):
                #发出移动指令
                # self.move(next_points[0],next_points[1],next_points[2],next_points[3],next_points[4],next_points[5],speeds[i])
                # rx=np.pi-0.01
                # (x,y,z) = self.calculate_endpoint_position(next_points[0], next_points[1],next_points[2], rx)
                self.move(next_points[0], next_points[1],next_points[2],np.pi,0,np.pi/2,speeds[i])
                
                #等待移动
                time.sleep(t0+0.02) if i==0 else time.sleep(dt)
        # mes='set_coord(z,350 ,1000)'
        # r=self.conn_socket.send(mes.encode())
        # if r>=0:
        #     print("send move:"+str(mes))
        # else:
        #     print("send move failed!")
        # print("涂胶完成！")

if __name__ == '__main__':
    # print('start:arm_conreol.py')
    # my_camera=camera.Camera()
    myArm=RobotArm()
    