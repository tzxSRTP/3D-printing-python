import numpy as np
import time
from socket import *

class armControl:
    def __init__(self) -> None:
        HOST = '192.168.43.234' #or 'localhost'
        PORT = 5001
        BUFSIZ =1024
        ADDR = (HOST,PORT)
        conn_socket = socket(AF_INET,SOCK_STREAM)
        conn_socket.connect(ADDR)
        #初始位置(这里可以根据需要改一下)
        init_position=np.array([200,140,250,np.pi-0.01,0,np.pi/2])
        #等待连接
        time.sleep(1)
        #移动到初始位置
        self.move(self.init_position[0],self.init_position[1],self.init_position[2],self.init_position[3],self.init_position[4],self.init_position[5],30)
    
    def move(self,x, y, z, rx, ry, rz, v):
            #注：向机械臂发送的单位要求为：xyz为mm，关节角度为°，速度mm/min且为0-2000之间的整数。
            rx*=(180 / np.pi) 
            ry*=(180 / np.pi)
            rz*=(180 / np.pi)
            v=int(v*60)
            mes="set_coords(" + str(x) + "," + str(y) + "," + str(z) + "," + str(rx) + "," + str(ry) + ", " + str(rz) + ", " + str(v) + ")"

            #TODO:为调试，可以先不发送指令 注释以下部分
            r=self.conn_socket.send(mes.encode())
            if r>=0:
                print("send move:"+str(mes))
            else:
                print("send move failed!")

    #停止运动指令
    def stopMove(self):
        mes='task_stop()'
        r=self.conn_socket.send(mes.encode())
        if r>=0:
            print('stop move')
        else:
            print("stop failed!")


if __name__=='__main__':
   