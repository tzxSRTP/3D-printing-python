from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime
import cv2
import numpy as np
import image_process
import mapper
import time
from PIL import Image
import matplotlib.pyplot as plt
import open3d as o3d


class Camera():
    def __init__(self) -> None:
        self.kinect=PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth | PyKinectV2.FrameSourceTypes_Color)
        # warmup=100
        # while warmup>0:
        #     self.get_last_rbg()
        #     warmup-=1

    # 获取深度图, 默认尺寸424x512
    # @return shape为(424,512,3)的numpy array，三个通道值相同
    def get_last_depth(self):
        frame = self.kinect.get_last_depth_frame()
        frame = frame.astype(np.uint8)
        dep_frame = np.reshape(frame, [424, 512])
        return cv2.cvtColor(dep_frame, cv2.COLOR_GRAY2RGB)

    # 获取rgb图, 1080x1920x3
    # @return shape为(1080,1920,3)的numpy array
    def get_last_rgb(self)->np.ndarray:
        frame=np.zeros([1,1])
        while (frame==0).all(): #循环读取，直到不是全0           
            frame = self.kinect.get_last_color_frame()
        print("get rgb frame!")
        return np.reshape(frame, [1080, 1920, 4])[:, :, 0:3]

    def get_point_cloud(self):
        # 首先获取rgb图到空间点坐标的映射
        rgb=self.get_last_rgb() #获取当前rgb图像
        world_point_map=mapper.color_2_world(self.kinect, self.kinect._depth_frame_data, PyKinectV2._CameraSpacePoint) #获取了rgb图到空间点坐标的映射
        world_point_map=np.asarray(world_point_map) #转为np array格式

        # 拼接rgb颜色值，形成(x,y,z,r,g,b)的点格式
        world_point_rgb=np.concatenate([world_point_map,rgb], axis=2)
        point_cloud=world_point_rgb.reshape(-1,6).T #变为(6,n)的点云格式
        return point_cloud
    
    def filter_point_cloud(self, point_cloud):
        # 从整帧点云中滤出零件点云
        # point_cloud.shape=(6,n)
        # 首先使用颜色阈值进行筛选
        color_lower=[0, 190, 190]
        color_upper=[140, 255, 255]
        point_cloud_filtered=[]
        for point in point_cloud.T:
            if cv2.inRange(point[3:], color_lower, color_upper):
                point_cloud_filtered.append(point)
        point_cloud_filtered=np.asarray(point_cloud_filtered).T #转回(6,n)的格式

        # 使用dbscan算法过滤掉离群点
        from sklearn.cluster import DBSCAN
        from collections import Counter
        points=points_filtered.T[:,0:3]
        dbscan=DBSCAN(eps=0.1,min_samples=20)
        if points.shape[1]==3:
            dbscan.fit(points)
        else:
            dbscan.fit(points[:,0:3])
        # 找到数量最多的点群
        counter=Counter(dbscan.labels_)
        main_idx=counter.most_common(2)[0][0]  
        # 如果是-1最多，那取其余中最多的cluster
        if main_idx==-1:
            main_idx=counter.most_common(2)[-1][0]     
        print("counter:",counter)
        print("main_idx:",main_idx)
        points_filtered=points[dbscan.labels_==main_idx]

        return points_filtered

    # 获取涂胶轮廓的核心函数
    # @return contour3d.shape=(3,x), 世界坐标系下的三维涂胶轮廓，x为轮廓上的点数
    # @return model3d
    # @return v_belt
    def get_glue_contour(self):
        #获取图像
        srcImg=self.get_last_rbg()

        #获取rgb到depth的映射表
        depth_map=mapper.depth_2_color_space(self.kinect, PyKinectV2._DepthSpacePoint, self.kinect._depth_frame_data, return_aligned_image=True)
        # print('depth_map',depth_map[420:450,450:500])
        #roi范围
        x1,x2,y1,y2=400,900,440,650
        roiImg=srcImg[y1:y2+1,x1:x2+1,:]
        roiImg_copy=np.array(roiImg)

        #分割rgb零件图得到二值图像（零件为黄色）
        roiImg=image_process.segment(roiImg)
        # cv2.imshow("ROI Image", roiImg)
        # cv2.waitKey(0)
        #计算轮廓的二值图像（轮廓为白色）和轮廓点像素坐标
        _,edge=image_process.get_edge(roiImg) #edge.shape=(x,2)
        if (edge==None).any():
            print("edge not found!")
            return edge,None,None

        #在rgb图上画出找到的轮廓用以显示
        roiImg=cv2.drawContours(roiImg_copy, (edge,), 0, color=(0,0,255))
        # cv2.imshow("glue contour",roiImg)
        # cv2.waitKey(0)
        cv2.imwrite('output.jpg', roiImg)

        #转换为原图像的像素坐标
        edge[:,0]=edge[:,0]+x1
        edge[:,1]=edge[:,1]+y1
        #涂胶点的像素坐标u和v
        u=edge[:,0].reshape(-1) #横坐标
        v=edge[:,1].reshape(-1) #纵坐标
        #获取轮廓的深度信息
        depths=depth_map[v,u,0]
        # print(depths)
        #相机参数
        fx = 1068.169623
        fy = 1068.258222
        u0 = 952.5807635 #1920
        v0 = 537.6288875 #1080
        #求解相机坐标系下的轮廓点坐标
        Zc=depths.reshape(-1)
        assert Zc.shape==u.shape, "Zc and u have different shapes!"
        Xc=(u-u0)*Zc/fx
        Yc=(v-v0)*Zc/fy
        one_line=np.ones(Xc.shape[0])
        contour_in_camera=np.vstack([Xc,Yc,Zc,one_line]) #shape=(4,x)

        print('contour_in_camera',contour_in_camera)

        #转换到世界坐标系
        # T_c_w=np.array([[0,1,0,329.4],
        #                  [1,0,0,129.9],
        #                  [0,0,-1,835],
        #                  [0,0,0,1]])
        #xyz可以在这里调整，不用改符号，加笔版
        # T_c_w=np.array([[0,1,0,200.4],
        #                  [-1,0,0,103.9],
        #                  [0,0,-1,859],
        #                  [0,0,0,1]])
        
        #用笔版
        T_c_w=np.array([[0,1,0,325.4],
                         [-1,0,0,123.9],
                         [0,0,-1,849],
                         [0,0,0,1]])
        #不加笔版
        # T_c_w=np.array([[0,1,0,280.4],
        #                  [-1,0,0,97.9],
        #                  [0,0,-1,914],
        #                  [0,0,0,1]])
        contour_in_world=np.dot(T_c_w,contour_in_camera)
        #注意返回的格式是(3,x)而非(4,x)
        v_belt=720/13.34#speed=50测速
        return contour_in_world[0:3,:],None,v_belt

    
    def get_glue_contour_v2(self,edgeType):
        srcImg=self.get_last_rbg()
        x1,x2,y1,y2=500,900,440,650     #ROI范围
        roiImg=srcImg[y1:y2+1,x1:x2+1,:]
        # roiImg = cv2.flip(roiImg, 1)    #左右镜像之后
        roiImg_copy=np.array(roiImg)
        roiImg=image_process.segment(roiImg)

        #TODO:获取深度表，现在用的还是深度相机获取
        depth_map=mapper.depth_2_color_space(self.kinect, PyKinectV2._DepthSpacePoint, self.kinect._depth_frame_data, return_aligned_image=True)

        # 查找轮廓
        contours, hierarchy = cv2.findContours(roiImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # 找到最左下角的点（作为物体坐标系原点）
        min_x =800
        max_y = 0
        for contour in contours:
            for point in contour:
                x, y = point[0]
                if x < min_x:
                    min_x = x
                if y > max_y:
                    max_y = y
        
        # 在最左下角的点上绘制一个圆
        cv2.circle(roiImg_copy, (min_x, max_y), 5, (0, 0, 255), -1)  # 在最右下角点绘制一个半径为5的红色实心圆


        _,edge=image_process.get_edge(roiImg,edgeType) #腐蚀算法
        if (edge==None).any():
            print("edge not found!")
            return edge,None,None#TODO:add
        # print('edge:',edge)
        if edgeType==1 or edgeType==2:
            edge=edge+np.array([min_x,max_y])
        else:
        #在rgb图上画出找到的轮廓用以显示
            roiImg=cv2.drawContours(roiImg_copy, (edge,), 0, color=(0,0,255))
            cv2.imwrite('output2.jpg', roiImg)
        
        #转换为原图像的像素坐标
        edge[:, 0] = edge[:, 0].astype(int) + x1
        edge[:, 1] = edge[:, 1].astype(int) + y1
        min_x = int(min_x + x1)
        max_y = int(max_y + y1)
        #涂胶点的像素坐标u和v
        u=edge[:,0].reshape(-1).astype(int) #横坐标
        v=edge[:,1].reshape(-1).astype(int) #纵坐标

        #获取轮廓的深度信息，这里还是在图像坐标系中获取的
        depths=depth_map[v,u,0]

        #相机参数
        fx = 1068.169623
        fy = 1068.258222
        u0 = 952.5807635 #1920
        v0 = 537.6288875 #1080
        #求解相机坐标系下的轮廓点坐标
        Zc=depths.reshape(-1)
        Zc.fill(650)    #TODO:这里是距离相机高度
        # Zc=Zc+12
        # print(Zc)
        assert Zc.shape==u.shape, "Zc and u have different shapes!"
        Xc=(u-u0)*Zc/fx
        Yc=(v-v0)*Zc/fy
        zero_x=(min_x-u0)*Zc/fx
        zero_y=(max_y-v0)*Zc/fy
        #轮廓在物体坐标系中坐标,都是负的
        Xc = -abs(Xc - zero_x)
        Yc = -abs(Yc - zero_y)
        
        one_line=np.ones(Xc.shape[0])
        contour_in_camera=np.vstack([Xc,Yc,Zc,one_line]) #shape=(4,x)
        T_c_w=np.array([[0,1,0,0],
                         [-1,0,0,0],
                         [0,0,1,0],
                         [0,0,0,1]])
        #不加笔版
        # T_c_w=np.array([[0,1,0,280.4],
        #                  [-1,0,0,97.9],
        #                  [0,0,-1,914],
        #                  [0,0,0,1]])
        contour_in_world=np.dot(T_c_w,contour_in_camera)
        return contour_in_world[0:3,:],None,1
        



    
if __name__ == "__main__":
    # camera=Camera()
    # srcImg=camera.get_last_rbg()
    # x1,x2,y1,y2=400,900,400,600
    # roiImg=srcImg[y1:y2+1,x1:x2+1,:]
    # cv2.imshow('img',roiImg)
    # cv2.waitKey()
    # cv2.imwrite('./measure/measure1.jpg', roiImg)

    # contour3d,model3d,v_belt=camera.get_glue_contour_v2()


    # 2023/12/13 测试获取rgb图、depth图和点云，并可视化
    camera=Camera()
    rgb=camera.get_last_rgb()
    cv2.imshow('rgb',rgb)
    cv2.waitKey()

    depth=camera.get_last_depth()
    cv2.imshow('depth',depth)
    cv2.waitKey()

    points=camera.get_point_cloud()
    pcd=o3d.geometry.PointCloud()
    pcd.points=o3d.utility.Vector3dVector(points[0:3,:].T)
    pcd.colors=o3d.utility.Vector3dVector(points[3:,:].T/255)
    o3d.visualization.draw_geometries([pcd])
    o3d.io.write_point_cloud("saved_point_cloud.ply", pcd)





    