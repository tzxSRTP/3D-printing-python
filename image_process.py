import cv2
import numpy as np
import mapper
import test
from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime
import camera
# FUSHI=0
# STAR=1

# 用于分割RGB图片得到二值图片，选出特定的颜色
def segment(img):
    # lower=np.array([0, 190, 190, 0])
    # upper=np.array([140, 255, 255, 255])
    width=img.shape[1]
    height=img.shape[0]
    lower=np.dot(np.ones([height,width,1]), np.array([[0, 190, 190]])).astype(np.uint8)
    upper=np.dot(np.ones([height,width,1]), np.array([[140, 255, 255]])).astype(np.uint8) #lower and upper bound for BGR yellow segmentation.
    # print(img.shape)
    # print(lower.shape)
    return cv2.inRange(img,lower,upper) #需要lower、upper和imgsize相同

# 用于在二值图片上找到涂胶轮廓(腐蚀算法)
def get_edge(img,typeOfEdge,margin=52):#TODO:初始值为15
    if typeOfEdge==0:#腐蚀边界
        #中值滤波
        img_f=cv2.medianBlur(img,11)
        #腐蚀缩小,margin决定腐蚀的程度大小
        kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,ksize=(margin,margin))
        img_e=cv2.erode(img_f,kernel)
        #提取边缘
        edge_img=cv2.Canny(img_e,100.0,300.0)
        edge=cv2.findContours(edge_img, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
        # print("edge: ",edge)
        print('get edge(image_process.get_edge)')
        if edge[0]==():
            edge_return=np.array([None])
        else:
            edge_return=edge[0][0].reshape(-1,2)
        # print(edge_return.shape)
        return edge_img,edge_return
    elif typeOfEdge==1:   #五角星边界
        relative_coords = np.loadtxt('relative_star.txt')
        # print(relative_coords.shape)
        return None,relative_coords
    else:
        paint_coords = np.loadtxt('paint_xy.txt')
        return None,paint_coords
    



# #工件匹配 TODO:还没调
# def match_obj(self):
#     # 读取拍摄的图像
#     image = cv2.imread('./images/yp.bmp', cv2.IMREAD_GRAYSCALE)

#     # 对图像进行阈值处理，获取二值图像
#     _, binary_image = cv2.threshold(image, 128, 255, cv2.THRESH_BINARY)

#     # 查找图像中的轮廓
#     contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     threshold_area=11.5
#     threshold_perimeter=12
#     # 遍历每个轮廓
#     for contour in contours:
#         # 计算轮廓的几何矩，包括面积、周长、重心等
#         area = cv2.contourArea(contour)
#         perimeter = cv2.arcLength(contour, True)
#         M = cv2.moments(contour)
#         cx = int(M["m10"] / M["m00"])
#         cy = int(M["m01"] / M["m00"])

#         # 基于几何特征进行工件的区分和匹配
#         if area > threshold_area and perimeter > threshold_perimeter:
#             # 匹配为矩形
#             cv2.putText(image, 'Rectangle', (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
#         else:
#             # 匹配为其他形状（例如圆形）
#             cv2.putText(image, 'Other Shape', (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

if __name__=='__main__':
    camera=camera.Camera()
    # scrImg=cv2.imread('./catch.jpg')
    srcImg=camera.get_last_rbg()
    x1,x2,y1,y2=400,900,440,650     #ROI范围
    roiImg=srcImg[y1:y2+1,x1:x2+1,:]
    roiImg_copy=np.array(roiImg)
    roiImg=segment(roiImg)

    #TODO:获取深度表，现在用的还是深度相机获取
    depth_map=mapper.depth_2_color_space(camera.kinect, PyKinectV2._DepthSpacePoint, camera.kinect._depth_frame_data, return_aligned_image=True)

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
    # 显示图像
    # cv2.imshow("Contour Image", roiImg_copy)
    # cv2.waitKey(0)
    edgeType=0
    _,edge=get_edge(roiImg,edgeType) #腐蚀算法
    if (edge==None).any():
        print("edge not found!")
    print('edge:',edge)
    if edgeType==1:
        edge=edge+np.array([min_x,max_y])
    else:
    #在rgb图上画出找到的轮廓用以显示
        roiImg=cv2.drawContours(roiImg_copy, (edge,), 0, color=(0,0,255))
        cv2.imwrite('output3.jpg', roiImg)
    #TODO:物体坐标系零点配准，保证涂胶起点
    
    #转换为原图像的像素坐标
    edge[:,0]=edge[:,0]+x1
    edge[:,1]=edge[:,1]+y1
    min_x=min_x+x1
    max_y=max_y+y1
    #涂胶点的像素坐标u和v
    u=edge[:,0].reshape(-1) #横坐标
    v=edge[:,1].reshape(-1) #纵坐标

    #获取轮廓的深度信息，这里还是在图像坐标系中获取的
    depths=depth_map[v,u,0]

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
    zero_x=(min_x-u0)*Zc/fx
    zero_y=(max_y-v0)*Zc/fy
    #轮廓在物体坐标系中坐标,都是负的
    Xc = -abs(Xc - zero_x)
    Yc = -abs(Yc - zero_y)
    
    one_line=np.ones(Xc.shape[0])
    contour_in_camera=np.vstack([Xc,Yc,Zc,one_line]) #shape=(4,x)

    # roiImg=cv2.drawContours(roiImg_copy, (points_relative,), 0, color=(0,0,255))
    # cv2.imshow("glue contour",roiImg)
    # cv2.waitKey(0)
    # plt.imshow(cv2.cvtColor(roiImg, cv2.COLOR_BGR2RGB))
    # plt.show(block=False)
    # cv2.imwrite('output.jpg', roiImg)

    # print('contour_in_camera',contour_in_camera)

    #TODO:不同形状轨迹规划，不同形状工件匹配







    
