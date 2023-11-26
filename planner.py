import numpy as np


def transform(points_camera):
    return 1

def sample_on_contour(contour, point_num):
    contour_length = contour.shape[1]
    step = max(contour_length // point_num, 1)  # 至少取一个点
    
    sampled_contour = contour[:, ::step]
    
    # 如果最后一个点不在采样点中，添加最后一个点到采样点中
    if sampled_contour.shape[1] < point_num:
        sampled_contour = np.concatenate((sampled_contour, contour[:, -1].reshape(-1, 1)), axis=1)
    
    # 线性插值填充缺失的采样点
    if sampled_contour.shape[1] < point_num:
        missing_points = point_num - sampled_contour.shape[1]
        step_size = contour_length / (point_num - 1)
        
        for i in range(missing_points):
            index = int(i * step_size)
            interpolated_point = contour[:, index].reshape(-1, 1)
            sampled_contour = np.insert(sampled_contour, i+1, interpolated_point, axis=1)

    # # 将前3个点添加到数组的末尾
    # sampled_contour = np.concatenate((sampled_contour, sampled_contour[:, :6]), axis=1)
    # 将第一个点重复两遍，并插入到数组的开头
    first_point = np.copy(sampled_contour[:, 0])
    sampled_contour = np.insert(sampled_contour, 0, [first_point, first_point,first_point, first_point], axis=1)
    
    up_point = np.copy(sampled_contour[:, -1])
    up_point[2] =up_point[2]-30
    sampled_contour = np.concatenate((sampled_contour, up_point.reshape(-1, 1)), axis=1)
    # up_point2=np.copy(sampled_contour[:, 0])
    sampled_contour[2, 0] -= 8
    
    
    
    
    return sampled_contour
    return contour

# 计算世界坐标系下的动态涂胶点坐标
# @param points_s: 静态涂胶点坐标，shape=(3,x) if norm_available else (6,x)
# @param v_belt: 传送带速度大小
# @param t_delay: 从静态涂胶点时间（拍照时间）到涂第一个涂胶点的时间差
# @param dt: 两个涂胶点间的时间差
def dynamicalize(points_s,v_belt,t_delay,dt):
    #把起始点加到最后，形成闭环
    points_s=np.hstack([ points_s, points_s[:,0].reshape(-1,1) ])
    point_num=points_s.shape[1]
    #计算所有点的平移量（y方向）
    delta_t=np.array(range(point_num))*dt+t_delay
    delta_y=delta_t*v_belt
    #计算动态涂胶点坐标
    points_s[1,:]=points_s[1,:]-delta_y
    return points_s

def dynamicalize_v2(points_s,v_belt,t_delay,dt):#注意传入的points_s是在物体坐标系中的位置

    #把起始点加到最后，形成闭环
    points_s=np.hstack([ points_s, points_s[:,0].reshape(-1,1) ])
    point_num=points_s.shape[1] 

    # l1=414    
    # R=300  
    # l1=345       #TODO:直线段长度(理应是拍摄时物体坐标系原点到弯道开始)，+v_belt*sleeptime
    # l1=332  #sleep 8.5
    l1=372   #sleep 7.5
    R=361       #传送带弧度段半径,注意是坐标系原点所在半径
    theta=np.zeros(point_num)
    delta_y=np.zeros(point_num)
    delta_x=np.zeros(point_num)
    delta_t=np.zeros(point_num)
    if v_belt==0:
        point_d=200
    else:
        point_d=int((l1/v_belt-t_delay)/dt)+1   #点数分割线
    print('point_d=',point_d,';','point_num=',point_num)
    points_dy = np.array(points_s, dtype=float)#动态涂胶点
    P_BX=345.5    #TODO:机械臂坐标系原点到物体坐标系起点（拍照时）原点,待调
    P_BY=l1-150
    if point_num<=point_d:   #在直线段上可完成打印
        delta_t=np.array(range(point_num))*dt+t_delay
        delta_y=delta_t*v_belt
        points_dy[1,:]=points_s[1,:]-delta_y+P_BY
        points_dy[0,:]=points_s[0,:]+P_BX
    else:       #需要运动到弧线段
        for i in range(point_d):
            delta_t[i] = i * dt + t_delay
            delta_y[i]=delta_t[i]*v_belt
            points_dy[1,i]=points_s[1,i]-delta_y[i]+P_BY
            points_dy[0,i]=points_s[0,i]+P_BX
        for i in range(point_d,point_num):
            theta[i]=((i*dt+t_delay)*v_belt-l1)/R
            # print(theta[i])
            delta_y[i]=l1 + R*np.sin(theta[i])  #均为绝对值，注意符号
            delta_x[i]=R-R*np.cos(theta[i])
            # print('delta_y['+str(i)+']='+str(delta_y[i]))
            # print('delta_x['+str(i)+']='+str(delta_x[i]))
            #注意这里旋转是z轴逆方向，故取负
            R_z=np.array([[np.cos(-theta[i]), -np.sin(-theta[i])],
                           [np.sin(-theta[i]), np.cos(-theta[i])]])
            R_BA=np.linalg.inv(R_z)
            # R_BA=R_z
            # print("R_z"+str(R_z))
            # print("R_BA"+str(R_BA))
            #动态涂胶点横坐标
            # print("pointsxy1="+str(points_s[0, i].astype(float))+','+str(points_s[1, i].astype(float)))
            points_dy[0, i] =R_BA[0,0]*points_s[0, i].astype(float)+R_BA[1,0]*points_s[1, i].astype(float)+P_BX-delta_x[i]
            # print(R_BA[0,0]*points_s[0, i].astype(float)+R_BA[1,0]*points_s[1, i].astype(float))
            #动态涂胶点纵坐标
            points_dy[1, i] =R_BA[0,1]*points_s[0, i].astype(float)+R_BA[1,1]*points_s[1, i].astype(float)+P_BY-delta_y[i]
            # print(R_BA[0,1]*points_s[0, i].astype(float)+R_BA[1,1]*points_s[1, i].astype(float))
    # print(delta_x)
    #TODO:换算成笔末端高度，可能需要调 被减数为平台到相机高度
    points_dy[2,:]=842-points_dy[2,:]

    return points_dy