import socket
import time
import math
import robot_model
from math import atan2, sqrt

ESP_IP = " "
ESP_PORT = 5000

processed_angle_arr = bytearray(20) # 二维数组表示处理后的角度数据
angle_arr = [[0, 0, 0] for _ in range(6)] # 二维数组表示目标角度数据
t_angle_arr = bytearray(20)# 最新的角度数据
body_endpoint = [[0, 0, 0] for _ in range(6)] # 二维数组表示身体末端点坐标
target_body_endpoint = [[0, 0, 0] for _ in range(6)] # 二维数组表示目标身体末端点坐标
leg_endpoint = [[0, 0, 0] for _ in range(6)] # 二维数组表示腿端点坐标

# class get_angle:
#     def __init__(self):
#         pass

#     def get_angle(self):
#         #先省略具体实现
#         pass

# class esp8266:
#     def __init__(self):
#         self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         self.sock.connect((ESP_IP, ESP_PORT))
#         print("Connected to ESP")
#         print(self.sock.recv(1024).decode())  # 接收欢迎消息
    
#     def send_angle(self, angle_data):
#         angle_data =  bytearray(processed_angle_arr) #将processed_angle_arr转换为字节数组
#         angle_data.append(0x0A) #添加换行符作为数据结束标志
#         self.sock.sendall(angle_data)
#         print("angle data has been sent,waiting for response...")
#         import time
#         time.sleep(0.5)  # 等待ESP处理数据
class Quaternion_tools:
    def __init__(self):
        pass
    #四元数工具

    #计算四元数的共轭，四元数的共轭可以用于将一个向量从一个坐标系转换到另一个坐标系，或者用于计算两个四元数之间的差异
    def quat_conjugate(self,q):
        w, a, b, c = q
        return [w, -a, -b, -c]
    
    #将一个向量v按照四元数q表示的旋转进行旋转变换，得到旋转后的向量
    def rotate_vector_by_quaternion(self, v, q):
        x, y, z = v
        w, a, b, c = q

        x_new = (1 - 2*b**2 - 2*c**2)*x + (2*a*b - 2*c*w)*y + (2*a*c + 2*b*w)*z
        y_new = (2*a*b + 2*c*w)*x + (1 - 2*a**2 - 2*c**2)*y + (2*b*c - 2*a*w)*z
        z_new = (2*a*c - 2*b*w)*x + (2*b*c + 2*a*w)*y + (1 - 2*a**2 - 2*b**2)*z

        return [x_new, y_new, z_new]
    
    #四元数归一化，确保四元数的模长为1，避免由于计算误差导致的四元数失效
    def quat_normalize(self, q):
        w, a, b, c = q
        norm = math.sqrt(w**2 + a**2 + b**2 + c**2)
        if norm < 1e-12:
            raise ValueError("quaternion norm is too small to normalize")
        return [w/norm, a/norm, b/norm, c/norm]
class Posture_control:
    def __init__(self):
        self.quaternion_tools = Quaternion_tools() #创建四元数工具实例
    #根据四元数所表示的旋转和平移向量，将目前的身体端点坐标转换为新的身体末端点坐标
    #因为四元数记录的目标坐标系对于是开机时的初始坐标系的变化，所以需要用一个静态数组来记录初始的身体末端点坐标
    #之后每次根据四元数和平移向量来更新这个数组中的坐标值 

    #低0步，在输入目标姿态前，先将目前的身体末端点坐标记录下来，作为初始坐标系

    def update_endpoint(self, quaternion, translation, body_endpoint, leg_endpoint):
        
        w, a, b, c = quaternion #用w，a，b，c表示四元数的四个分量
        t_x, t_y, t_z = translation #用t_x，t_y，t_z表示平移向量的三个分量

        #根据四元数和平移向量更新身体末端点坐标
        #减去平移向量，再进行旋转变换，最后再加上平移向量
        for i in range(6):
            body_endpoint[i] = [
                body_endpoint[i][0] - t_x,
                body_endpoint[i][1] - t_y,
                body_endpoint[i][2] - t_z
            ]
            leg_endpoint[i] = [
                leg_endpoint[i][0] - t_x,
                leg_endpoint[i][1] - t_y,
                leg_endpoint[i][2] - t_z
            ]

        #先更新身体末端点坐标，再更新腿端点坐标
        for i in range(6):
            x, y, z = body_endpoint[i]
            x_new = (1 - 2*b**2 - 2*c**2)*x + (2*a*b - 2*c*w)*y + (2*a*c + 2*b*w)*z
            y_new = (2*a*b + 2*c*w)*x + (1 - 2*a**2 - 2*c**2)*y + (2*b*c - 2*a*w)*z
            z_new = (2*a*c - 2*b*w)*x + (2*b*c + 2*a*w)*y + (1 - 2*a**2 - 2*b**2)*z
            body_endpoint[i] = [x_new, y_new, z_new]

        #更新腿端点坐标
        for i in range(6):
            x, y, z = leg_endpoint[i]
            x_new = (1 - 2*b**2 - 2*c**2)*x + (2*a*b - 2*c*w)*y + (2*a*c + 2*b*w)*z
            y_new = (2*a*b + 2*c*w)*x + (1 - 2*a**2 - 2*c**2)*y + (2*b*c - 2*a*w)*z
            z_new = (2*a*c - 2*b*w)*x + (2*b*c + 2*a*w)*y + (1 - 2*a**2 - 2*b**2)*z
            leg_endpoint[i] = [x_new, y_new, z_new]

        #最后再加上平移向量
        for i in range(6):
            body_endpoint[i] = [
                body_endpoint[i][0] + t_x,
                body_endpoint[i][1] + t_y,
                body_endpoint[i][2] + t_z
            ]
            leg_endpoint[i] = [
                leg_endpoint[i][0] + t_x,
                leg_endpoint[i][1] + t_y,
                leg_endpoint[i][2] + t_z
            ]

        return body_endpoint, leg_endpoint

    #单腿逆运动学 对第二、第三段腿，约定向上转为正，向下转为负，即抬腿为正，落腿为负
    def single_leg_IK(self,leg_vector_body,leg_length,mount_angle):
        #足端相对身体末端点坐标系的坐标值
        #用x，y，z表示足端相对身体末端点坐标系的坐标值
        x, y, z = leg_vector_body
        angle = [0, 0, 0] #用angle表示三个角度值
        #angle1 = atan2(y,x)

        angle[0] = atan2(y, x) - mount_angle #angle1表示第一段腿的旋转角度，绕z轴旋转
        if angle[0] < -math.pi:
            angle[0] = -math.pi
        elif angle[0] > math.pi:
            angle[0] = math.pi
        #水平方向模长： r = sqrt(x**2 + y**2) ; 真正进入结算的距离 u = r - l1 ; 其中l1是第一段腿的长度
        #两点距离： d = sqrt(u**2 + w**2) ; 其中w = -z
        r = sqrt(x**2 + y**2)
        u = r - leg_length[0]
        w = -z
        d = sqrt(u**2 + w**2)
        if d < 1e-6: #如果距离过小，可能会导致计算误差，直接返回初始角度
            d = 1e-6
        #需要判断d是否满足逆运动学的可达性条件： d <= l2 + l3 and d >= abs(l2 - l3) ; 其中l2和l3分别是第二段腿和第三段腿的长度
        #如果满足可达性条件，则可以计算出angle2和angle3的值

        cosd = (leg_length[1]**2 + leg_length[2]**2 - d**2) / (2 * leg_length[1] * leg_length[2])
        cosd = max(min(cosd, 1.0), -1.0) #将cosd限制在[-1, 1]范围内，避免由于计算误差导致的acos函数输入值超出范围
        angle[2] = -(math.pi - math.acos(cosd)) #angle3表示第三段腿的旋转角度
        if angle[2] < -math.pi:
            angle[2] = -math.pi
        elif angle[2] > math.pi:
            angle[2] = math.pi


        #α表示向量d与水平面的夹角，β表示l3、l2、d构成的三角形中，l2与d的夹角
        alpha = atan2(w, u)
        cos3 = (leg_length[1]**2 + d**2 - leg_length[2]**2) / (2 * leg_length[1] * d)
        cos3 = max(min(cos3, 1.0), -1.0) #将cos3限制在[-1, 1]范围内，避免由于计算误差导致的acos函数输入值超出范围
        beta = math.acos(cos3)
        angle[1] = beta - alpha #angle2表示第二段腿的旋转
        if angle[1] < -math.pi:
            angle[1] = -math.pi
        elif angle[1] > math.pi:
            angle[1] = math.pi

        return angle
    
    #单腿正运动学
    def single_leg_FK(self,angle,leg_length,mount_angle,body_endpoint,body_quaternion):
        #angle表示三个角度值，leg_length表示三段腿的长度，mount_angle表示第一段腿的安装角度，body_endpoint表示身体末端点坐标
        #计算出足端相对身体末端点坐标系的坐标值

        #body层
        theta = angle[0] + mount_angle #第一段腿的旋转角度加上安装角度
        ux = math.cos(theta)
        uy = math.sin(theta)
        #第一关节末端点坐标
        p1 = [
            leg_length[0] * ux,
            leg_length[0] * uy,
            0.0
        ]

        #第二关节末端点坐标
        p2 = [
            p1[0] + leg_length[1] * math.cos(angle[1]) * ux,
            p1[1] + leg_length[1] * math.cos(angle[1]) * uy,
            p1[2] + leg_length[1] * math.sin(angle[1])
        ]

        #第三关节末端点坐标，也就是足端坐标
        p3 = [
            p2[0] + leg_length[2] * math.cos(angle[2]+angle[1]) * ux,
            p2[1] + leg_length[2] * math.cos(angle[2]+angle[1]) * uy,
            p2[2] + leg_length[2] * math.sin(angle[2]+angle[1])
        ]

        #world层
        #将足端坐标转换到世界坐标系下
        p1_rot = self.quaternion_tools.rotate_vector_by_quaternion(p1, body_quaternion) #将第一关节末端点坐标旋转到世界坐标系下
        p2_rot = self.quaternion_tools.rotate_vector_by_quaternion(p2, body_quaternion) #将第二关节末端点坐标旋转到世界坐标系下
        p3_rot = self.quaternion_tools.rotate_vector_by_quaternion(p3, body_quaternion) #将第三关节末端点坐标旋转到世界坐标系下
        p1_world = [
            p1_rot[0] + body_endpoint[0],
            p1_rot[1] + body_endpoint[1],
            p1_rot[2] + body_endpoint[2]
        ]
        p2_world = [
            p2_rot[0] + body_endpoint[0],
            p2_rot[1] + body_endpoint[1],
            p2_rot[2] + body_endpoint[2]
        ]
        p3_world = [
            p3_rot[0] + body_endpoint[0],
            p3_rot[1] + body_endpoint[1],
            p3_rot[2] + body_endpoint[2]
        ]

        return [p1_world, p2_world, p3_world]

       

    #第一步：中心坐标系转换到身体末端点坐标系
    def center_to_body_endpoint(self,target_quaternion,translation,body_endpoint,target_body_endpoint):
        q_w, q_a, q_b, q_c = target_quaternion #用w，a，b，c表示四元数的四个分量
        t_x, t_y, t_z = translation #用t_x，t_y，t_z表示平移向量的三个分量
        
        # 计算身体末端点坐标
        #遇到平移先减去平移向量，再进行旋转变换
        for i in range(6):
            target_body_endpoint[i] = [
                body_endpoint[i][0] - t_x,
                body_endpoint[i][1] - t_y,
                body_endpoint[i][2] - t_z
            ]
        # 旋转变换使用四元数进行计算
        #公式：x_new = (1 - 2*q_b**2 - 2*q_c**2)*x + (2*q_a*q_b - 2*q_c*q_w)*y + (2*q_a*q_c + 2*q_b*q_w)*z
        #      y_new = (2*q_a*q_b + 2*q_c*q_w)*x + (1 - 2*q_a**2 - 2*q_c**2)*y + (2*q_b*q_c - 2*q_a*q_w)*z
        #      z_new = (2*q_a*q_c - 2*q_b*q_w)*x + (2*q_b*q_c + 2*q_a*q_w)*y + (1 - 2*q_a**2 - 2*q_b**2)*z
        for i in range(6):
            x, y, z = target_body_endpoint[i]
            x_new = (1 - 2*q_b**2 - 2*q_c**2)*x + (2*q_a*q_b - 2*q_c*q_w)*y + (2*q_a*q_c + 2*q_b*q_w)*z
            y_new = (2*q_a*q_b + 2*q_c*q_w)*x + (1 - 2*q_a**2 - 2*q_c**2)*y + (2*q_b*q_c - 2*q_a*q_w)*z
            z_new = (2*q_a*q_c - 2*q_b*q_w)*x + (2*q_b*q_c + 2*q_a*q_w)*y + (1 - 2*q_a**2 - 2*q_b**2)*z
            target_body_endpoint[i] = [x_new, y_new, z_new]
        for i in range(6):
            target_body_endpoint[i] = [
                target_body_endpoint[i][0] + t_x,
                target_body_endpoint[i][1] + t_y,
                target_body_endpoint[i][2] + t_z
            ]
        return target_body_endpoint
    
    #第二步： 根据腿长和“ 身体末端点 ”、“ 腿部端点 ”两个坐标点之间的距离计算出腿的目标角度
    def body_to_leg(self,target_body_endpoint,leg_length,leg_endpoint,mount_angle,target_quaternion):#目前默认不平移，只进行旋转变换
        # 更具现在腿端点坐标和目标身体末端点坐标，运用逆运动学算法计算出每条腿的目标角度
        #执行平移命令，打算按单位距离进行平移，也就是设置一个平移步长，输入的是n个平移步长，暂时不做高级的平移控制算法
        #先省略具体实现
        #…………………………………………
        #pass
        #…………………………………………
        
        # 逆运动学算法计算出每条腿的目标角度
        angle_arr = [ [0, 0, 0]for _ in range(6) ]#二维数组表示目标角度数据

        q_inv = self.quaternion_tools.quat_conjugate(target_quaternion) #计算目标姿态四元数的共轭，用于将世界坐标系下的向量转换到身体坐标系下

        for i in range(6):
            #计算出，在世界坐标系下，腿端点坐标和目标身体末端点坐标之间的向量差，也就是腿端点相对于身体末端点的向量
            v_world = [
                leg_endpoint[i][0] - target_body_endpoint[i][0],
                leg_endpoint[i][1] - target_body_endpoint[i][1],
                leg_endpoint[i][2] - target_body_endpoint[i][2]
            ]

            #将这个向量从世界坐标系转换到身体坐标系下，得到腿端点相对于身体末端点的向量在身体坐标系下的表示
            v_body = self.quaternion_tools.rotate_vector_by_quaternion(v_world, q_inv) #将世界坐标系下的向量转换到身体坐标系下

            angle_arr[i] = self.single_leg_IK(v_body, leg_length, mount_angle[i]) #计算单腿逆运动学，得到每条腿的目标角度

        return angle_arr
        
class test:
    def __init__(self):
        self.quaternion_tools = Quaternion_tools() #创建四元数工具实例
    def test_function(self):
        
        leg_length = [10, 10, 10] #三段腿的长度
        #假设每条腿初始状态都为伸直的状态
        #六条腿编号
        # [
        #     1 , 2
        #     3 , 4
        #     5 , 6
        # ]
        #假设宽为4 ，长为6，则身体末端点坐标可以设置为以下六个点的坐标
        #坐标系为x前，y左，z上
        body_endpoint = [
            [3, 2, 0],
            [3, -2, 0],
            [0, 2, 0],
            [0, -2, 0],
            [-3, 2, 0],
            [-3, -2, 0]

        ]
        cost = math.cos(math.pi/3) #腿1 的第一段与x轴夹角为60°，前后左右对称，中间两条为垂直x轴,
        sint = math.sin(math.pi/3)
        #第三段腿的末端点坐标可以设置为以下六个点的坐标，假设每条腿初始状态都为伸直的状态
        leg_endpoint = [
            [30*cost + 3, 30*sint + 2, 0],
            [30*cost + 3, -30*sint - 2, 0],
            [0, 30 + 2, 0],
            [0, -30 - 2, 0],
            [-30*cost - 3, 30*sint + 2, 0],
            [-30*cost - 3, -30*sint - 2, 0]
        ]

        mount_angle =[
            math.atan2(leg_endpoint[i][1] - body_endpoint[i][1], leg_endpoint[i][0] - body_endpoint[i][0]) for i in range(6)
        ]
        hexapode = robot_model.Hexapode_model(leg_length, body_endpoint, leg_endpoint,mount_angle)
        posture_control = Posture_control()

        #每条腿的初始状态设置为0,30,-120
        init_angle_arr = [0, math.pi/6, -2*math.pi/3] 

        per_leg_point = [[0, 0, 0] for _ in range(18)] #每条腿的三个关节末端点坐标，暂时设置为0，后续根据正运动学计算出具体坐标值

        for i in range(6):
            per_leg_point[3*i],per_leg_point[3*i+1],per_leg_point[3*i+2] = posture_control.single_leg_FK(init_angle_arr, leg_length, mount_angle[i], body_endpoint[i], [1, 0, 0, 0]) #初始姿态四元数设置为单位四元数，表示没有旋转
            leg_endpoint[i] = [per_leg_point[3*i+2][0], per_leg_point[3*i+2][1], per_leg_point[3*i+2][2]]

        hexapode.per_leg_point = per_leg_point

        #……………………………………

        #IMU四元数设置为绕x=y,z=0的轴旋转10°，目标姿态设为水平状态，测试姿态控制函数的功能
        IMU_quaternion = [math.cos(math.pi/36), math.sin(math.pi/36)/math.sqrt(2), math.sin(math.pi/36)/math.sqrt(2), 0] #绕x=y,z=0的轴旋转10°的四元数表示
        target_quaternion = [1, 0, 0, 0] #水平状态的四元数表示
        #将两个四元数归一化，确保四元数的模长为1，避免由于计算误差导致的四元数失效
        IMU_quaternion = self.quaternion_tools.quat_normalize(IMU_quaternion)
        target_quaternion = self.quaternion_tools.quat_normalize(target_quaternion)

        body_endpoint, leg_endpoint, per_leg_point = self.test_posture_control(posture_control, body_endpoint, leg_endpoint, leg_length, mount_angle, IMU_quaternion, per_leg_point, target_quaternion, [0, 0, 0]) #假设地面绕x=y,z=0的轴旋转10°，目标姿态设为水平

        IMU_quaternion = [1, 0, 0, 0] #地面状态的四元数表示，假设姿态回到水平状态
        target_quaternion = [math.cos(math.pi/36), math.sin(math.pi/36)/math.sqrt(2), math.sin(math.pi/36)/math.sqrt(2), 0] #目标姿态设为绕x=y,z=0的轴旋转10°

        #将两个四元数归一化，确保四元数的模长为1，避免由于计算误差导致的四元数失效
        IMU_quaternion = self.quaternion_tools.quat_normalize(IMU_quaternion)
        target_quaternion = self.quaternion_tools.quat_normalize(target_quaternion)

        body_endpoint, leg_endpoint, per_leg_point = self.test_posture_control(posture_control, body_endpoint, leg_endpoint, leg_length, mount_angle, IMU_quaternion, per_leg_point, target_quaternion, [0, 0, 0]) #假设姿态回到水平状态，IMU四元数设为单位四元数，目标姿态也设为绕x=y,z=0的轴旋转10°

    def test_posture_control(self,posture_control, body_endpoint, leg_endpoint, leg_length, mount_angle,IMU_quaternion,per_leg_point,target_quaternion,translation=[0, 0, 0]):
        #现在假定给两个温和的四元数，一个表示地面情况，一个表示目标姿态，平移向量暂时设置为0，之后再根据实际情况进行调整
        #假设地面绕x=y,z=0的轴旋转10°，目标姿态设为水平

        
        q_w, q_a, q_b, q_c = IMU_quaternion
        c_IMU_quaternion = [q_w, -q_a, -q_b, -q_c] #地面四元数的共轭，用于计算误差四元数

        #目标姿态总是相对初始状态，也就是机身水平时的状态

        error_quaternion = [
            target_quaternion[0]*c_IMU_quaternion[0] - target_quaternion[1]*c_IMU_quaternion[1] - target_quaternion[2]*c_IMU_quaternion[2] - target_quaternion[3]*c_IMU_quaternion[3],
            target_quaternion[0]*c_IMU_quaternion[1] + target_quaternion[1]*c_IMU_quaternion[0] + target_quaternion[2]*c_IMU_quaternion[3] - target_quaternion[3]*c_IMU_quaternion[2],
            target_quaternion[0]*c_IMU_quaternion[2] - target_quaternion[1]*c_IMU_quaternion[3] + target_quaternion[2]*c_IMU_quaternion[0] + target_quaternion[3]*c_IMU_quaternion[1],
            target_quaternion[0]*c_IMU_quaternion[3] + target_quaternion[1]*c_IMU_quaternion[2] - target_quaternion[2]*c_IMU_quaternion[1] + target_quaternion[3]*c_IMU_quaternion[0]
        ]

        body_endpoint, leg_endpoint = posture_control.update_endpoint(IMU_quaternion, translation, body_endpoint, leg_endpoint)

        target_body_endpoint = [[0, 0, 0] for _ in range(6)] #目标身体末端点坐标，初始设置为0，后续根据中心坐标系转换函数计算出具体坐标值
        
        target_body_endpoint = posture_control.center_to_body_endpoint(error_quaternion, translation, body_endpoint, target_body_endpoint)
        #姑且认为身体末端点坐标已经更新为目标坐标，之后再根据实际情况进行调整
        #深拷贝
        body_endpoint = [p[:] for p in target_body_endpoint] 
        

        angle_arr = [ [0, 0, 0]for _ in range(6) ]#二维数组表示目标角度数据
        angle_arr = posture_control.body_to_leg(target_body_endpoint, leg_length, leg_endpoint, mount_angle, target_quaternion)
        for i in range(6):
            per_leg_point[3*i],per_leg_point[3*i+1],per_leg_point[3*i+2] = posture_control.single_leg_FK(angle_arr[i], leg_length, mount_angle[i], target_body_endpoint[i], target_quaternion)

        for i in range(6):
            fk_foot = per_leg_point[3*i+2] #每条腿的足端坐标
            target_foot = leg_endpoint[i] #每条腿的目标足端坐标

            err = [
                fk_foot[0] - target_foot[0],
                fk_foot[1] - target_foot[1],
                fk_foot[2] - target_foot[2]
            ]
        
            err_nom = math.sqrt(err[0]**2 + err[1]**2 + err[2]**2)

            print(f"Leg {i+1} target foot = {target_foot}")
            print(f"Leg {i+1} fk foot     = {fk_foot}")
            print(f"Leg {i+1} error       = {err}")
            print(f"Leg {i+1} error norm  = {err_nom}")
            print("-" * 40)

        for i in range(6):
            leg_endpoint[i] = per_leg_point[3*i+2][:] #更新腿端点坐标为新的足端坐标

        for i in range(6):
            print(f"Leg {i+1} target angles: {angle_arr[i]}")

        for i in range(18):
            print(f"Leg {i//3 + 1} joint {i%3 + 1} endpoint: {per_leg_point[i]}")

        return body_endpoint, leg_endpoint, per_leg_point


class run_posture_control:
    def __init__(self):
        self.quaternion_tools = Quaternion_tools() #创建四元数工具实例
        self.posture_control = Posture_control() #创建姿态控制实例
        leg_length = [10, 10, 10] #三段腿的长度
        #假设每条腿初始状态都为伸直的状态
        #六条腿编号
        # [
        #     1 , 2
        #     3 , 4
        #     5 , 6
        # ]
        #假设宽为4 ，长为6，则身体末端点坐标可以设置为以下六个点的坐标
        #坐标系为x前，y左，z上
        body_endpoint = [
            [3, 2, 0],
            [3, -2, 0],
            [0, 2, 0],
            [0, -2, 0],
            [-3, 2, 0],
            [-3, -2, 0]
        ]
        cost = math.cos(math.pi/3) #腿1 的第一段与x轴夹角为60°，前后左右对称，中间两条为垂直x轴,
        sint = math.sin(math.pi/3)
        #第三段腿的末端点坐标可以设置为以下六个点的坐标，假设每条腿初始状态都为伸直的状态
        leg_endpoint = [
            [30*cost + 3, 30*sint + 2, 0],
            [30*cost + 3, -30*sint - 2, 0],
            [0, 30 + 2, 0],
            [0, -30 - 2, 0],
            [-30*cost - 3, 30*sint + 2, 0],
            [-30*cost - 3, -30*sint - 2, 0]
        ]
        mount_angle =[
            math.atan2(leg_endpoint[i][1] - body_endpoint[i][1], leg_endpoint[i][0] - body_endpoint[i][0]) for i in range(6)
        ]
        self.initial_body_endpoint = [p[:] for p in body_endpoint]
        self.initial_leg_endpoint = [p[:] for p in leg_endpoint]
        self.hexapode = robot_model.Hexapode_model(
            leg_length,
            [p[:] for p in self.initial_body_endpoint],
            [p[:] for p in self.initial_leg_endpoint],
            mount_angle,
        )
        self.q_now = [1, 0, 0, 0] #初始姿态四元数设置为单位四元数
        self.q_prev = [1, 0, 0, 0] #上一次姿态四元数设置为单位四元数
    def post_init(self):
        self.hexapode.body_endpoint = [p[:] for p in self.initial_body_endpoint]
        self.hexapode.leg_endpoint = [p[:] for p in self.initial_leg_endpoint]

        #每条腿的初始状态设置为0,30,-120
        init_angle_arr = [0, math.pi/6, -2*math.pi/3] 

        per_leg_point = [[0, 0, 0] for _ in range(18)] #每条腿的三个关节末端点坐标，暂时设置为0，后续根据正运动学计算出具体坐标值

        for i in range(6):
            per_leg_point[3*i],per_leg_point[3*i+1],per_leg_point[3*i+2] = self.posture_control.single_leg_FK(init_angle_arr, self.hexapode.leg_length, self.hexapode.mount_angle[i], self.hexapode.body_endpoint[i], [1, 0, 0, 0]) #初始姿态四元数设置为单位四元数，表示没有旋转
            self.hexapode.leg_endpoint[i] = [per_leg_point[3*i+2][0], per_leg_point[3*i+2][1], per_leg_point[3*i+2][2]]

        self.hexapode.per_leg_point = per_leg_point
        return self.hexapode
    
    def run(self, IMU_quaternion, target_quaternion, translation):
        q_w, q_a, q_b, q_c = IMU_quaternion
        c_IMU_quaternion = [q_w, -q_a, -q_b, -q_c] #地面四元数的共轭，用于计算误差四元数

        error_quaternion = [
            target_quaternion[0]*c_IMU_quaternion[0] - target_quaternion[1]*c_IMU_quaternion[1] - target_quaternion[2]*c_IMU_quaternion[2] - target_quaternion[3]*c_IMU_quaternion[3],
            target_quaternion[0]*c_IMU_quaternion[1] + target_quaternion[1]*c_IMU_quaternion[0] + target_quaternion[2]*c_IMU_quaternion[3] - target_quaternion[3]*c_IMU_quaternion[2],
            target_quaternion[0]*c_IMU_quaternion[2] - target_quaternion[1]*c_IMU_quaternion[3] + target_quaternion[2]*c_IMU_quaternion[0] + target_quaternion[3]*c_IMU_quaternion[1],
            target_quaternion[0]*c_IMU_quaternion[3] + target_quaternion[1]*c_IMU_quaternion[2] - target_quaternion[2]*c_IMU_quaternion[1] + target_quaternion[3]*c_IMU_quaternion[0]
        ]

        #计算旋转增量
        #深拷贝
        self.q_now = IMU_quaternion[:]
        self.q_prev = self.quaternion_tools.quat_conjugate(self.q_prev) #计算上一次姿态四元数的共轭
        q_delta = [
            self.q_now[0]*self.q_prev[0] - self.q_now[1]*self.q_prev[1] - self.q_now[2]*self.q_prev[2] - self.q_now[3]*self.q_prev[3],
            self.q_now[0]*self.q_prev[1] + self.q_now[1]*self.q_prev[0] + self.q_now[2]*self.q_prev[3] - self.q_now[3]*self.q_prev[2],
            self.q_now[0]*self.q_prev[2] - self.q_now[1]*self.q_prev[3] + self.q_now[2]*self.q_prev[0] + self.q_now[3]*self.q_prev[1],
            self.q_now[0]*self.q_prev[3] + self.q_now[1]*self.q_prev[2] - self.q_now[2]*self.q_prev[1] + self.q_now[3]*self.q_prev[0]
        ]
        self.q_prev = self.q_now[:] # 更新上一次姿态四元数

        self.hexapode.body_endpoint, self.hexapode.leg_endpoint = self.posture_control.update_endpoint(q_delta, translation, self.hexapode.body_endpoint, self.hexapode.leg_endpoint)

        target_body_endpoint = self.posture_control.center_to_body_endpoint(error_quaternion, translation, self.hexapode.body_endpoint, [[0, 0, 0] for _ in range(6)])
        self.hexapode.body_endpoint = [p[:] for p in target_body_endpoint] #深拷贝

        angle_arr = self.posture_control.body_to_leg(target_body_endpoint, self.hexapode.leg_length, self.hexapode.leg_endpoint, self.hexapode.mount_angle, target_quaternion)
        for i in range(6):
            self.hexapode.per_leg_point[3*i],self.hexapode.per_leg_point[3*i+1],self.hexapode.per_leg_point[3*i+2] = self.posture_control.single_leg_FK(angle_arr[i], self.hexapode.leg_length, self.hexapode.mount_angle[i], target_body_endpoint[i], target_quaternion)
            self.hexapode.leg_endpoint[i] = self.hexapode.per_leg_point[3*i+2][:] #更新腿端点坐标为新的足端坐标
        
        return self.hexapode, angle_arr

        



        
def main():

    test_instance = test()
    test_instance.test_function()

if __name__ == "__main__":
    main()
    
