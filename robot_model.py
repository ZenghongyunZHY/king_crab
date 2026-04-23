class Hexapode_model:
    def __init__(self, leg_length, body_endpoint, leg_endpoint, mount_angle):
        self.leg_length = leg_length #腿长
        self.body_endpoint = body_endpoint #身体末端点坐标
        self.leg_endpoint = leg_endpoint #腿末端点坐标
        self.per_leg_point = [[0, 0, 0] for _ in range(18)] #每条腿的三个关节末端点坐标，暂时设置为0，后续根据正运动学计算出具体坐标值
        self.mount_angle = mount_angle #第一关节安装角度
        


    