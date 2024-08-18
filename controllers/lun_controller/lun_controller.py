from controller import Supervisor, Motor, InertialUnit, Keyboard
from PID控制器 import PidTypeDef  # 确保你的 PID 控制器类可用
import numpy as np
# 创建 Supervisor 实例
supervisor = Supervisor()
class BipedalSupervisor(Supervisor):
    def __init__(self):
        super(BipedalSupervisor, self).__init__()
        # 获取仿真步长
        self.timestep = int(supervisor.getBasicTimeStep())

        # 获取电机
        self.motor_R_H_M_lun = supervisor.getDevice("R_H_M_lun")
        self.motor_L_H_M_lun = supervisor.getDevice("L_H_M_lun")
        # 将电机模式设置为速度控制（无限位置）
        self.motor_R_H_M_lun.setPosition(float('inf'))
        self.motor_L_H_M_lun.setPosition(float('inf'))
        # 获取轮子传感器并激活
        self.sensor_R_H_S_lun = supervisor.getDevice("R_H_S_lun")
        self.sensor_L_H_S_lun = supervisor.getDevice("L_H_S_lun")
        self.sensor_R_H_S_lun.enable(self.timestep)
        self.sensor_L_H_S_lun.enable(self.timestep)
        # 初始化轮子的位置变量
        self.previous_position_R = self.sensor_R_H_S_lun.getValue()
        self.previous_position_L = self.sensor_L_H_S_lun.getValue()

        # 获取机体关节
        self.motor_R_H_M_da = supervisor.getDevice("R_H_M_da")
        self.motor_L_H_M_da = supervisor.getDevice("L_H_M_da")
        self.motor_R_F_M_da = supervisor.getDevice("R_F_M_da")
        self.motor_L_F_M_da = supervisor.getDevice("L_F_M_da")
        # 获取 IMU 传感器并启用
        self.imu = supervisor.getDevice('my_imu')
        self.imu.enable(self.timestep)

        # 获取 关节 传感器并启用
        self.motor_R_H_S_da = supervisor.getDevice("R_H_S_da")
        self.motor_L_H_S_da = supervisor.getDevice("L_H_S_da")
        self.motor_R_F_S_da = supervisor.getDevice("R_F_S_da")
        self.motor_L_F_S_da = supervisor.getDevice("L_F_S_da")
        self.motor_R_H_S_da.enable(self.timestep)
        self.motor_L_H_S_da.enable(self.timestep)
        self.motor_R_F_S_da.enable(self.timestep)
        self.motor_L_F_S_da.enable(self.timestep)

        # 初始化键盘监听
        self.keyboard = Keyboard()
        self.keyboard.enable(self.timestep)
        
        # 初始化变量
        self.diff_vel = 15 #左右转速度
        self.balance_y = 0.11104 #机体的中心相对于轮子的纵向距离
        self.set_y_right = 0
        self.set_y_left = 0
        # 初始化 PID 控制器，调整这些参数以适应你的机体
        self.chassis_Balance_pid = PidTypeDef('PID_POSITION', [600, 0.1, 40], 100.0, 100.0)
        self.chassis_roll_pid = PidTypeDef('PID_POSITION', [0.9,0, 0.9], 100.0, 100.0)
        self.chassis_speed_pid = PidTypeDef('PID_POSITION', [0.015, 0, 0.01], 100.0, 100.0)

    def keyboard_control(self):
    # 监听键盘输入
        key = self.keyboard.getKey()
        if key == ord('W'):
            # 向前移动，设置向前值为1，向后值为0
            self.velocity_forward = 1
            self.velocity_backward = 0
        elif key == ord('S'):
            # 向后移动，设置向前值为0，向后值为1
            self.velocity_forward = 0
            self.velocity_backward = 1
        elif key == ord('A'):
            # 向左转，设置向左值为1，向右值为0
            self.velocity_turnleft = 1
            self.velocity_turnright = 0
        elif key == ord('D'):
            # 向右转，设置向左值为0，向右值为1
            self.velocity_turnleft = 0
            self.velocity_turnright = 1
        else:
            self.velocity_forward = 0
            self.velocity_backward = 0
            self.velocity_turnleft = 0
            self.velocity_turnright = 0


    def legs_Inversekinematics(self, x, y, len_chassis, len_leg1, len_leg2):
        x += len_chassis / 2

        len_behind2end = np.sqrt(x**2 + y**2)
        len_front2end = np.sqrt((len_chassis - x)**2 + y**2)

        def slides2angle(nei_slide1, nei_slide2, opp_slide):
            angle_mem = nei_slide1**2 + nei_slide2**2 - opp_slide**2
            angle_den = 2 * nei_slide1 * nei_slide2
            return np.arccos(angle_mem / angle_den)

        behind_theta_1 = slides2angle(len_leg1, len_behind2end, len_leg2)
        behind_theta_2 = slides2angle(len_chassis, len_behind2end, len_front2end)
        front_theta_1 = slides2angle(len_leg1, len_front2end, len_leg2)
        front_theta_2 = slides2angle(len_chassis, len_front2end, len_behind2end)

        front_theta_sum = front_theta_1 + front_theta_2
        behind_theta_sum = behind_theta_1 + behind_theta_2
        print(x)
        return (np.pi - front_theta_sum).item(), (np.pi - behind_theta_sum).item()
    
    # 计算两个轮子的速度，并返回平均速度
    def calculate_velocity(self):
        # 当前位置
        current_position_R = self.sensor_R_H_S_lun.getValue()
        current_position_L = self.sensor_L_H_S_lun.getValue()
        print('上次左轮的位置:',self.previous_position_L)
        print("左轮位置：",current_position_L)
        # 计算角速度
        angular_velocity_R = (current_position_R - self.previous_position_R) / (self.timestep / 1000.0)
        angular_velocity_L = (current_position_L - self.previous_position_L) / (self.timestep / 1000.0)

        # 更新上一次位置
        self.previous_position_R = current_position_R
        self.previous_position_L = current_position_L

        
        return round((angular_velocity_R+angular_velocity_L)/2,5)-1
    
    def tasks_control(self):
        '''
        初始化pitch方向平衡控制器
        '''
        # 获取 IMU 的 pitch 值作为机体当前的倾斜状态
        orientation = self.imu.getRollPitchYaw()
        pitch = orientation[0]  # 机体当前的 pitch 值
        roll = orientation[1]  # 机体当前的 pitch 值
        print('roll值为：',roll)
        Balance_pid_out =  self.chassis_Balance_pid.PID_Calc(0, pitch)# 目标 pitch 为 0
        print(f"Current Pitch: {pitch:.5f}, Speed Adjustment: {Balance_pid_out:.2f}")
        # 动力轮平衡环+转速环
        self.motor_L_H_M_lun.setVelocity(Balance_pid_out + self.diff_vel*self.velocity_turnleft  - self.diff_vel*self.velocity_turnright)
        self.motor_R_H_M_lun.setVelocity(Balance_pid_out - self.diff_vel*self.velocity_turnleft  + self.diff_vel*self.velocity_turnright)
        
        # roll轴平衡控制
        set_v = -2 * self.velocity_forward + 2 * self.velocity_backward
        avg_v = self.calculate_velocity()/10

        print("轮子平均速度为:",avg_v)
        print("set_v:",set_v)
        speed_pid_out = self.chassis_speed_pid.PID_Calc(set_v, avg_v)# 目标 为 set_v, 输入为轮子的传感器角速度平均值
        
        print("speed PID输出为:",speed_pid_out)
        set_x = -speed_pid_out
        roll_pid_out = self.chassis_roll_pid.PID_Calc(0, roll)# 目标 roll 为 0
        print("set_x输出为:",set_x)
        print("roll PID输出为:",roll_pid_out)
        if roll_pid_out >=0:
            self.set_y_right = self.balance_y + roll_pid_out
            self.set_y_left = self.balance_y - roll_pid_out
        else:
            self.set_y_right = self.balance_y + roll_pid_out
            self.set_y_left = self.balance_y - roll_pid_out


        # set_y_left = self.set_y
        # set_y_right = self.set_y
        # 腿长限幅（防止干涉和解算极限）
        if (self.set_y_right<0.07):
            self.set_y_right = 0.07
        if (self.set_y_right>0.14):
            self.set_y_right = 0.14

        if (self.set_y_left<0.07):
            self.set_y_left = 0.07
        if (self.set_y_left>0.14):
            self.set_y_left = 0.14

        R_F_leg,R_H_leg  = self.legs_Inversekinematics(set_x, self.set_y_right, 0.06, 0.05, 0.1)
        L_F_leg,L_H_leg = self.legs_Inversekinematics(set_x, self.set_y_left, 0.06, 0.05, 0.1)
        print("右前腿角度",R_F_leg)
        Robot.motor_R_F_M_da.setPosition(np.deg2rad(45)-R_F_leg)
        Robot.motor_R_H_M_da.setPosition(R_H_leg-np.deg2rad(45))
        Robot.motor_L_F_M_da.setPosition(np.deg2rad(45)-L_F_leg)
        Robot.motor_L_H_M_da.setPosition(L_H_leg-np.deg2rad(45))



Robot = BipedalSupervisor()
# 主循环
while supervisor.step(Robot.timestep) != -1:
    # # 获取 IMU 的 pitch 值作为机体当前的倾斜状态
    # orientation = Robot.imu.getRollPitchYaw()
    # current_pitch = orientation[0]  # 机体当前的 pitch 值

    # # 使用 PID 控制器计算电机的速度调整
    # speed_adjustment = Robot.chassis_Balance_pid.PID_Calc(0, current_pitch)  # 目标 pitch 为 0
    # print(f"Current Pitch: {current_pitch:.2f}, Speed Adjustment: {speed_adjustment:.2f}")

    # K_num = 229  # 机体四个电机对应倾斜角的平衡线性值
    # B_num = 0.005 # 偏移量，调节微小的偏移

    # Robot.motor_R_H_M_da.setPosition(-current_pitch*K_num+B_num)
    # Robot.motor_L_H_M_da.setPosition(-current_pitch*K_num+B_num)
    # Robot.motor_R_F_M_da.setPosition(-current_pitch*K_num+B_num)
    # Robot.motor_L_F_M_da.setPosition(-current_pitch*K_num+B_num)
    # Robot.keyboard_control()
    
    Robot.keyboard_control()
    Robot.tasks_control()
    # print(Robot.chassis_speed_pid.PID_Calc(0,-2))
    
