from controller import Supervisor


# 创建 Supervisor 实例
supervisor = Supervisor()

# 获取仿真步长（毫秒）
timestep = int(supervisor.getBasicTimeStep())

# 获取电机
left_motor = supervisor.getDevice("L_F_1")

# 设置电机的位置或速度
left_motor.setPosition(float('inf'))  # 无限位置模式，通常用于速度控制
left_motor.setVelocity(5.0)  # 设置电机速度

while supervisor.step(timestep) != -1:
    # 这里可以放置控制逻辑
    pass
