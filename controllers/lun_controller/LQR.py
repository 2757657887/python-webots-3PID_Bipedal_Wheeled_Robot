import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import StateSpace, lsim
from control.matlab import *

# 系统参数
m = 0.5  # 小车的质量，单位kg
l = 0.2  # 悬臂的长度，单位m
g = 9.81 # 重力加速度，单位m/s^2
J = m * l**2 # 转动惯量

# 状态空间表示
A = np.array([[0, 1],
              [m*g*l/J, 0]])
B = np.array([[0],
              [1/J]])
C = np.array([[1, 0],
              [0, 1]])
D = np.array([[0],
              [0]])

# 创建状态空间模型
sys = StateSpace(A, B, C, D)

# 设定Q和R矩阵
Q = np.array([[1000, 0],
              [0, 10]])
R = np.array([[1]])

# 计算LQR控制器增益
K, _, _ = lqr(sys, Q, R)

# 打印LQR增益
print("LQR增益K:", K)

# 初始条件和模拟时间
X0 = np.array([[0.1], [0]])  # 初始倾斜角度0.1弧度，初始角速度0
T = np.linspace(0, 10, 100)  # 从0到10秒，100个时间点

print("Shape of T:", T.shape)  # 检查T的形状

# 模拟系统响应
_, _, xout = lsim(sys, np.zeros(len(T)), T, X0)

# 计算控制输入 u = -Kx
U = -np.dot(K, xout.T).T

print("Shape of U:", U.shape)  # 检查U的形状

# 再次使用计算出的控制输入进行模拟
_, _, xout = lsim(sys, U, T, X0)

# 绘图显示结果
plt.figure()
plt.plot(T, xout)
plt.title('Response of the balanced car with LQR Control')
plt.ylabel('State')
plt.xlabel('Time (sec)')
plt.legend(['Angle (rad)', 'Angular velocity (rad/s)'])
plt.grid(True)
plt.show()
