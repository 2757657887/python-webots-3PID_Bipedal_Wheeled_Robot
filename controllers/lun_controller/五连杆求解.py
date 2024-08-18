import numpy as np

# 已知角度转换为弧度
theta1_radians = np.deg2rad(100)  # 100度
theta2_radians = np.deg2rad(135)   # 135度

# 提供的边长值
l1_val = 60
l2_val = 50
l3_val = 100
l4_val = 100
l5_val = 50

def get_deg(L1,L2,L3,L4,L5,theta1,theta2):

    C = np.sqrt(L1**2+L2**2-2*L1*L2*np.cos(theta2))
    theta_s_1 = np.arcsin(L2*np.sin(theta2)/C)
    D = np.sqrt(C**2+L5**2-2*C*L5*np.cos(theta1-theta_s_1))

    theta_s_2 = np.arcsin(C*np.sin(theta1-theta_s_1)/D) #左边的
    theta_s_3 = np.arcsin(L5*np.sin(theta1-theta_s_1)/D) #右边的
    theta_s_4 = np.pi-theta2-theta_s_1# 右边的

    theta_s_5 = np.arccos((D**2+L4**2-L3**2)/(2*D*L4)) #左边的
    theta_s_6 = np.arccos((D**2+L3**2-L4**2)/(2*D*L3))#右边的

    theta3 = theta_s_4+theta_s_6+theta_s_3
    theta4 = np.arccos((L3**2+L4**2-D**2)/(2*L4*L3))
    theta5 = theta_s_5+theta_s_2

    return theta3, theta4, theta5

theta3, theta4, theta5 = get_deg(l1_val,l2_val,l3_val,l4_val,l5_val,theta1_radians,theta2_radians)

print(np.rad2deg(theta3))
print(np.rad2deg(theta4))
print(np.rad2deg(theta5))