
import math

class PidTypeDef:
    def __init__(self, mode, PID, max_out, max_iout):
        self.mode = mode
        self.Kp, self.Ki, self.Kd = PID
        self.max_out = max_out
        self.max_iout = max_iout
        self.Dbuf = [0.0, 0.0, 0.0]
        self.error = [0.0, 0.0, 0.0]
        self.Pout = self.Iout = self.Dout = self.out = 0.0
        self.fdb = self.set = 0.0

    def limit_max(self, value, max_value):
        if value > max_value:
            return max_value
        elif value < -max_value:
            return -max_value
        return value

    def PID_Calc(self, ref, set):
        # 更新误差的历史记录
        self.error[2], self.error[1], self.error[0] = self.error[1], self.error[0], set - ref

        if self.mode == 'PID_POSITION':
            self.Pout = self.Kp * self.error[0]
            self.Iout += self.Ki * self.error[0]
            self.Iout = self.limit_max(self.Iout, self.max_iout)
            if math.isnan(self.Iout):  # 检查 Iout 是否为 NaN
                self.Iout = 0
            self.Dbuf[2], self.Dbuf[1], self.Dbuf[0] = self.Dbuf[1], self.Dbuf[0], (self.error[0] - self.error[1])
            self.Dout = self.Kd * self.Dbuf[0]

            self.out = self.Pout + self.Iout + self.Dout
            self.out = self.limit_max(self.out, self.max_out)


        elif self.mode == 'PID_DELTA':  # 如果是增量模式
            # 计算增量模式下的比例项、积分项和微分项
            self.Pout = self.Kp * (self.error[0] - self.error[1])
            self.Iout = self.Ki * self.error[0]
            self.Dbuf[2], self.Dbuf[1], self.Dbuf[0] = self.Dbuf[1], self.Dbuf[0], (self.error[0] - 2.0 * self.error[1] + self.error[2])
            self.Dout = self.Kd * self.Dbuf[0]

            # 增量模式累加输出
            self.out += self.Pout + self.Iout + self.Dout
            self.out = self.limit_max(self.out, self.max_out)

        return self.out

    def PID_clear(self):
        # 将所有误差、微分缓冲和输出值重置为0
        self.error = [0.0, 0.0, 0.0]
        self.Dbuf = [0.0, 0.0, 0.0]
        self.out = self.Pout = self.Iout = self.Dout = 0.0
        # 重置反馈和设定值
        self.fdb = self.set = 0.0


# # 示例用法
# pid = PidTypeDef('PID_POSITION', [0.25, 0, 0.1], 100.0, 50.0)
# pid_output = pid.PID_Calc(0, -0.6)
# print("PID Output:", pid_output)
# pid.PID_clear()

# # 模拟反馈过程，初始反馈为20，目标为25
# feedback = -0.6
# setpoint = 0
# time_points = range(100)
# outputs = []

# for t in time_points:
#     pid_output = pid.PID_Calc(feedback, setpoint)
#     outputs.append(pid_output)
#     # 更新反馈值为当前输出（简单模拟）
#     feedback += pid_output * 0.1  # 假设输出影响反馈值的速度
#     print(feedback)
