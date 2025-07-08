#!/usr/bin/env python3
# encoding:utf-8
# 4自由度机械臂逆运动学：给定相应的坐标（X,Y,Z），以及俯仰角，计算出每个关节转动的角度
# 可在俯仰角范围内求解，答案可选接近区间端点或区间内参考数据，无解则返回False

import logging
import os
import sys
import time
import can
import math
import numpy as np
from inverseKinematics import IK
# import matplotlib.pyplot as plt

# 添加pcan_cybergear库的路径
sys.path.append(os.path.join("..", "cybergear"))
# noinspection PyUnresolvedReferences
from pcan_cybergear import CANMotorController

ik = IK()

def init_motors():
    """
    初始化CAN总线和电机控制器
    返回: (bus对象, 电机控制器列表)
    """
    try:
        # 连接到CAN总线 (1 Mbps)
        bus = can.interface.Bus(
            interface="pcan",
            channel="PCAN_USBBUS1",
            bitrate=1000000
        )
        logging.info("CAN总线连接成功")

        # 创建电机控制器 (使用CANopen协议节点ID)
        motor1 = CANMotorController(bus, motor_id=101, main_can_id=254)
        motor2 = CANMotorController(bus, motor_id=102, main_can_id=254)
        motors = [motor1, motor2]
        logging.info("电机控制器初始化完成")

        # 配置电机参数（符合CANopen SDO参数配置规范）
        for motor in motors:
            motor.write_param_table("limit_cur", 0.5)  # 电流限制
            motor.write_param_table("loc_kp", 8)  # 位置环比例增益
            motor.write_param_table("spd_kp", 2)  # 速度环比例增益
            motor.write_param_table("spd_ki", 0.03)  # 速度环积分增益
            motor.write_single_param("limit_spd", value=4)  # 最大速度限制
            motor.disable()  # 进入SWITCH_ON_DISABLED状态
            motor.set_0_pos()  # 设置机械零点
        logging.info("电机参数配置完成并设置零点")

        # 设置为位置控制模式（符合DSP402运动控制协议）
        for motor in motors:
            motor.set_run_mode(motor.RunModes.POSITION_MODE)
            motor.enable()  # 进入OPERATION_ENABLED状态
        logging.info("位置控制模式已启用")

        return bus, motors

    except Exception as e:
        logging.error(f"电机初始化失败: {str(e)}")
        raise


class ArmIK:
    def __init__(self):
        self.bus, self.motors = init_motors()  # 实例初始化时连接硬件

    def motorsMove(self, angles, movetime=None):
        # 驱动1234号电机转动，如果没有指定运动时间，自动计算最长时间
        time.sleep(0.02)
        if movetime is None:
            movetime = int(max(angles[0:2]))  # 切片，包含angle[0]和[1]

        for i, motor in enumerate(self.motors):
            motor.write_single_param("loc_ref", angles[i])
        time.sleep(movetime)

        return movetime

    def setPitchRange(self, coordinate_data, alpha1, alpha2, da=1):
        # 给定坐标coordinate_data和俯仰角的范围alpha1，alpha2, 自动在范围内寻找到的合适的解
        # 如果无解返回False,否则返回对应舵机角度,俯仰角
        # 坐标单位cm， 以元组形式传入，例如(0, 5, 10)
        # da为俯仰角遍历时每次增加的角度
        x, y, z = coordinate_data

        if (alpha1 < alpha2 and da < 0) or (alpha1 > alpha2 and da > 0):    # 调整步长方向（确保方向与范围一致）
            da = -da  # 反转步长方向
        # 遍历搜索，使用np.arange创建角度序列（左闭右开）
        for alpha in np.arange(alpha1, alpha2 + da, da):
            result = ik.getRotationAngle((x, y, z), alpha)
            if result:
                # 转换为Python原生类型
                theta1 = round(float(np.radians(result['theta1'])), 4)
                theta2 = round(float(np.radians(result['theta2'])), 4)
                angles = [
                    theta1,theta2
                ]
                logging.info('angles: %s', angles)  # 使用%格式符
                logging.info('alpha: %s', alpha)  # 自动类型转换
                return angles, alpha
        return False    # 无解时返回False

    def setPitchRangeMoving(self, coordinate_data, alpha, alpha1, alpha2, movetime=None):
        # 给定坐标coordinate_data和俯仰角alpha,以及俯仰角范围的范围alpha1, alpha2，自动寻找最接近给定俯仰角的解，并转到目标位置
        # 如果无解返回False,否则返回舵机角度、俯仰角、运行时间
        # 坐标单位cm， 以元组形式传入，例如(0, 5, 10)
        # alpha为给定俯仰角
        # alpha1和alpha2为俯仰角的取值范围
        # movetime为舵机转动时间，单位ms, 如果不给出时间，则自动计算
        x, y, z = coordinate_data
        result1 = self.setPitchRange((x, y, z), alpha, alpha1)  # 从alpha出发向两边找
        result2 = self.setPitchRange((x, y, z), alpha, alpha2)
        if result1:    # 在(alpha1,alpha)和(alpha,alpha2)两个区间中找到距离alpha最近的解
            data = result1
            if result2 and abs(result2[1] - alpha) < abs(result1[1] - alpha):
                data = result2
        else:
            if result2:
                data = result2
            else:
                logging.info('找不到合适解')
                return False
        if data:
            angles, alpha = data[0], data[1]
            movetime = self.motorsMove(angles, movetime)
        return angles, alpha, movetime

    # def drawMoveRange2D(self, x_min, x_max, dx, y_min, y_max, dy, z, a_min, a_max, da):
    #     # 测试可到达点, 以2d图形式展现，z固定
    #     #测试可到达点, 以3d图形式展现，如果点过多，3d图会比较难旋转
    #     try:
    #         for y in np.arange(y_min, y_max, dy):
    #             for x in np.arange(x_min, x_max, dx):
    #                 result = self.setPitchRange((x, y, z), a_min, a_max, da)
    #                 if result:
    #                     plt.scatter(x, y, s=np.pi, c='r')
    #
    #         plt.xlabel('X Label')
    #         plt.ylabel('Y Label')
    #
    #         plt.show()
    #     except Exception as e:
    #         print(e)
    #         pass


def main():
    # 初始化日志系统
    logging.basicConfig(
        level=logging.INFO,  # CRITICAL, ERROR, WARNING, INFO, DEBUG
        format='%(asctime)s - %(levelname)s - %(message)s'
    )
    # logger = logging.getLogger(__name__)
    AK = None  # 预先声明变量
    try:
        AK = ArmIK()
        angles, alpha, movetime = AK.setPitchRangeMoving((0, 33.775, 33.2),20, 60,1)
        AK.motorsMove(angles, movetime)

        # 关闭电机
        for motor in AK.motors:
            motor.disable()
        logging.info("运动序列完成，电机已停止")

    except Exception as e:
        logging.error(f"程序执行出错: {str(e)}")
    finally:
        # 确保关闭CAN总线连接
        if AK and hasattr(AK, 'motors'):  # 确保AK已初始化
            for motor in AK.motors:
                motor.disable()
        AK.bus.shutdown()  # 正确关闭总线
        logging.info("CAN总线已关闭")


if __name__ == "__main__":
    main()