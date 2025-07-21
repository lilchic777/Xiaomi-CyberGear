#!/usr/bin/env python3
# encoding: utf-8

import os
import sys
import time
import logging
import can

# 添加pcan_cybergear库的路径
sys.path.append(os.path.join("..", "cybergear"))
from pcan_cybergear import CANMotorController

# 初始化日志系统
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

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
        motor3 = CANMotorController(bus, motor_id=103, main_can_id=254)
        motor4 = CANMotorController(bus, motor_id=104, main_can_id=254)
        motors = [motor1, motor2, motor3, motor4]
        logging.info("电机控制器初始化完成")

        # 配置电机参数（符合CANopen SDO参数配置规范）
        for motor in motors:
            motor.write_param_table("limit_cur", 3)  # 电流限制
            motor.write_param_table("loc_kp", 8)  # 位置环比例增益
            motor.write_param_table("spd_kp", 2)  # 速度环比例增益
            motor.write_param_table("spd_ki", 0.03)  # 速度环积分增益
            motor.write_single_param("limit_spd", value=0.5)  # 最大速度限制
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


def main():
    try:
        # 初始化CAN总线和电机
        bus, motors = init_motors()

        # 位置控制序列：逆时针→顺时针→回零
        logging.info("开始逆时针运动")
        for i, motor in enumerate(motors):
            if i == 2:  # motor3反向
                motor.write_single_param("loc_ref", value=0.2)
            else:
                motor.write_single_param("loc_ref", value=-0.2)
            time.sleep(2)

        logging.info("回到零点位置")
        for motor in motors:
            motor.write_single_param("loc_ref", value=0)  # 回零
            time.sleep(2)

        logging.info("开始顺时针运动")
        for i, motor in enumerate(motors):
            if i == 2:  # motor3反向
                motor.write_single_param("loc_ref", value=-0.2)
            else:
                motor.write_single_param("loc_ref", value=0.2)
            time.sleep(2)

        logging.info("回到零点位置")
        for motor in motors:
            motor.write_single_param("loc_ref", value=0)  # 回零
            time.sleep(2)

        # 关闭电机
        for motor in motors:
            motor.disable()
        logging.info("运动序列完成，电机已停止")

    except Exception as e:
        logging.error(f"程序执行出错: {str(e)}")
    finally:
        # 确保关闭CAN总线连接
        if 'bus' in locals():
            bus.shutdown()
            logging.info("CAN总线已关闭")


if __name__ == "__main__":
    main()