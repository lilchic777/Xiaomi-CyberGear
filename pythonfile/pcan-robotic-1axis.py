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


def main():
    # 初始化日志系统
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

    # 添加自定义库路径
    sys.path.append(os.path.join("..", "cybergear"))

    try:
        # 连接到CAN总线 (1 Mbps)
        bus = can.interface.Bus(
            interface="pcan",
            channel="PCAN_USBBUS1",
            bitrate=1000000
        )
        logging.info("CAN总线连接成功")

        # 创建电机控制器 (使用ID为102的电机)
        motor = CANMotorController(bus, motor_id=101, main_can_id=254)
        logging.info("电机控制器初始化完成")

        # 配置电机参数
        motor.write_param_table("limit_cur", 1.5)  # 电流限制
        motor.write_param_table("loc_kp", 8)  # 位置环比例增益
        motor.write_param_table("spd_kp", 2)  # 速度环比例增益
        motor.write_param_table("spd_ki", 0.03)  # 速度环积分增益

        # 停止电机并设置零点
        motor.disable()
        motor.set_0_pos()
        logging.info("电机已停止并设置零点")

        # 设置为位置控制模式
        motor.set_run_mode(motor.RunModes.POSITION_MODE)
        motor.enable()
        logging.info("进入位置控制模式")

        # 设置最大速度
        motor.write_single_param("limit_spd", value=4)

        # 位置控制
        logging.info("开始逆时针运动")
        motor.write_single_param("loc_ref", value=-2)  # 逆时针方向
        time.sleep(0.5)

        logging.info("回到零点位置")
        motor.write_single_param("loc_ref", value=0)  # 回零
        time.sleep(0.5)

        logging.info("开始顺时针运动")
        motor.write_single_param("loc_ref", value=2)  # 顺时针方向
        time.sleep(0.5)

        logging.info("回到零点位置")
        motor.write_single_param("loc_ref", value=0)  # 回零
        time.sleep(0.5)

        # 关闭电机
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