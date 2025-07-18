#!/usr/bin/env python3
# encoding: utf-8

from flask import Flask, render_template
from flask_socketio import SocketIO
import time
import os
import sys
import logging
import can
import threading

# 添加pcan_cybergear库的路径
sys.path.append(os.path.join("..", "..","cybergear"))
from pcan_cybergear import CANMotorController

# 全局状态
bus = None
motors = []
init_flag = False
lock = threading.Lock()
mode_flag = 0    # 0 未启动，1 位置，2 速度

# 初始化日志系统
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

app = Flask(__name__)
socketio = SocketIO(app)

def init_motors():  # 初始化CAN总线和电机控制器
    global bus, motors, init_flag

    if init_flag:  # 已初始化则跳过
        logging.info("电机已初始化，跳过重复初始化")
        return
    try:
        logging.info("初始化电机测试")
        # 连接到CAN总线 (1 Mbps)
        bus = can.interface.Bus(
            interface="pcan",
            channel="PCAN_USBBUS1",
            bitrate=1000000
        )
        logging.info("CAN总线连接成功")

        # 创建电机控制器
        motor1 = CANMotorController(bus, motor_id=101, main_can_id=254)
        motor2 = CANMotorController(bus, motor_id=102, main_can_id=254)
        motor3 = CANMotorController(bus, motor_id=103, main_can_id=254)
        motor4 = CANMotorController(bus, motor_id=104, main_can_id=254)
        motors = [motor1, motor2, motor3, motor4]
        logging.info("电机控制器初始化完成")

        # 配置电机参数
        for motor in motors:
            motor.write_param_table("limit_cur", 0.5)  # 电流限制
            motor.write_param_table("loc_kp", 8)  # 位置环比例增益
            motor.write_param_table("spd_kp", 2)  # 速度环比例增益
            motor.write_param_table("spd_ki", 0.03)  # 速度环积分增益
            motor.write_single_param("limit_spd", value=2)  # 最大速度限制
            motor.disable()  # 进入SWITCH_ON_DISABLED状态
            motor.set_0_pos()  # 设置机械零点
        logging.info("电机参数配置完成并设置零点")
        init_flag = True

    except Exception as e:
        logging.error(f"电机初始化失败: {str(e)}")
        raise


@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('position')
def handle_position():
    global init_flag, mode_flag
    with lock:
        try:
            # logging.info("位置模式测试")
            init_motors()
            # 设置为位置控制模式
            for motor in motors:
                motor.set_run_mode(motor.RunModes.POSITION_MODE)
                motor.enable()  # 进入OPERATION_ENABLED状态
            mode_flag = 1
            logging.info("位置模式已启用")
        except Exception as e:
            logging.error(f"位置模式启动失败: {str(e)}")
            raise


@socketio.on('speed')
def handle_speed():
    global init_flag, mode_flag
    with lock:
        try:
            init_motors()
            # 设置为位置控制模式
            for motor in motors:
                motor.set_run_mode(motor.RunModes.SPEED_MODE)
                motor.enable()  # 进入OPERATION_ENABLED状态
                motor.write_single_param("spd_ref", value=0)
            mode_flag = 2
            logging.info("速度模式已启用")

        except Exception as e:
            logging.error(f"速度模式启动失败: {str(e)}")
            raise

@socketio.on('forward')
def handle_forward():
    if not init_flag:
        logging.info("请启动电机！")
        return
    with lock:
        logging.info("开始顺时针运动")
        if mode_flag == 1:
            for i, motor in enumerate(motors):
                if i == 2:  # motor3反向
                    motor.write_single_param("loc_ref", value=-0.2)
                else:
                    motor.write_single_param("loc_ref", value=0.2)
            time.sleep(0.5)
        elif mode_flag == 2:
            for i, motor in enumerate(motors):
                if i == 2:  # motor3反向
                    motor.write_single_param("spd_ref", value=-0.2)
                else:
                    motor.write_single_param("spd_ref", value=0.2)

@socketio.on('backward')
def handle_backward():
    if not init_flag:
        logging.info("请启动电机！")
        return
    with lock:
        logging.info("开始逆时针运动")
        if mode_flag == 1:
            for i, motor in enumerate(motors):
                if i == 2:  # motor3反向
                    motor.write_single_param("loc_ref", value=0.2)
                else:
                    motor.write_single_param("loc_ref", value=-0.2)
            time.sleep(0.5)
        elif mode_flag == 2:
            for i, motor in enumerate(motors):
                if i == 2:  # motor3反向
                    motor.write_single_param("spd_ref", value=0.2)
                else:
                    motor.write_single_param("spd_ref", value=-0.2)

@socketio.on('zero')
def handle_zero():
    if not init_flag:
        logging.info("请启动电机！")
        return
    with lock:
        if mode_flag == 1:
            logging.info("回到零点位置")
            for motor in motors:
                motor.write_single_param("loc_ref", value=0)  # 回零
            time.sleep(0.5)
        elif mode_flag == 2:
            logging.info("速度回零")
            for motor in motors:
                motor.write_single_param("spd_ref", value=0)

@socketio.on('stop')
def handle_stop():
    global bus, motors, init_flag, mode_flag
    with lock:
        for motor in motors:
            motor.disable()
        motors.clear()
        logging.info("电机已停止")
        # if 'bus' in locals():
        if bus is not None:
            bus.shutdown()
            if hasattr(bus, '_detach'):  # PCAN接口的特殊方法
                bus._detach()  # 强制释放驱动级资源
            bus = None
            # os.system('peakcpl -r PCAN_USBBUS1')
            logging.info("CAN总线已关闭")
        init_flag = False
        mode_flag = 0

if __name__ == '__main__':
    socketio.run(app, debug=False, host='0.0.0.0', port=5001, allow_unsafe_werkzeug=True)
