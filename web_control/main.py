#!/usr/bin/env python3
# encoding: utf-8

from flask import Flask, render_template
from flask_socketio import SocketIO
import serial
from serial.serialutil import SerialException
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
is_initialized = False
lock = threading.Lock()

# 初始化日志系统
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

app = Flask(__name__)
socketio = SocketIO(app)
# socketio = SocketIO(app, async_mode='eventlet')  # 明确异步模式

# ser = serial.Serial('COM5', 921600, timeout=1)

# def send_command(hex_data):
#     try:
#         ser.write(bytes.fromhex(hex_data))
#     except SerialException as e:
#         print(f"An error occurred: {e}")
#         ser.close()
#         ser.open()

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('start')
def handle_start():
    """
    初始化CAN总线和电机控制器
    返回: (bus对象, 电机控制器列表)
    """
    global bus, motors, is_initialized
    with lock:
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
            motor2 = CANMotorController(bus, motor_id=127, main_can_id=254)
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
            is_initialized = True
            # return bus, motors

        except Exception as e:
            logging.error(f"电机初始化失败: {str(e)}")
            raise

@socketio.on('forward')
def handle_forward():
    if not is_initialized:
        logging.info("请启动电机！")
        return
    with lock:
        logging.info("开始顺时针运动")
        for motor in motors:
            motor.write_single_param("loc_ref", value=2)  # 顺时针方向
        time.sleep(0.5)

@socketio.on('backward')
def handle_backward():
    if not is_initialized:
        logging.info("请启动电机！")
        return
    with lock:
        logging.info("开始逆时针运动")
        for motor in motors:
            motor.write_single_param("loc_ref", value=-2)  # 逆时针方向
        time.sleep(0.5)

@socketio.on('zero')
def handle_zero():
    if not is_initialized:
        logging.info("请启动电机！")
        return
    with lock:
        logging.info("回到零点位置")
        for motor in motors:
            motor.write_single_param("loc_ref", value=0)  # 回零
        time.sleep(0.5)

@socketio.on('stop')
def handle_stop():
    global bus, motors, is_initialized
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
        is_initialized = False

if __name__ == '__main__':
    socketio.run(app, debug=False, host='0.0.0.0', port=5001, allow_unsafe_werkzeug=True)
