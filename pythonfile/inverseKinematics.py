#!/usr/bin/env python3
# encoding: utf-8
# 4自由度机械臂逆运动学：给定相应的坐标（X,Y,Z），以及俯仰角，计算出每个关节转动的角度
# 仅求解给定的一组坐标和俯仰角对应的数据，无解则返回False

import logging
from math import *

class IK:
    # 电机从下往上数
    # 即4自由度机械臂的连杆参数
    l1 = 13.7    # 机械臂底座中心到第二个电机中心轴的距离13.7cm
    l2 = 14.5   # 第二个电机到第三个电机的距离14.5cm
    l3 = 14.5    #第三个电机到第四个电机的距离14.5cm
    l4 = 10    # 第四个电机到手爪末端距离10cm

    def setLinkLength(self, L1=l1, L2=l2, L3=l3, L4=l4):
        # 更改机械臂的连杆长度，为了适配相同结构不同长度的机械臂
        self.l1 = L1
        self.l2 = L2
        self.l3 = L3
        self.l4 = L4

    def getLinkLength(self):
        # 获取当前设置的连杆长度
            return {"L1":self.l1, "L2":self.l2, "L3":self.l3, "L4":self.l4}

    def getRotationAngle(self, coordinate_data, Alpha):
        # 给定指定坐标和俯仰角，返回每个关节应该旋转的角度，如果无解返回False
        # coordinate_data为夹持器末端坐标（即目标点坐标），坐标单位cm， 以元组形式传入，例如(0, 5, 10)
        # Alpha为夹持器与水平面的夹角，单位度

        # 设夹持器末端为P(X, Y, Z), 坐标原点为O, 原点为云台中心在地面的投影， P点在地面的投影为P_
        # l1与l2的交点为A, l2与l3的交点为B，l3与l4的交点为C
        # CD与PD垂直，CD与z轴垂直，则俯仰角Alpha为DC与PC的夹角, AE垂直DP_， 且E在DP_上， CF垂直AE，且F在AE上
        # 夹角表示：例如AB和BC的夹角表示为ABC
        X, Y, Z = coordinate_data
        # 求底座旋转角度
        theta1 = round(degrees(atan2(Y, X)),4)

        P_O = sqrt(X * X + Y * Y)  # P_到原点O距离
        CD = self.l4 * cos(radians(Alpha))  # Cartesian Distance（笛卡尔距离），即末端连杆在水平面（XY平面）的投影长度
        PD = self.l4 * sin(radians(Alpha))  # Pitch Direction（俯仰方向），末端连杆在垂直方向（Z轴）的投影长度，当俯仰角为正时，PD为正，当俯仰角为负时，PD为负
        AF = P_O - CD  # Adjustment Factor（调整因子），用于计算肩关节到目标点的水平修正量
        CF = Z - self.l1 - PD  # Compensation Factor（补偿因子），表示肩关节到目标点的垂直高度差
        AC = sqrt(pow(AF, 2) + pow(CF, 2))  # Adjacent Connection（邻接连接），肩关节（A点）到腕关节（C点）的直线距离
        if round(CF, 4) < -self.l1:
            logging.debug('高度低于0, CF(%s)<l1(%s)', CF, -self.l1)
            return False
        if self.l2 + self.l3 < round(AC, 4):  # 两边之和小于第三边
            logging.debug('不能构成连杆结构, l2(%s) + l3(%s) < AC(%s)', self.l2, self.l3, AC)
            return False

        # 求theta3
        cos_ABC = round((-pow(AC, 2) + pow(self.l2, 2) + pow(self.l3, 2)) / (2 * self.l2 * self.l3), 4)  # 余弦定理
        if abs(cos_ABC) > 1:
            logging.debug('不能构成连杆结构, abs(cos_ABC(%s)) > 1', cos_ABC)
            print('不能构成连杆结构, abs(cos_ABC(%s)) > 1', cos_ABC)
            return False
        ABC = acos(cos_ABC)  # 反三角算出弧度
        theta3 = 180.0 - degrees(ABC)

        # 求theta2
        CAF = acos(AF / AC)
        cos_BAC = round((pow(AC, 2) + pow(self.l2, 2) - pow(self.l3, 2)) / (2 * self.l2 * AC), 4)  # 余弦定理
        if abs(cos_BAC) > 1:
            logging.debug('不能构成连杆结构, abs(cos_BAC(%s)) > 1', cos_BAC)
            return False
        if CF < 0:
            zf_flag = -1
        else:
            zf_flag = 1
        theta2 = round(degrees(CAF * zf_flag + acos(cos_BAC)), 4)

        # 求theta4
        theta4 = float(round(Alpha - theta2 - theta3, 4))

        return {"theta4": theta4, "theta3": theta3, "theta2": theta2, "theta1": theta1}  # 有解时返回角度字典

def main():
    # 初始化日志系统
    logging.basicConfig(
        level=logging.INFO,  # CRITICAL, ERROR, WARNING, INFO, DEBUG
        format='%(asctime)s - %(levelname)s - %(message)s'
    )
    # logger = logging.getLogger(__name__)
    ik = IK()
    print('连杆长度：', ik.getLinkLength())
    print(ik.getRotationAngle((0, 0, ik.l1 + ik.l2 + ik.l3 + ik.l4), 90))
    print(ik.getRotationAngle((0, 33.775, 33.2), 30))
    print(ik.getRotationAngle((29.25, 16.8875, 33.2), 30))

if __name__ == '__main__':
    main()