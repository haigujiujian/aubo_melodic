# -*- coding: UTF-8 -*-
import rospy
import math
from std_msgs.msg import String,Float64,Bool
import time
import numpy
import os
import socket
import yaml
from aubo_kinematics import *
from aubo_robotcontrol import *


class aubo_control():
    def __init__(self):
        self.configname='/home/haigujiujian/catkin_ws/src/arm_motion/config/camera_aubo_config.yaml'
        self.yamlDic=None
        self.Opreating_yaml()
        self.aubo_my_kinematics=Aubo_kinematics()
        self.Aubo_IP=self.yamlDic['AuboIP']
        self.maxacctuple=tuple(self.yamlDic['Aubomaxacctuple'])
        self.maxvelctuple=tuple(self.yamlDic['Aubomaxvelctuple'])
    def Init_node(self):
        rospy.init_node("auboarm_control")
    def Init_aubo_driver(self):
        # 初始化logger
        #logger_init()
        # 启动测试
        print("{0} test beginning...".format(Auboi5Robot.get_local_time()))
        # 系统初始化
        Auboi5Robot.initialize()
        # 创建机械臂控制类
        robot = Auboi5Robot()
        # 创建上下文
        handle = robot.create_context()
        # 打印上下文
        print("robot.rshd={0}".format(handle))
        try:

            # 链接服务器
            ip = self.Aubo_IP#'192.168.1.11'
            port = 8899
            result = robot.connect(ip, port)
            if result != RobotErrorType.RobotError_SUCC:
                print("connect server{0}:{1} failed.".format(ip, port))
            else:
                # # 重新上电
                # robot.robot_shutdown()
                #
                # # 上电
                #robot.robot_startup()
                #
                # # 设置碰撞等级
                # robot.set_collision_class(7)

                # 设置工具端电源为１２ｖ
                # robot.set_tool_power_type(RobotToolPowerType.OUT_12V)

                # 设置工具端ＩＯ_0为输出
                robot.set_tool_io_type(RobotToolIoAddr.TOOL_DIGITAL_IO_0, RobotToolDigitalIoDir.IO_OUT)

                # 获取工具端ＩＯ_0当前状态
                tool_io_status = robot.get_tool_io_status(RobotToolIoName.tool_io_0)
                # print("tool_io_0={0}".format(tool_io_status))

                # 设置工具端ＩＯ_0状态
                robot.set_tool_io_status(RobotToolIoName.tool_io_0, 1)

                # 获取控制柜用户DI
                io_config = robot.get_board_io_config(RobotIOType.User_DI)

                # 输出DI配置
                # print(io_config)

                # 获取控制柜用户DO
                io_config = robot.get_board_io_config(RobotIOType.User_DO)

                # 输出DO配置
                # print(io_config)
                # 当前机械臂是否运行在联机模式
                # print("robot online mode is {0}".format(robot.is_online_mode()))
        except RobotError as e:
            logger.error("{0} robot Event:{1}".format(robot.get_local_time(), e))
        return robot
    def Opreating_yaml(self,):       
        yaml_path = self.configname
        # print yaml_path
        file_data = open(yaml_path)
        self.yamlDic = yaml.load(file_data)
        # print "hhh",self.yamlDic
        file_data.close()
    def DisConnect_Aubo_No_ShutDown(self,auboRobot):
        # 断开服务器链接
        auboRobot.disconnect()
    def DisConnect_Aubo(self,auboRobot):
        # 断开服务器链接
        if auboRobot.connected:
            # 关闭机械臂
            auboRobot.robot_shutdown()
            # 断开机械臂链接
            auboRobot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()
        print("{0} test completed.".format(Auboi5Robot.get_local_time()))
    def Aubo_trajectory_init(self,robot):
        joint_status = robot.get_joint_status()
        print("joint_status={0}".format(joint_status))
        # 初始化全局配置文件
        robot.init_profile()
        # 设置关节最大加速度
        robot.set_joint_maxacc(self.maxacctuple)#(2.5, 2.5, 2.5, 2.5, 2.5, 2.5)

        # 设置关节最大加速度
        robot.set_joint_maxvelc(self.maxvelctuple)#(0.3, 0.3, 0.3, 0.3, 0.3, 0.3)
        # 设置机械臂末端最大线加速度(m/s)
        robot.set_end_max_line_acc(0.5)
        # 获取机械臂末端最大线加速度(m/s)
        # robot.set_end_max_line_velc(0.2)
        robot.set_end_max_line_velc(0.5)
    

if __name__=='__main__':
    ros_rate=1
    auboctrl=aubo_control()
    auboctrl.Init_node()
    rate=rospy.Rate(ros_rate)
    try:
        Robot=auboctrl.Init_aubo_driver()
        auboctrl.Aubo_trajectory_init(Robot)
    except:
        rospy.loginfo("aubo init is not ok")
        
    while not rospy.is_shutdown():

        rate.sleep()