#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
from PySide2.QtCore import Qt, QTimer,QThread
from PySide2.QtWidgets import QApplication, QWidget 
from PySide2.QtGui import QImage,QGuiApplication
from qframelesswindow import FramelessMainWindow, StandardTitleBar
from UI.chessGameWindows_ui import Ui_Form
from qfluentwidgets import *
import matplotlib.pyplot as plt
import numpy as np
import io
import rospy
import cv2
import cv_bridge
from std_msgs.msg import Int32MultiArray


class RosSpinThread(QThread):
    def run(self):
        rospy.spin()
class MainWindows(QWidget, Ui_Form):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # 设置标题栏
        # self.setTitleBar(StandardTitleBar(self))
        # self.titleBar.raise_()
        # self.resize(1000,650)
        self.setWindowTitle("五子棋游戏")
        self.start = False
        self.boardState = np.full((9, 9), -1)
        # 居中
        screen = QGuiApplication.primaryScreen()
        rect = screen.availableGeometry()
        w, h = rect.width(), rect.height()
        self.move(w//2-self.width()//2, h//2-self.height()//2)
        self.ruleButton.clicked.connect(self.ruleMessageBoxShow)
        self.startButton.clicked.connect(self.startGame)
        self.imageLabel.text = ""
        self.getBoard = rospy.Subscriber(
            "/boardState", Int32MultiArray, self.getBoardState, queue_size=10)
        # self.imageLabel.setImage()
            # 启动 ROS spin 线程
        self.ros_spin_thread = RosSpinThread()
        self.ros_spin_thread.start()

    def closeEvent(self, event):
        rospy.signal_shutdown("Shutting down ROS thread")
        self.ros_spin_thread.wait()  # 等待线程完全退出
        event.accept()


    def ruleMessageBoxShow(self):
        w = MessageBox(
            "游戏规则", "单击“开始游戏”运行;\n玩家默认持黑棋（橙色方块），机器人持白棋（蓝色方块）;\n机器人将会等待玩家行动结束后自动下棋；\n", self)

        if w.exec():
            print('确认')
        else:
            print('取消')

    def startGame(self):
        self.start = True
        buf = self.draw_gomoku_board(self.boardState)
        self.imageLabel.setImage(buf)

    def draw_gomoku_board(self, board):
        """
        绘制五子棋棋盘，board 是一个 10x10 的 numpy 数组，-1 表示空，0 表示黑棋，1 表示白棋。
        """
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.set_xlim(-1, 9)
        ax.set_ylim(-1, 9)
        ax.set_xticks(np.arange(0, 9, 1))
        ax.set_yticks(np.arange(0, 9, 1))
        ax.grid(True)

        # 绘制棋子
        for y in range(9):
            for x in range(9):
                if board[y, x] == 0:  # 黑棋
                    ax.plot(x, y, 'ko', markersize=20)
                elif board[y, x] == 1:  # 白棋
                    ax.plot(x, y, 'wo', markersize=20,
                            markeredgecolor='black')

        ax.set_aspect('equal')
        plt.gca().invert_yaxis()
        plt.axis('on')

        # 将图像转换为字节流
        buf = io.BytesIO()
        plt.savefig(buf, format='png')
        buf.seek(0)
        qimage = QImage.fromData(buf.read())

        return qimage

    def getBoardState(self, msg):
        """
        将 Int32MultiArray 转换为 10x10 的 numpy 数组。
        :param msg: Int32MultiArray 消息
        :return: 10x10 的 numpy 数组
        """
        # 将接收到的数据转换为 numpy 数组
        data = np.array(msg.data)

        # 检查数据长度是否为 100（10x10）
        if data.size != 81:
            raise ValueError(
                "Received data size is not 100, cannot reshape into 10x10 board.")
        if self.start:
            self.playerName="Human"
            self.start=False
        else:
            self.start=True
            self.playerName="Robot"
        # 将一维数组重塑为 10x10 的二维数组
        self.boardState = data.reshape((9, 9))
        buf = self.draw_gomoku_board(self.boardState)
        self.imageLabel.setImage(buf)

if __name__ == '__main__':
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps)
    rospy.init_node("chessboard_detect", anonymous=True)

    app = QApplication(sys.argv)
    windows = MainWindows()
    windows.show()

    # rospy.spin()
    sys.exit(app.exec_())
