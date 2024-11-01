#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import io
import random
import sys

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import rospy

from std_msgs.msg import Int32MultiArray, Bool
from robot_msgs.srv import Board_State, Board_StateRequest, Board_StateResponse
from robot_msgs.msg import CubePosition, ChessBoardState
from std_msgs.msg import Int32

from PySide2.QtCore import Qt, QTimer, QThread, QCoreApplication, Signal
from PySide2.QtWidgets import (
    QApplication,
    QWidget,
    QTableWidget,
    QHeaderView,
    QTableWidgetItem,
    QMenu,
    QMessageBox
)
from PySide2.QtGui import QImage, QGuiApplication, QColor, QBrush, QIcon, QPixmap 
from qframelesswindow import FramelessWindow, StandardTitleBar
from UI.Ui_chessGameWindows import Ui_Form
from qfluentwidgets import *


ROW=10
COL=8
class RosSpinThread(QThread):
    """ROS Spin 线程类，用于运行 rospy.spin()"""

    def run(self):
        rospy.spin()


class MainWindows(FramelessWindow, Ui_Form):
    """主窗口类"""
    
    boardErrorSignal = Signal()
    winGameSignal = Signal()
    loseGameSignal = Signal()
    error_shown = False  # 新增标志位，避免重复弹出

    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.initializeUI()
        self.initializeROS()
        self.initializeConnections()

    def initializeUI(self):
        """初始化用户界面相关设置"""
        self.isstart = False
        self.boardState = np.full((ROW, COL), 0)

         # 设置标题栏
        self.setTitleBar(StandardTitleBar(self))
        self.titleBar.raise_()

        # 居中显示窗口
        screen = QGuiApplication.primaryScreen()
        rect = screen.availableGeometry()
        w, h = rect.width(), rect.height()
        self.move(w // 2 - self.width() // 2, h // 2 - self.height() // 2)

        # 配置游戏表格
        self.configureGameTables()

    def initializeROS(self):
        """初始化ROS相关设置"""
        self.ros_spin_thread = RosSpinThread()
        self.ros_spin_thread.start()
        self.getBoard = rospy.Subscriber(
            "/boardState", ChessBoardState, self.getBoardState, queue_size=10
        )
        self.startGamepub = rospy.Publisher("/startGame", Bool, queue_size=10)
        self.chooseDegreepub = rospy.Publisher("/degree", Int32, queue_size=10) 
        self.cubeGameClient = rospy.ServiceProxy("/cubegame", Board_State)

    def initializeConnections(self):
        """初始化信号与槽的连接"""
        self.ruleButton.clicked.connect(self.ruleMessageBoxShow)
        self.startButton.clicked.connect(self.startGame)
        self.stopbutton.clicked.connect(self.stopGame)
        self.degreeButton.clicked.connect(self.chooseDegree)
        self.game_one_bt.clicked.connect(self.game_one_botton_function)
        self.game_two_bt.clicked.connect(self.game_two_botton_function)
        self.game_three_bt.clicked.connect(self.game_three_botton_function)
        self.game2_table.cellClicked.connect(self.show_color_menu1)
        self.game3_table.cellClicked.connect(self.show_color_menu2)
        self.game2_setButton_3.clicked.connect(self.clear_tabel)
        self.game3_setButton_3.clicked.connect(self.clear_tabel)
        self.game2_setButton.clicked.connect(self.set_pile_type)
        self.game3_setButton.clicked.connect(self.set_put_type)
        self.game2_setButton_2.clicked.connect(self.sendGoal1)
        self.game3_setButton_2.clicked.connect(self.sendGoal2)
        self.game3_numButton_1.clicked.connect(self.place1)
        self.game3_numButton_2.clicked.connect(self.place2)
        self.game3_numButton_3.clicked.connect(self.place3)
        self.game3_numButton_4.clicked.connect(self.place4)
        self.game3_numButton_5.clicked.connect(self.place5)
        self.game3_numButton_6.clicked.connect(self.place6)
        self.game2_put_Button_1.clicked.connect(self.put1)
        self.game2_put_Button_2.clicked.connect(self.put2)
        self.game2_put_Button_3.clicked.connect(self.put3)
        self.game2_put_Button_4.clicked.connect(self.put4)
        self.game2_put_Button_5.clicked.connect(self.put5)
        self.game2_put_Button_6.clicked.connect(self.put6)
        self.boardErrorSignal.connect(self.showBoardErrorMessage)
        self.winGameSignal.connect(self.winGameMessage)
        self.loseGameSignal.connect(self.loseGameMessage)
        

    def sendGoal1(self):
        self.sendGoal(self.color_changed_matrix, 1)

    def sendGoal2(self):
        self.sendGoal(self.vaild_stack_matrix, 2)

    def sendGoal(self, cube_matrix, type):
        state = Board_StateRequest()
        for index, matrix in enumerate(cube_matrix):
            state.positions.append(CubePosition(matrix[0], matrix[1]))
            state.angle.append(matrix[2])
            state.color.append(matrix[3])
        state.type = type
        resp = Board_StateResponse()
        resp = self.cubeGameClient.call(state)
        if resp is not None:
            print("任务完成结果为：", resp.success)
    
    def configureGameTables(self):
        """配置游戏表格的外观和行为"""
        for table in [self.game2_table, self.game3_table]:
            table.verticalHeader().hide()
            table.horizontalHeader().hide()
            table.setShowGrid(True)
            
            table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
            # 设置选择模式为多选
            table.setSelectionMode(table.MultiSelection)

            for i in range(table.rowCount()):
                for j in range(table.columnCount()):
                    item = QTableWidgetItem("")
                    item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
                    table.setItem(i, j, item)

            # 调用调整单元格比例的方法
            table.resizeEvent = self.adjustCellSizes  # 重载resizeEvent来动态调整单元格比例
            
            """配置 QTableWidget 的样式"""
            table.setStyleSheet("""
                QTableWidget {
                    background-color: rgba(0, 0, 0, 0);  /* 设置表格背景为透明 */
                    gridline-color: black;  /* 设置网格线为黑色 */
                    border-left: 1px solid black;  /* 设置单元格边框为黑色 */
                    border-top: 1px solid black;  /* 设置单元格边框为黑色 */          
                }
            """)
            
    def adjustCellSizes(self, event=None):
        """动态调整单元格大小,使得宽高比例保持1:1"""
        for table in [self.game2_table, self.game3_table]:
            rows = table.rowCount()
            cols = table.columnCount()
            if rows > 0 and cols > 0:
                # 获取表格可用的宽度和高度
                available_width = table.viewport().width()
                available_height = table.viewport().height()

                # 计算每个单元格的宽高，使得宽高比例为 1:1
                cell_size = min(available_width // cols, available_height // rows)

                for i in range(rows):
                    table.setRowHeight(i, cell_size)
                for j in range(cols):
                    table.setColumnWidth(j, cell_size)

        # 调用父类的resizeEvent方法
        if event:
            super(QTableWidget, table).resizeEvent(event)

    def place1(self):
        self.clear_tabel()
        array_5x5 = np.array(
            [
                [0, 1, 0, 1, 0],
                [0, 1, 0, 1, 0],
                [0, 1, 1, 1, 0],
                [0, 0, 0, 1, 0],
                [0, 0, 0, 1, 0],
            ]
        )
        self.place_templete_mode(array_5x5)

    def place2(self):
        self.clear_tabel()
        array_5x5 = np.array(
            [
                [0, 1, 1, 1, 0],
                [0, 1, 0, 0, 0],
                [0, 1, 1, 1, 0],
                [0, 1, 0, 1, 0],
                [0, 1, 1, 1, 0],
            ]
        )
        self.place_templete_mode(array_5x5)

    def place3(self):
        self.clear_tabel()
        array_5x5 = np.array(
            [
                [0, 0, 1, 0, 0],
                [0, 1, 0, 1, 0],
                [0, 0, 1, 0, 0],
                [0, 1, 0, 1, 0],
                [0, 0, 1, 0, 0],
            ]
        )
        self.place_templete_mode(array_5x5)

    def place4(self):
        self.clear_tabel()
        array_5x5 = np.array(
            [
                [0, 1, 1, 1, 0],
                [0, 1, 0, 1, 0],
                [0, 1, 1, 1, 0],
                [0, 1, 0, 0, 0],
                [0, 1, 0, 0, 0],
            ]
        )
        self.place_templete_mode(array_5x5)

    def place5(self):
        self.clear_tabel()
        array_5x5 = np.array(
            [
                [1, 0, 0, 0, 1],
                [0, 1, 0, 1, 0],
                [0, 0, 1, 0, 0],
                [0, 1, 0, 1, 0],
                [1, 0, 0, 0, 1],
            ]
        )
        self.place_templete_mode(array_5x5)

    def place6(self):
        self.clear_tabel()
        array_5x5 = np.array(
            [
                [1, 0, 1, 0, 1],
                [0, 1, 1, 1, 0],
                [1, 0, 1, 0, 1],
                [0, 1, 1, 1, 0],
                [1, 0, 1, 0, 1],
            ]
        )
        self.place_templete_mode(array_5x5)

    def place_templete_mode(self, matrix):
        """设置摆放模版"""
        colors = ["orange", "green", "blue"]
        for i in range(self.game3_table.rowCount()):
            for j in range(self.game3_table.columnCount()):
                if matrix[i][j] == 1:
                    item = self.game3_table.item(i, j)
                    # 从颜色列表中随机选择一个颜色
                    random_color = random.choice(colors)
                    item.setBackground(QBrush(QColor(random_color)))

    def put1(self):
        self.clear_tabel()
        array_5x5 = np.array(
            [
                [0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0],
                [0, 1, 1, 1, 0],
                [1, 1, 1, 1, 1],
            ]
        )
        self.put_templete_mode(array_5x5)

    def put2(self):
        self.clear_tabel()
        array_5x5 = np.array(
            [
                [0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0],
                [0, 1, 0, 1, 0],
                [0, 1, 0, 1, 0],
                [1, 1, 1, 1, 1],
            ]
        )
        self.put_templete_mode(array_5x5)

    def put3(self):
        self.clear_tabel()
        array_5x5 = np.array(
            [
                [0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0],
                [1, 0, 1, 0, 1],
                [1, 0, 1, 0, 1],
                [1, 0, 1, 0, 1],
            ]
        )
        self.put_templete_mode(array_5x5)

    def put4(self):
        self.clear_tabel()
        array_5x5 = np.array(
            [
                [0, 0, 1, 0, 0],
                [0, 0, 1, 0, 0],
                [0, 0, 1, 0, 0],
                [0, 0, 1, 0, 0],
                [0, 0, 1, 0, 0],
            ]
        )
        self.put_templete_mode(array_5x5)

    def put5(self):
        self.clear_tabel()
        array_5x5 = np.array(
            [
                [0, 0, 0, 0, 0],
                [0, 1, 0, 0, 1],
                [1, 1, 0, 1, 1],
                [1, 1, 0, 1, 1],
                [1, 1, 0, 1, 1],
            ]
        )
        self.put_templete_mode(array_5x5)

    def put6(self):
        self.clear_tabel()
        array_5x5 = np.array(
            [
                [0, 0, 0, 0, 0],
                [0, 1, 0, 1, 0],
                [0, 1, 0, 1, 0],
                [1, 1, 1, 1, 1],
                [1, 1, 1, 1, 1],
            ]
        )
        self.put_templete_mode(array_5x5)

    def put_templete_mode(self, matrix):
        """设置摆放模版"""
        colors = ["orange", "green", "blue"]
        for i in range(self.game2_table.rowCount()):
            for j in range(self.game2_table.columnCount()):
                if matrix[i][j] == 1:
                    item = self.game2_table.item(i, j)
                    # 从颜色列表中随机选择一个颜色
                    random_color = random.choice(colors)
                    item.setBackground(QBrush(QColor(random_color)))

    def clear_tabel(self):
        """清空表格中的颜色"""
        for table in [self.game2_table, self.game3_table]:
            for i in range(table.rowCount()):
                for j in range(table.columnCount()):
                    item = table.item(i, j)
                    item.setBackground(QBrush(QColor(255, 255, 255, 0)))
            table.clearSelection()
            table.clearFocus()

    def set_pile_type(self):
        rows = self.game2_table.rowCount()
        cols = self.game2_table.columnCount()

        # 创建一个矩阵来记录每个单元格的状态
        color_changed = np.zeros((rows, cols), dtype=int)
        vaild_stack = []
        # 遍历所有单元格，记录被更改颜色的单元格状态
        for i in range(rows):
            for j in range(cols):
                item = self.game2_table.item(i, j)
                color = item.background().color()

                if color == QColor("orange"):
                    color_changed[i, j] = 1
                elif color == QColor("blue"):
                    color_changed[i, j] = 2
                elif color == QColor("green"):
                    color_changed[i, j] = 3
                # 默认值为0，不需要特别处理

        # 检查上层格子的条件
        for i in range(rows - 1):  # 不检查最底层
            for j in range(cols):
                if color_changed[i, j] != 0:  # 如果当前格子被改变颜色
                    if color_changed[i + 1, j] == 0:  # 如果下层格子没有被改变颜色
                        w = MessageBox(
                            "无效数据",
                            "数据无效请重新勾选，上层格子在其下方没有被改变颜色的格子！",
                            self,
                        )
                        w.exec()
                        return
        for i in range(rows - 1, -1, -1):
            for j in range(cols):
                if color_changed[i, j] != 0:
                    vaild_stack.append((i, j, 0, color_changed[i, j]))
        # 如果所有检查通过，则将color_changed保存为类内变量
        self.color_changed_matrix = vaild_stack
        print("所有的上层格子均满足条件, color_changed_matrix 已创建。")

    def is_valid_position(self, row, col, rows, cols):
        """检查位置是否在有效的表格范围内"""
        return 0 <= row < rows and 0 <= col < cols

    def has_no_adjacent_neighbors(self, row, col, color_stacked, rows, cols):
        """检查指定格子是否至少有一组对边是没有相邻格子的，返回是否有效及方向"""
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 上  # 下  # 左  # 右

        # 检查左右方向是否有相邻格子
        left_right_count = 0
        up_down_count = 0

        # 检查左右
        for i in [2, 3]:  # 左右方向
            direction = directions[i]
            new_row, new_col = row + direction[0], col + direction[1]
            if (
                self.is_valid_position(new_row, new_col, rows, cols)
                and color_stacked[new_row, new_col] == 1
            ):
                left_right_count += 1

        # 如果左右没有相邻格子，则满足条件，返回 True 和 0 (表示左右方向)
        if left_right_count == 0:
            return True, 90

        # 检查上下
        for i in [0, 1]:  # 上下方向
            direction = directions[i]
            new_row, new_col = row + direction[0], col + direction[1]
            if (
                self.is_valid_position(new_row, new_col, rows, cols)
                and color_stacked[new_row, new_col] == 1
            ):
                up_down_count += 1

        # 如果上下没有相邻格子，则满足条件，返回 True 和 90(表示上下方向)
        if up_down_count == 0:
            return True, 0

        # 如果所有方向都有相邻格子，则不满足条件
        return False, None

    def set_put_type(self):
        rows = self.game3_table.rowCount()
        cols = self.game3_table.columnCount()
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 上  # 下  # 左  # 右
        # 创建一个矩阵来记录每个单元格的状态
        color_changed = np.zeros((rows, cols), dtype=int)
        color_stacked = np.zeros((rows, cols), dtype=int)
        # 遍历所有单元格，记录被更改颜色的单元格状态
        for i in range(rows):
            for j in range(cols):
                item = self.game3_table.item(i, j)
                color = item.background().color()

                if color == QColor("orange"):
                    color_changed[i, j] = 1
                elif color == QColor("blue"):
                    color_changed[i, j] = 2
                elif color == QColor("green"):
                    color_changed[i, j] = 3
        all_changed_positions = [
            (i, j) for i in range(rows) for j in range(cols) if color_changed[i, j] > 0
        ]
        stack = [(2, 2)]
        vaild_stack = []
        for start_row, start_col in all_changed_positions:
            if color_stacked[start_row, start_col] != 0:
                continue

            # 深度遍历
            stack = [(start_row, start_col)]
            while stack:
                current_row, current_col = stack.pop()
                if color_stacked[current_row, current_col] != 0:
                    continue
                color_stacked[current_row, current_col] = 2

                if color_changed[current_row, current_col] != 0:
                    color_stacked[current_row, current_col] = 1
                    res, angle = self.has_no_adjacent_neighbors(
                        current_row, current_col, color_stacked, rows, cols
                    )
                    if res:
                        vaild_stack.append(
                            (
                                current_row,
                                current_col,
                                angle,
                                color_changed[current_row, current_col],
                            )
                        )
                    else:
                        w = MessageBox(
                            "无效数据", "数据无效请重新勾选，无法正常摆放！", self
                        )
                        w.exec_()
                        return

                for direction in directions:
                    new_row, new_col = (
                        current_row + direction[0],
                        current_col + direction[1],
                    )
                    if (
                        self.is_valid_position(new_row, new_col, rows, cols)
                        and color_changed[new_row, new_col] > 0
                        and color_stacked[new_row, new_col] == 0
                    ):
                        stack.append((new_row, new_col))

            self.vaild_stack_matrix = vaild_stack

    def show_color_menu1(self, row, column):
        """显示颜色菜单以更改 game2_table 单元格的颜色"""
        self.show_color_menu(self.game2_table, row, column)

    def show_color_menu2(self, row, column):
        """显示颜色菜单以更改 game3_table 单元格的颜色"""
        self.show_color_menu(self.game3_table, row, column)

    def show_color_menu(self, table, row, column):
        """通用的颜色菜单显示函数"""
        menu = QMenu(self)
        
        # 创建颜色选项
        colors = {
            "橙色": "orange",
            "蓝色": "blue",
            "绿色": "green",
            "复原": "transparent",
        }

        # 添加颜色选项到菜单
        for color_name in colors.keys():
            menu.addAction(color_name)

        item_rect = table.visualItemRect(table.item(row, column))
        table_pos = table.viewport().mapToGlobal(item_rect.center())
        action = menu.exec_(table_pos)

        if action:
            selected_items = table.selectedItems()
            # 获取用户选择的颜色
            selected_color = colors[action.text()]

            for item in selected_items:
                # 设置背景颜色，如果选择了复原，则重置为透明
                item.setBackground(QBrush(QColor(selected_color) if selected_color != "transparent" else table.palette().base()))

            table.clearSelection()
            table.clearFocus()

    def game_one_botton_function(self):
        """切换到第一个游戏视图"""
        self.stackedWidget.setCurrentIndex(0)
        buf = self.draw_gomoku_board(self.boardState)
        self.chessboardLabel.setImage(buf)

    def game_two_botton_function(self):
        """切换到第二个游戏视图"""
        self.stackedWidget.setCurrentIndex(1)

    def game_three_botton_function(self):
        """切换到第三个游戏视图"""
        self.stackedWidget.setCurrentIndex(2)

    def closeEvent(self, event):
        """关闭事件处理"""
        rospy.signal_shutdown("Shutting down ROS thread")
        self.ros_spin_thread.wait()
        event.accept()

    def ruleMessageBoxShow(self):
        """显示游戏规则对话框"""
        w = MessageBox(
            "游戏规则",
            "单击“开始游戏”运行;\n玩家默认持黑棋(橙色方块），机器人持白棋（蓝色方块）;\n机器人将会等待玩家行动结束后自动下棋;\n",
            self,
        )
        w.exec()

    def chooseDegree(self):
        """选择游戏难度"""
        w = MessageBox("难度设置", "请选择游戏难度：", self)

        # 添加按钮选项
        w.yesButton.setText("正常")
        w.cancelButton.setText("困难")

        # 根据用户点击的按钮判断选择
        if w.exec():
            self.chooseDegreepub.publish(Int32(0))
            print("选择了正常难度")
            self.difficulty = "正常"
        else:
            self.chooseDegreepub.publish(Int32(1))
            print("选择了困难难度")
            self.difficulty = "困难"
    
    def startGame(self):
        """开始游戏并显示棋盘"""
        self.isstart = True
        self.startGamepub.publish(Bool(True))
        # 检查 boardState 是否为空
        if np.any(self.boardState):
            # 弹出提示框
            QMessageBox.warning(self, "提示", "棋盘状态不为空，请先重置棋盘!")
            self.isstart = False
            self.startGamepub.publish(Bool(False))
            return  # 如果boardState不为空，则不继续执行后续代码
        self.startButton.setEnabled(False)
        
    matplotlib.use('Agg')  # 使用非GUI后端，避免Tkinter后端和多线程操作不兼容
    def draw_gomoku_board(self, board):
        """绘制五子棋棋盘并返回图像"""     
        figsizeH = self.chessboardWidget.size().height() / 90
        figsizeW = self.chessboardWidget.size().width() / 90
        # 如果没有图形对象则创建一个，否则清空当前图形
        if not hasattr(self, 'fig'):  # 检查是否已经存在图形
            self.fig, self.ax = plt.subplots(figsize=(figsizeW, figsizeH))  # 创建图形和坐标轴
        else:
            self.ax.clear()  # 清空现有图形内容
            self.fig.set_size_inches(figsizeW, figsizeH, forward=True)  # 更新图形大小 forward=True立即生效

        # 创建10x8的网格
        for x in range(ROW):
            self.ax.plot([x, x], [0, COL - 1], color='black')
        for y in range(COL):
            self.ax.plot([0, ROW - 1], [y, y], color='black')

        # 设置行列标记
        self.ax.set_xticks(range(ROW))
        self.ax.set_yticks(range(COL))

        # 设置网格的边界
        self.ax.set_xlim(-0.5, ROW - 0.5)
        self.ax.set_ylim(-0.5, COL - 0.5)

        # 设置透明背景
        self.fig.patch.set_alpha(0.0)  # 图形背景透明
        self.ax.set_facecolor((1, 1, 1, 0))  # 网格背景透明
        self.ax.tick_params(axis='both', which='both', length=0)

        # 隐藏坐标轴标签
        self.ax.tick_params(left=False, bottom=False)
        plt.gca().invert_yaxis()  # 翻转y轴以匹配棋盘布局
        
        # 检查游戏是否处于活动状态
        if self.isstart:
            # 绘制棋子
            for x in range(ROW):
                for y in range(COL):
                    if board[x, y] == -1:  # 黑棋(机器人)
                        self.ax.plot(x, y, "ko", markersize=20)
                    elif board[x, y] == 1:  # 白棋
                        self.ax.plot(x, y, "wo", markersize=20, markeredgecolor="black")

        # 保存图像到内存中
        buf = io.BytesIO()
        plt.savefig(buf, format="png", bbox_inches='tight', pad_inches=0.1, transparent=True)
        buf.seek(0) 
        qimage = QImage.fromData(buf.read())  # 转换为QImage以供PyQt使用

        return qimage
    
    def resizeEvent(self, event):
        """捕捉窗口大小变化事件并重新绘制"""
        buf = self.draw_gomoku_board(self.boardState)
        self.chessboardLabel.setImage(buf)
        super().resizeEvent(event)  # 调用父类的resizeEvent以处理其他默认行为

    def stopGame(self):
        self.startGamepub.publish(Bool(False))
        self.isstart = False # 设置游戏停止的标志位
        buf = self.draw_gomoku_board(np.full((ROW, COL), 0)) # 重置棋盘
        self.chessboardLabel.setImage(buf)
        self.startButton.setEnabled(True) # 允许开始游戏

    def getBoardState(self, msg):
        """接收并处理 ROS 发布的棋盘状态""" 
        
        if msg.iserror and not self.error_shown:  # 如果有错误且未显示过
            # 发出信号
            self.boardErrorSignal.emit()
            self.error_shown = True  # 标志位设置为 True，防止重复弹出

        elif not msg.iserror:  # 如果错误消失了，允许再次弹出
            self.error_shown = False
            
            data = np.array(msg.board)
            
            if msg.iswin == 2:
                self.winGameSignal.emit()

            elif msg.iswin == 1:
                self.loseGameSignal.emit()

            elif data.size != ROW*COL:
                raise ValueError(
                    f"Received data size is {len(msg.board)},cannot reshape into 10*8 board."
                )

            if msg.turn % 2 != 0:
                self.playerName.setText(QCoreApplication.translate(
                    "Form", u"<html><head/><body><p><span style=\" font-size:22pt; font-weight:600; color:#ef2929;\">Robot</span></p></body></html>", None))
            else:
                self.playerName.setText(QCoreApplication.translate(
                    "Form", u"<html><head/><body><p><span style=\" font-size:22pt; font-weight:600; color:#ef2929;\">Human</span></p></body></html>", None))
                
            self.boardState = data.reshape((ROW, COL))
            buf = self.draw_gomoku_board(self.boardState)
            self.chessboardLabel.setImage(buf)

    def showBoardErrorMessage(self):
        """显示棋盘错误消息的槽函数"""
        w = MessageBox("棋盘错误", "请遵守游戏规则！", self)
        w.exec_()
    
    def winGameMessage(self):
            w = MessageBox("游戏结束", "恭喜你赢了！", self)
            w.exec_()

    def loseGameMessage(self):
            w = MessageBox("游戏结束", "很抱歉你输了！", self)
            w.exec_()


if __name__ == "__main__":
    # 设置高DPI支持
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps)

    # 初始化 ROS 节点
    rospy.init_node("chessboard_detect", anonymous=True)

    # 启动应用程序
    app = QApplication(sys.argv)
    windows = MainWindows()
    windows.show()

    sys.exit(app.exec_())
    