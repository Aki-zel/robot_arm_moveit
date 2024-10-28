#!/usr/bin/env python
import rospy
from robot_msgs.srv import NextMove  # 导入服务消息类型
import numpy as np

# 定义棋盘结构
class Checkerboard:
    def __init__(self, rows=10, cols=8):
        self.rows = rows
        self.cols = cols
        self.board = np.full((self.rows, self.cols), 0)  # 初始化棋盘为0(空位)

    def set_piece(self, row, col, player):
        """设置指定位置的棋子,player 为 1 或 -1"""
        if 0 <= row < self.rows and 0 <= col < self.cols:
            self.board[row, col] = player

    def flatten_board(self):
        """将棋盘展平成一维数组"""
        return self.board.flatten()

def get_next_move(board_state, last_chess_x, last_chess_y):
    rospy.wait_for_service('next_move')  # 等待服务启动
    try:
        # 创建服务代理
        next_move_service = rospy.ServiceProxy('next_move', NextMove)

        # 请求服务，传递棋盘状态和上一步棋子的坐标
        response = next_move_service(board_state, last_chess_x, last_chess_y)

        # 返回 AI 的决策
        return response.x, response.y
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None, None

if __name__ == "__main__":
    rospy.init_node('next_move_client')

    # 创建棋盘状态
    board = Checkerboard()

    # 设置棋盘的状态，例如在某个位置放置棋子
    board.set_piece(5, 5, 1)  # 设置 (5, 5) 位置的玩家为 1
    board.set_piece(4, 4, -1) # 设置 (4, 4) 位置的对手为 -1
    board.set_piece(5, 3, 1)  # 设置 (5, 3) 位置的玩家为 1

    # 获取棋盘状态并展平成一维数组
    board_state = board.flatten_board()

    # 假设上一步棋子
    last_chess_x = 5
    last_chess_y = 6

    # 调用服务并获取 AI 的下一步决策
    x, y = get_next_move(board_state, last_chess_x, last_chess_y)

    if x is not None and y is not None:
        rospy.loginfo(f"AI recommends move at: ({x}, {y})")
    else:
        rospy.logerr("Failed to get the next move from the AI.")
