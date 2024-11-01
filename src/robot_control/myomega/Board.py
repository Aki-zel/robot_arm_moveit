import numpy as np

# 玩家标识
o = 1
x = -1
empty = 0
n_in_a_row = 5  # 几子连珠
row = 10
col = 8
start_player = 1  # start player

def coordinates_set(width, height):
    """根据宽和高生成一个坐标元组集合。"""
    return {(i, j) for i in range(width) for j in range(height)}

class Board:
    def __init__(self):
        self.reset()  # 重置棋盘

    def __copy__(self):
        new_board = Board()
        new_board.board = self.board.copy()
        new_board.available_actions = self.available_actions.copy()
        new_board.last_action = self.last_action
        new_board.current_player = self.current_player
        return new_board

    def reset(self):
        """重置棋盘。"""
        self.board = np.zeros((row, col))
        self.available_actions = coordinates_set(row, col)
        self.last_action = None
        self.current_player = o  # 设置初始玩家

    def step(self, action):
        """执行下一步动作，调用后棋盘状态改变。"""
        if action not in self.available_actions:
            return False
        self.board[action] = self.current_player
        self.available_actions.remove(action)
        self.last_action = action
        self.current_player = -self.current_player  # 轮到对方
        return True

    def result(self):
        """分析当前局面是否有玩家胜利，或者平局，或者未结束。"""
        for piece in coordinates_set(row, col) - self.available_actions:
            i, j = piece

            # 检查横向、纵向和两条对角线
            for di, dj in [(0, 1), (1, 0), (1, 1), (-1, 1)]:
                count = sum(1 for k in range(n_in_a_row) if 
                            0 <= i + k * di < row and 
                            0 <= j + k * dj < col and 
                            self.board[i + k * di, j + k * dj] == self.current_player)
                if count == n_in_a_row:
                    return True, self.current_player

        return len(self.available_actions) == 0, empty  # 检查平局或未结束

    def set_state(self, board_state):
        """将一维数组转为二维棋盘，并设置棋盘状态"""
        self.board = np.array(board_state).reshape(row, col)
        self.available_actions = {(i, j) for i in range(row) for j in range(col) if self.board[i, j] == empty}