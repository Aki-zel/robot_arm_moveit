#!/usr/bin/env python
import rospy
from robot_msgs.srv import NextMove, NextMoveResponse
from Board import Board
from AI_MCTS_Net import AI_MCTS_Net
from PolicyValueNet_from_junxiaosong import PolicyValueNet_from_junxiaosong 

# 模型地址
model_path =  "/home/yds/rwm_moveit/src/robot_control/myomega/weights/best_5000_10.h5"

def handle_next_move(req):
    # 初始化一个新的棋盘
    board = Board()
    board.last_action = (req.last_chess_x, req.last_chess_y)
    board.set_state(req.board_state)  # 将一维数组转换为棋盘状态

    # 重新创建AI玩家，每次请求都重新初始化
    network = PolicyValueNet_from_junxiaosong(is_new_model=False, model_record_path=model_path)
    policy_value_function = network.predict  # 传递预测函数句柄
    board_to_xlabel = network.board_to_xlabel(board)  # 根据新的棋盘生成 xlabel
    ai_player = AI_MCTS_Net(policy_value_function, board_to_xlabel, search_times=1000)

    # AI 做出决策
    action = ai_player.take_action(board, is_output_action=False)

    # 返回AI决策的坐标
    return NextMoveResponse(x=action[0], y=action[1])

def next_move_server():
    rospy.init_node('next_move_server')
    # 定义服务，'next_move'是服务名，NextMove是服务类型
    rospy.Service('next_move', NextMove, handle_next_move)
    print("Ready to calculate next move.")
    rospy.spin()

if __name__ == "__main__":
    next_move_server()
