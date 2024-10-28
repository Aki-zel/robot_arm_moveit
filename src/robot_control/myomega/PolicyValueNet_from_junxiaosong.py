from Network import Network
import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import Input
from tensorflow.keras import layers
from tensorflow.keras import optimizers
from tensorflow.keras import regularizers
import tensorflow.keras.backend as K

import Board as BOARD

# https://github.com/CLOXnu/Omega_Gomoku_AI/issues/2
tf.compat.v1.disable_eager_execution()


def data_augmentation_new(x_label, y_label):
    """
    数据扩充。
    Data augmentation.
    :param x_label: 神经网络的输入 x_label。 Input of the neural network.
    :param y_label: 神经网络的输出 x_label。 Output of the neural network.
    :return: 数据扩充后的 data.  Data after data augmentation.
    """
    all_action_probs, values = y_label
    extend_data = []
    for board_input, action_probs, value in zip(x_label, all_action_probs, values):
        for i in [0, 2]:
            # rotate counterclockwise
            new_board_input = np.array([np.rot90(one_board_input, i) for one_board_input in board_input])
            new_action_probs = np.rot90(np.flipud(action_probs.reshape(BOARD.row, BOARD.col)), i)
            extend_data.append((new_board_input,
                                np.flipud(new_action_probs).flatten(),
                                value))
            # flip horizontally
            new_board_input = np.array([np.fliplr(one_board_input) for one_board_input in new_board_input])
            new_action_probs = np.fliplr(new_action_probs)
            extend_data.append((new_board_input,
                                np.flipud(new_action_probs).flatten(),
                                value))
    return extend_data


class PolicyValueNet_from_junxiaosong(Network):
    """
    [junxiaosong](https://github.com/junxiaosong/AlphaZero_Gomoku)
    使用 @junxiaosong 的神经网络。
    Network by @junxiaosong.
    """

    def __init__(self, is_new_model, model_record_path="", is_in_thread=False):
        self.l2_const = 1e-4
        self.model_record_path = model_record_path
        self.is_in_thread = is_in_thread
        self.session = tf.compat.v1.Session(graph=tf.Graph())

        if is_new_model:
            self.create_net()
        else:
            if is_in_thread:
                with self.session.graph.as_default():
                    tf.compat.v1.keras.backend.set_session(self.session)
                    self.model = keras.models.load_model(self.model_record_path)
            else:
                self.model = keras.models.load_model(self.model_record_path)

    def __str__(self):
        return "PolicyValueNet_from_junxiaosong"

    def create_net(self):
        """
        创建策略价值网络。
        Create policy value net.
        """

        net = input_net = Input((4, BOARD.row, BOARD.col))
        net = layers.Conv2D(filters=32, kernel_size=(3, 3), padding="same", data_format="channels_last",
                            activation="relu", kernel_regularizer=regularizers.l2(self.l2_const))(net)
        net = layers.Conv2D(filters=64, kernel_size=(3, 3), padding="same", data_format="channels_last",
                            activation="relu", kernel_regularizer=regularizers.l2(self.l2_const))(net)
        net = layers.Conv2D(filters=128, kernel_size=(3, 3), padding="same", data_format="channels_last",
                            activation="relu", kernel_regularizer=regularizers.l2(self.l2_const))(net)

        policy_net = layers.Conv2D(filters=4, kernel_size=(1, 1), data_format="channels_last",
                                   activation="relu", kernel_regularizer=regularizers.l2(self.l2_const))(net)
        policy_net = layers.Flatten()(policy_net)
        policy_net = layers.Dense(BOARD.row * BOARD.col, activation="softmax",
                                  kernel_regularizer=regularizers.l2(self.l2_const))(policy_net)

        value_net = layers.Conv2D(filters=2, kernel_size=(1, 1), data_format="channels_last",
                                  activation="relu", kernel_regularizer=regularizers.l2(self.l2_const))(net)
        value_net = layers.Flatten()(value_net)
        value_net = layers.Dense(64, kernel_regularizer=regularizers.l2(self.l2_const))(value_net)
        value_net = layers.Dense(1, activation="tanh", kernel_regularizer=regularizers.l2(self.l2_const))(value_net)

        self.model = keras.Model(input_net, [policy_net, value_net])
        self.model.compile(optimizer=optimizers.Adam(),
                           loss=['categorical_crossentropy', 'mean_squared_error'])

    def board_to_xlabel(self, board):
        """
        将 Board 转化为作为神经网络输入的 x_label。
        Convert Board to x_label as input to the neural network.
        :param board: 棋盘。 The board.
        :return: x_label 需要输入到神经网络的 x_label。 Input of the neural network.
        """

        # board -> board_input (x_label)
        x_label = np.zeros((4, BOARD.row, BOARD.col))

        # 1: 我方棋子位置。 Position of our pieces.
        x_label[0][board.board == board.current_player] = 1

        # 2: 对方棋子位置。 Position of opponent pieces.
        x_label[1][board.board == -board.current_player] = 1

        # 3: 上次落子。 Last action.
        if board.last_action is not None:
            x_label[2][board.last_action[0], board.last_action[1]] = 1

        # 4: 当前 Player 是否先手。 Whether the current player is the first player.
        if board.current_player == BOARD.start_player:
            x_label[3][:, :] = 1

        # flip
        flipped_x_label = []
        for one_board in x_label:
            flipped_x_label.append(np.flipud(one_board))
        x_label = np.array(flipped_x_label)

        return x_label


    def predict(self, board):
        """
        对当前局面预测下一步的概率。
        Probability of predicting the next step for the current board.
        :param board: 当前局面。 Current board.
        :return: [(action, probability), value]
        """

        board_input = self.board_to_xlabel(board)
        board_input = board_input.reshape((-1, 4, BOARD.row, BOARD.col))

        if self.is_in_thread:
            # https://github.com/CLOXnu/Omega_Gomoku_AI/issues/1
            with self.session.graph.as_default():
                tf.compat.v1.keras.backend.set_session(self.session)
                probs, value = self.model.predict_on_batch(board_input)
        else:
            probs, value = self.model.predict_on_batch(board_input)

        probs = np.array(probs)
        probs = probs.reshape((BOARD.row, BOARD.col))

        action_probs = []
        for available_action in board.available_actions:
            action_probs.append((available_action, probs[available_action[0], available_action[1]]))

        return action_probs, value[0][0]

    def evaluate(self, x_label, y_label):
        """
        评估网络。
        Evaluate the network.
        :param x_label: 神经网络的输入 x_label。 Input of the neural network.
        :param y_label: 神经网络的输出 x_label。 Output of the neural network.
        :return: 网络的评估值。 Evaluation of the network.
        """

        board_input = np.array(x_label)

        probs, values = y_label
        probs_output = np.array(probs)
        values_output = np.array(values)

        return self.model.evaluate(board_input, [probs_output, values_output], batch_size=len(board_input), verbose=0)

    def get_entropy(self, x_label):
        """
        取得熵。
        Get the entropy.
        :param x_label: 神经网络的输入 x_label。 Input of the neural network.
        :return: 熵。 The entropy.
        """
        board_input = np.array(x_label)

        probs, _ = self.model.predict_on_batch(board_input)

        return -np.mean(np.sum(probs * np.log(probs + 1e-10), axis=1))
