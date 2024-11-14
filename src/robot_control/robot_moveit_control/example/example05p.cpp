#include <ros/ros.h>
#include <bits/stdc++.h>
#include <MoveitServer.h>
#include <robotTool.h>
#include <robot_msgs/Get_Board_State.h>
#include <robot_msgs/Hand_Catch.h>
#include <robot_msgs/NextMove.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <robot_msgs/ChessBoardState.h>

const int ROW = 10; // 棋盘大小
const int COL = 8;
const int NUM_STATES = 3; // 棋子的三种状态：空(em)，人类(human)，机器人(robot)
const double J_joint[6] = {0, -26, 77, 0, 109, 90};
// 定义模式的最大值
const int MAX_PATTERN = 32; // 5位模式，范围是 0 到 31
int degree = 0; //难度选择
int last_human_move_x = -1; // 人类上一步坐标
int last_human_move_y = -1;

// 使用数组存储模式对应的分数
int patternScores[MAX_PATTERN] = {0};

// 定义评分阈值
const int FIVE = 1000000;
const int FOUR = 10000;
const int THREE = 1000;
const int TWO = 200;
const int ONE = 1;
// 定义用于存储分数和深度的结构体
struct TranspositionEntry
{
    int score; // 分数
    int depth; // 搜索深度
};

// 表示棋盘上的一个位置
struct position
{
    int x; // 行
    int y; // 列

    // 定义比较操作符，用于 std::set 的去重和排序
    bool operator<(const position &other) const
    {
        if (x != other.x)
            return x < other.x;
        return y < other.y;
    }
};

// 棋盘状态的结构体
struct checkerboard
{
    int board[ROW][COL]; // 棋盘上的棋子状态
    int round; // 当前回合数
    checkerboard() : round(0)
    {
        // 初始化棋盘上的值为0
        for (int i = 0; i < ROW; ++i)
        {
            for (int j = 0; j < COL; ++j)
            {
                board[i][j] = 0;
            }
        }
    }
};

// 枚举定义棋子的三种状态
enum chess
{
    em = 0, // 空位置
    human = 1,  // 人类棋子（O）
    robot = -1  // 机器人棋子（X）
};

// Zobrist 哈希表，用于快速计算棋盘状态的唯一标识
std::vector<std::vector<std::vector<std::size_t>>> zobristTable(
    ROW, std::vector<std::vector<std::size_t>>(ROW, std::vector<std::size_t>(NUM_STATES)));

// 置换表，用于缓存搜索结果以优化计算
std::unordered_map<std::size_t, TranspositionEntry> transpositionTable;

// 机器人下棋类
class playRobot
{
private:
    ros::NodeHandle nh;                               // ROS节点句柄
    std::unique_ptr<MoveitServer> arm;                // Moveit 机械臂服务器
    robotTool tools;                                  // 机器人工具
    checkerboard cb;                                  // 当前棋盘状态
    ros::ServiceClient obj_detection, cube_detection, omega_AI; // 服务客户端，用于检测棋盘和抓取棋子
    std::vector<geometry_msgs::PoseStamped> poses;    // 棋子的位置姿态
    ros::Publisher boardStatePub;                     // 棋盘状态的发布者
    ros::Subscriber startGameSub, chooseDegreeSub;    // 订阅开始游戏和选择难度的界面按钮信号
    bool is_start = false;

public:
    playRobot(std::string planGroup)
    {
        // 初始化机械臂、服务客户端、设置机械臂速度、广告发布棋盘状态
        arm = std::make_unique<MoveitServer>(planGroup);
        cube_detection = nh.serviceClient<robot_msgs::Hand_Catch>("color_detect");
        obj_detection = nh.serviceClient<robot_msgs::Get_Board_State>("chessboard_detect");
        omega_AI = nh.serviceClient<robot_msgs::NextMove>("next_move");
        arm->setMaxVelocity(1, 1);
        boardStatePub = nh.advertise<robot_msgs::ChessBoardState>("/boardState", 10);
        startGameSub = nh.subscribe("/startGame", 10, &playRobot::subscribeCallback, this);
        chooseDegreeSub = nh.subscribe("/degree", 10, &playRobot::degreeCallback, this);
        poses.clear(); // 清空位置姿态
    }

    int evaluateLine(const checkerboard &state, int startX, int startY, int dx, int dy, int length);
    bool checkDirection(const checkerboard &state, int player, int startX, int startY, int dx, int dy);
    std::vector<position> getPossibleMoves(const checkerboard &state, int player);
    bool isWinningMove(const checkerboard &state, int player);
    int evaluate(const checkerboard &state);
    int evaluatePosition(const checkerboard &state, int player, int x, int y);
    int minimax(checkerboard &state, std::vector<position> possiblemoves, int depth, bool isMax, int alpha, int beta);
    int findBestMove(checkerboard &state, int &k1, int &k2, int degree, int &l1, int &l2);

    bool move(geometry_msgs::Pose pose);
    bool searchBoard();
    bool getchess();
    bool Victory();
    bool startGame();
    void subscribeCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        is_start = msg->data;
    }
    void degreeCallback(const std_msgs::Int32::ConstPtr &msg)
    {
        degree = msg->data;
    }

    ~playRobot() {} // 析构函数
};

// 初始化Zobrist表
void initializeZobristTable()
{
    std::mt19937_64 rng(std::random_device{}());
    std::uniform_int_distribution<std::size_t> dist;

    for (int i = 0; i < ROW; ++i)
    {
        for (int j = 0; j < COL; ++j)
        {
            for (int k = 0; k < NUM_STATES; ++k)
            {
                zobristTable[i][j][k] = dist(rng);
            }
        }
    }
}
// 初始化数组
void initializePatternScores()
{
    patternScores[0b10000] = ONE;
    patternScores[0b01000] = ONE;
    patternScores[0b00100] = ONE;
    patternScores[0b00010] = ONE;
    patternScores[0b00001] = ONE;

    patternScores[0b11000] = TWO;
    patternScores[0b01100] = TWO;
    patternScores[0b00110] = TWO;
    patternScores[0b00011] = TWO;
    patternScores[0b10100] = 3*TWO;
    patternScores[0b01010] = 3*TWO;
    patternScores[0b00101] = 3*TWO;
    patternScores[0b10010] = TWO;
    patternScores[0b01001] = TWO;
    patternScores[0b10001] = TWO;

    patternScores[0b11100] = THREE;
    patternScores[0b01110] = 6*THREE;
    patternScores[0b00111] = THREE;
    patternScores[0b11010] = 3*THREE;
    patternScores[0b01101] = 3*THREE;
    patternScores[0b10110] = 3*THREE;
    patternScores[0b01011] = 3*THREE;
    patternScores[0b11001] = THREE;
    patternScores[0b10011] = THREE;
    patternScores[0b10101] = THREE;

    patternScores[0b11110] = FOUR;
    patternScores[0b11101] = FOUR;
    patternScores[0b11011] = FOUR;
    patternScores[0b10111] = FOUR;
    patternScores[0b01111] = FOUR;

    patternScores[0b11111] = FIVE;
}
// 计算棋盘状态的哈希值
std::size_t computeHash(const checkerboard &state)
{
    std::size_t hash = 0;
    for (int i = 0; i < ROW; ++i)
    {
        for (int j = 0; j < COL; ++j)
        {
            int piece = state.board[i][j] + 1; // 偏移 -1, 0, 1 到 0, 1, 2
            hash ^= zobristTable[i][j][piece];
        }
    }
    return hash;
}

// 打印棋盘状态的函数
void printBoard(const checkerboard &state)
{
    int width = 4; // 设置每列的宽度

    std::cout << std::setw(width) << " "; // 设置初始空格
    for (int i = 0; i < COL; ++i)
        std::cout << std::setw(width) << i; // 设置每个数字的宽度
    std::cout << "\n";

    for (int i = 0; i < ROW; ++i)
    {
        std::cout << std::setw(width) << i; // 输出行号
        for (int j = 0; j < COL; ++j)
        {
            char symbol = state.board[i][j] == human ? 'O' : (state.board[i][j] == robot ? 'X' : '.');
            std::cout << std::setw(width) << symbol; // 设置每个符号的宽度
        }
        std::cout << "\n";
    }
}

void reverseBoard(checkerboard &cb)
{
    for (int i = 0; i < ROW; ++i)
    {
        for (int j = 0; j < COL; ++j)
        {
            // 检查棋盘上的棋子，1变-1，-1变1
            if (cb.board[i][j] == 1)
            {
                cb.board[i][j] = -1;
            }
            else if (cb.board[i][j] == -1)
            {
                cb.board[i][j] = 1;
            }
        }
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "example05");
    ros::NodeHandle nh;
    playRobot arm("rokae_arm");
    initializeZobristTable();
    initializePatternScores();
    arm.startGame();
    // checkerboard board2;
    // printBoard(board2);
    // arm.evaluate(board2);
    // int k1, k2, l1, l2;
    // l1 = last_human_move_x;
    // l2 = last_human_move_y;
    // bool c = true;

    // while (true)
    // {
    //     // // 每次循环开始时处理消息
    //     // ros::spinOnce();
        
    //     std::cout << "输入：";
    //     std::cin >> k1 >> k2;
    //     board2.board[k1][k2] = human;
    //     // board2.board[5][3] = human;
    //     l1 = k1;
    //     l2 = k2;
    //     arm.findBestMove(board2, k1, k2, degree, l1, l2);
    //     board2.board[k1][k2] = robot;
    //     printBoard(board2);
        // reverseBoard(board2);
        // arm.findBestMove(board2, k1, k2, 0, l1, l2);
        // board2.board[k1][k2] = robot;
        // l1 = k1;
        // l2 = k2;
        // reverseBoard(board2);
        
    //     for (int i = 0; i < ROW; ++i)
    //     {
    //         for (int j = 0; j < COL; ++j)
    //         {
    //             if (board2.board[i][j] == -1)
    //             {
    //                 c = false;
    //                 break;
    //             }
    //         }
    //         if (!c)
    //             break;
    //     }
    //     if (arm.isWinningMove(board2, robot) || arm.isWinningMove(board2, human) || c)
    //     {
    //         checkerboard board;
    //         board2 = board;
    //         printBoard(board2);
    //     }
    // }
    // return 0;
}

// 检查某一方向上是否形成五子连珠
bool playRobot::checkDirection(const checkerboard &state, int player, int startX, int startY, int dx, int dy)
{
    int count = 0;
    for (int i = 0; i < 5; ++i)
    {
        int x = startX + i * dx;
        int y = startY + i * dy;
        if (x < 0 || x >= ROW || y < 0 || y >= COL)
            return false;
        if (state.board[x][y] == player)
        {
            count++;
        }
        else
        {
            break;
        }
    }
    return count == 5;
}
// 判断当前玩家是否赢得比赛
bool playRobot::isWinningMove(const checkerboard &state, int player)
{
    // 横向、纵向、对角线检测
    for (int i = 0; i < ROW; ++i)
    {
        for (int j = 0; j < COL; ++j)
        {
            if (state.board[i][j] == player)
            {
                if (checkDirection(state, player, i, j, 1, 0))
                    return true; // 横向
                if (checkDirection(state, player, i, j, 0, 1))
                    return true; // 纵向
                if (checkDirection(state, player, i, j, 1, 1))
                    return true; // 正对角线
                if (checkDirection(state, player, i, j, 1, -1))
                    return true; // 反对角线
            }
        }
    }
    return false;
}

// 辅助函数：检查某个位置是否有邻居
bool hasNeighbor(const checkerboard &state, int x, int y, int distance = 1)
{
    for (int i = -distance; i <= distance; ++i)
    {
        for (int j = -distance; j <= distance; ++j)
        {
            if (i == 0 && j == 0)
                continue;
            int nx = x + i;
            int ny = y + j;
            if (nx >= 0 && nx < ROW && ny >= 0 && ny < COL)
            {
                if (state.board[nx][ny] != em)
                {
                    return true;
                }
            }
        }
    }
    return false;
}
std::vector<position> playRobot::getPossibleMoves(const checkerboard &state, int player)
{
    std::vector<position> comfives, humfives, comFours, humFours;
    std::vector<position> Threes, HThrees;
    std::vector<position> Twos, neighbors;
    int opponent = (player == human) ? robot : human;
    for (int i = 0; i < ROW; ++i)
    {
        for (int j = 0; j < COL; ++j)
        {
            if (state.board[i][j] == em && hasNeighbor(state, i, j, 1))
            {
                checkerboard newstate = state;
                newstate.board[i][j] = player;
                int playerScore = evaluatePosition(newstate, player, i, j);
                newstate.board[i][j] = opponent;
                int opponentScore = evaluatePosition(newstate, opponent, i, j);
                newstate.board[i][j] = -1;
                position p = {i, j};

                if (playerScore >= FIVE)
                {
                    comfives.push_back(p);
                }
                else if (opponentScore >= FIVE)
                {
                    humfives.push_back(p);
                }
                else if (playerScore >= FOUR)
                {
                    comFours.push_back(p);
                }
                else if (opponentScore >= FOUR)
                {
                    humFours.push_back(p);
                }
                else if (playerScore >= 3 * THREE || opponentScore >= 3 * THREE)
                {
                    HThrees.push_back(p);
                    Threes.push_back(p);
                }
                else if (playerScore >= THREE || opponentScore >= THREE)
                {
                    Threes.push_back(p);
                }
                else if (playerScore >= TWO || opponentScore >= TWO)
                {
                    Twos.push_back(p);
                }
                else
                {
                    neighbors.push_back(p);
                }
            }
        }
    }

    // 根据优先级返回合适的走法
    if (!comfives.empty())
        return comfives;
    if (!humfives.empty())
        return humfives;
    // if (!HThrees.empty())
    // {
    //     return HThrees;
    // }
    // if (!comFours.empty() && humFours.empty())
    // {
    //     comFours.insert(comFours.end(), HThrees.begin(), HThrees.end());
    //     return comFours;
    // }
    // if (comFours.empty() && !humFours.empty())
    // {
    //     humFours.insert(humFours.end(), HThrees.begin(), HThrees.end());
    //     return humFours;
    // }
    std::vector<position> fours;
    if (!comFours.empty() && !humFours.empty())
    {
        fours.insert(fours.end(), humFours.begin(), humFours.end());
        fours.insert(fours.end(), comFours.begin(), comFours.end());
        fours.insert(fours.end(), HThrees.begin(), HThrees.end());
        return fours;
    }
    std::vector<position> result;
    // if (HThrees.size() >= 2)
    // {
    //     return HThrees;
    // }
    result.insert(result.end(), humFours.begin(), humFours.end());
    result.insert(result.end(), comFours.begin(), comFours.end());
    result.insert(result.end(), Threes.begin(), Threes.end());
    result.insert(result.end(), Twos.begin(), Twos.end());
    // result.insert(result.end(), neighbors.begin(), neighbors.end());
    // 如果低分走法过多，限制返回的数量
    const int countLimit = 30; // 可以通过配置传递
    if (result.size() > countLimit)
    {
        result.resize(countLimit);
    }

    return result.empty() ? neighbors : result;
}
// 评估棋盘上某个点的得分
int playRobot::evaluatePosition(const checkerboard &state, int player, int x, int y)
{
    int totalScore = 0;

    // 四个方向：水平、垂直、正对角线、反对角线
    int directions[4][2] = {{1, 0}, {0, 1}, {1, 1}, {1, -1}};

    for (auto &dir : directions)
    {
        for (int i = -4; i <= 0; ++i)
        { // 向左或向上延伸4格
            int humanPattern = 0, robotPattern = 0, count = 0;

            for (int j = 0; j < 5; ++j)
            {
                int nx = x + (i + j) * dir[0];
                int ny = y + (i + j) * dir[1];

                if (nx >= 0 && nx < ROW && ny >= 0 && ny < COL)
                {
                    humanPattern = (humanPattern << 1) | (state.board[nx][ny] == human ? 1 : 0);
                    robotPattern = (robotPattern << 1) | (state.board[nx][ny] == robot ? 1 : 0);
                    count++;
                }
                else
                {
                    humanPattern = robotPattern = -1; // 越界时无效
                    break;
                }
            }
            if (count != 5)
            {
                continue;
            }
            if (humanPattern == 0 && robotPattern == 0)
            {
                continue;
            }
            if (humanPattern == 0 && player == robot)
            {
                totalScore += patternScores[robotPattern];
                // if (robotPattern == 0b01110)
                // {
                //     totalScore *= 2;
                // }
            }
            if (robotPattern == 0 && player == human)
            {
                totalScore += patternScores[humanPattern];
                // if (humanPattern == 0b01110)
                // {
                //     totalScore *= 2;
                // }
            }
        }
    }

    return totalScore;
}

int playRobot::evaluate(const checkerboard &state)
{
    int score = 0;
    // 遍历每一行、每一列、每一条对角线
    for (int i = 0; i < ROW; ++i)
    {
        score += evaluateLine(state, i, 0, 0, 1, COL - 4); // 横线
        score += evaluateLine(state, 0, i, 1, 0, ROW - 4); // 竖线
        if (i < ROW - 3)
        {
            if (i < COL)
            {
                score += evaluateLine(state, 0, i, 1, 1, COL - 4); // 正对角线
                score += evaluateLine(state, 0, COL - i - 1, 1, -1, COL - 4); // 反对角线
            }
            score += evaluateLine(state, i, 0, 1, 1, COL - 4); // 正对角线
            score += evaluateLine(state, i, COL - 1, 1, -1, COL - 4); // 反对角线
        }
    }

    return score;
}

// 评估一条线的得分
int playRobot::evaluateLine(const checkerboard &state, int startX, int startY, int dx, int dy, int length)
{
    int humanScore = 0, robotScore = 0;
    // checkerboard newstate = state;
    for (int i = 0; i < length; ++i)
    { // 评估的起点
        int humanPattern = 0, robotPattern = 0, count = 0;
        int humancount = 0;
        // 表示评估线的长度
        for (int j = 0; j < 5; ++j)
        {
            int x = startX + (i + j) * dx;
            int y = startY + (i + j) * dy;
         
            if (x < 0 || x >= ROW || y < 0 || y >= COL)
                break;
            // newstate.board[x][y] = robot;
            if (state.board[x][y] == human)
            {
                humancount++;
            }
            humanPattern = (humanPattern << 1) | (state.board[x][y] == human ? 1 : 0);
            robotPattern = (robotPattern << 1) | (state.board[x][y] == robot ? 1 : 0);
            count++;
        }
        if (count != 5)
        {
            continue;
        }
        if (humanPattern == 0 && robotPattern == 0)
        {
            continue;
        }
        if (humanPattern == 0)
        {
            robotScore += patternScores[robotPattern];
        }
        if (robotPattern == 0)
        {
            // 直接访问数组以获取分数
            humanScore += (patternScores[humanPattern]);
            // if (humancount > 2)
            // {
            // humanScore *= humancount;
            // }
        }
    }
    // printBoard(newstate);
    return ((int)(2*robotScore) - 5*humanScore);
}

std::mutex transpositionTableMutex;

int playRobot::minimax(checkerboard &state, std::vector<position> possiblemoves, int depth, bool isMax, int alpha, int beta)
{
    std::size_t stateHash = computeHash(state);
    {
        // 加锁保护 transpositionTable 的访问
        std::lock_guard<std::mutex> lock(transpositionTableMutex);
        auto it = transpositionTable.find(stateHash);
        if (it != transpositionTable.end())
        {
            const TranspositionEntry &entry = it->second;
            if (entry.depth >= depth)
            {
                return entry.score; // 剪枝：返回缓存的结果
            }
        }
    }
    std::vector<position> moves;
    int score = evaluate(state);
    // 终止条件
    if (depth >= 5 || isWinningMove(state, human) || isWinningMove(state, robot) || possiblemoves.empty())
    {
        return score;
    }
    bool hasValidMove = false;
    for (const auto &move : possiblemoves)
    {
        if (state.board[move.x][move.y] == em)
        {
            hasValidMove = true;
            break;
        }
    }

    if (!hasValidMove)
    {
        // 如果没有可行走法，提前返回当前评估分数
        return score;
    }
    int best;

    if (isMax)
    {
        best = std::numeric_limits<int>::min();
        for (const auto &move : possiblemoves)
        {
            if (state.board[move.x][move.y] == em)
            {
                state.board[move.x][move.y] = robot;
                moves = getPossibleMoves(state, human);
                best = std::max(best, minimax(state, moves, depth + 1, !isMax, alpha, beta));
                state.board[move.x][move.y] = em;
                alpha = std::max(alpha, best);
                if (beta <= alpha)
                    break; // Beta 剪枝
            }
        }
    }
    else
    {
        best = std::numeric_limits<int>::max();
        for (const auto &move : possiblemoves)
        {
            if (state.board[move.x][move.y] == em)
            {
                state.board[move.x][move.y] = human;
                moves = getPossibleMoves(state, robot);
                best = std::min(best, minimax(state, moves, depth + 1, !isMax, alpha, beta));
                state.board[move.x][move.y] = em;
                beta = std::min(beta, best);

                if (beta <= alpha)
                    break; // Alpha 剪枝
            }
        }
    }

    {
        // 加锁保护 transpositionTable 的更新
        std::lock_guard<std::mutex> lock(transpositionTableMutex);
        transpositionTable[stateHash] = {best, depth};
    }

    return best;
}

int playRobot::findBestMove(checkerboard &state, int &k1, int &k2, int degree, int &l1, int &l2)
{
    if (degree == 1)
    {
        // 准备请求数据
        robot_msgs::NextMove srv;
        srv.request.last_chess_x = l1;
        srv.request.last_chess_y = l2;
        std::vector<int32_t> board_state;
        ROS_INFO("human last move: x=%d, y=%d", l1, l2);
        
        // 将当前棋盘状态转换为一维数组
        for (int i = 0; i < ROW; ++i)
        {
            for (int j = 0; j < COL; ++j)
            {
                board_state.push_back(state.board[i][j]);
            }
        }
        srv.request.board_state = board_state;

        // 调用服务端获取下一步棋
        if (omega_AI.call(srv))
        {
            // 获取AI计算的最佳落子点
            k1 = srv.response.x;
            k2 = srv.response.y;
            ROS_INFO("AI calculated next move: x=%d, y=%d", k1, k2);
            return k1 * COL + k2;
        }
        else
        {
            ROS_ERROR("Failed to call next_move service");
            return -1;
        }
    }
    else
    {
        // 优先考虑中心区域
        int bestValue = std::numeric_limits<int>::min();
        int bestMove = -1;
        int moveVal;
        int move;
        // int depth=4;
        std::vector<position> moves = getPossibleMoves(state, robot);
        std::vector<std::future<std::pair<int, int>>> futures;
        // if (moves.size() > 0 && moves.size() < 5)
        // {
        //     depth = 10;
        // }
        // if (moves.size() > 0 && moves.size() < 10)
        // {
        //     depth = 8;
        // }
        // 异步计算每个可能走法的分数
        for (const auto &pos : moves)
        {
            futures.push_back(std::async(std::launch::async, [this, pos, stateCopy = state, movesCopy = moves]() -> std::pair<int, int>
                                        {
                                            checkerboard localState = stateCopy; // 独立的局部副本，避免并发问题
                                            int i = pos.x;
                                            int j = pos.y;
                                            if (i >= 0 && i < ROW && j >= 0 && j < COL && localState.board[i][j] == em)
                                            {
                                                localState.board[i][j] = robot;

                                                int moveVal = minimax(localState, movesCopy, 0, false, std::numeric_limits<int>::min(), std::numeric_limits<int>::max());
                                                localState.board[i][j] = em;
                                                return {moveVal, i * COL + j};
                                            }
                                            return {std::numeric_limits<int>::min(), -1}; // 无效的结果
                                        }));
        }

        // 收集所有线程的结果，并计算最佳走法
        for (auto &f : futures)
        {
            auto result = f.get();
            if (result.second != -1)
            {
                moveVal = result.first;
                move = result.second;

                if (moveVal > bestValue)
                {
                    bestValue = moveVal;
                    bestMove = move;
                    k1 = bestMove / COL;
                    k2 = bestMove % COL;
                }
            }
            else
            {
                ROS_INFO("没有得到正确解析");
            }
        }
        ROS_INFO("i:%d j:%d", k1, k2);
        return bestMove;      
    }
}

bool isround0 = true;
bool playRobot::searchBoard()
{
    ros::Duration(2).sleep();
    try
    {
        robot_msgs::Get_Board_State states;
        robot_msgs::ChessBoardState msg;
        states.request.run = true;
        // 根据是否有位置信息设置请求
        states.request.getpositions = poses.empty(); 
        
        if (obj_detection.call(states) && !states.response.board.empty())
        {
            states.request.getpositions = false;
        }
        else
        {
            states.request.getpositions = true;
        }
        if (obj_detection.call(states) && !states.response.board.empty())
        {
            if ((states.response.round >= cb.round && states.response.round < cb.round + 2))
            {
                int change_count = 0;
                if (states.response.round != 0)
                {
                    for (int i = 0; i < ROW; ++i)
                    {
                        for (int j = 0; j < COL; ++j)
                        {
                            int index=i * COL + j;
                            if (cb.board[i][j] != states.response.board[index])
                            {
                                change_count++;
                            }
                        }
                    }
                    // 如果变化的个数大于1，返回false以触发重新扫描
                    if (change_count > 1)
                    {
                        ROS_WARN("Detected more than one change, rescan needed.");
                        return false;
                    }
                }

                for (int i = 0; i < ROW; ++i)
                {
                    for (int j = 0; j < COL; ++j)
                    {
                        int index=i * COL + j;
                        cb.board[i][j] = states.response.board[index];
                    }
                }
                cb.round = states.response.round;
                msg.iserror = false;
                msg.board=states.response.board;
                msg.turn = cb.round;
                msg.iswin = 0;
                boardStatePub.publish(msg);
                ROS_INFO("ROUND: %d ", cb.round);
                if (poses.empty())
                    poses = states.response.positions;
                printBoard(cb);
                return true;
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return false;
}

bool playRobot::move(geometry_msgs::Pose pose)
{
    pose.position.z = 0;
    pose = tools.transPose(pose, "tool", "xMate3_link6");
    pose = arm->setPoint(std::vector<double>{pose.position.x, pose.position.y, pose.position.z, 0, tools.degreesToRadians(180), 0});
    pose = tools.calculateTargetPose(pose, arm->setPoint(std::vector<double>{-0.004, -0.008, 0, 0, 0, 0}));
    tools.publishStaticTFwithRot(pose, "board");
    arm->move_l(tools.moveFromPose(pose, -0.05));
    arm->move_l(tools.moveFromPose(pose, -0.007));
    arm->Set_Tool_DO(1, true);
    arm->move_l(tools.moveFromPose(pose, -0.05));
    arm->move_j(std::vector<double>{tools.degreesToRadians(J_joint[0]), tools.degreesToRadians(J_joint[1]), tools.degreesToRadians(J_joint[2]),
                                    tools.degreesToRadians(J_joint[3]), tools.degreesToRadians(J_joint[4]), tools.degreesToRadians(J_joint[5])});
    return true;
}

bool playRobot::getchess()
{
    // arm->move_j(std::vector<double>{tools.degreesToRadians(90), tools.degreesToRadians(-20), tools.degreesToRadians(-60),
    //                                 tools.degreesToRadians(0), tools.degreesToRadians(-100), tools.degreesToRadians(0)});
    ROS_INFO("Get Chess");
    robot_msgs::Hand_Catch ct;
    ct.request.name = "green";
    ct.request.run = true;
    bool success;
    int i = 0;
    while ((true))
    {
        if (cube_detection.call(ct) && !ct.response.positions.empty())
        {
            success = false;
            while (!success && i < ct.response.positions.size())
            {
                geometry_msgs::Pose p = ct.response.positions[i].pose;
                p = arm->setPoint(std::vector<double>{p.position.x, p.position.y, p.position.z, 0, tools.degreesToRadians(180), tools.degreesToRadians(-90)});
                p = tools.transPose(p, "tool", "xMate3_link6");
                // p = arm->setPoint(p.position.x, p.position.y, p.position.z);
                tools.publishStaticTFwithRot(p, "chess");
                success = arm->move_l(tools.calculateTargetPose(p, arm->setPoint(std::vector<double>{-0.10, 0, -0.35, 0, 0, 0})));
                i++;
            }
            if (success)
            {
                i = 0;
                success = false;
                ros::Duration(2).sleep();
                while (true)
                {
                    if (cube_detection.call(ct) && !ct.response.positions.empty())
                    {

                        while (!success && i < ct.response.positions.size())
                        {
                            geometry_msgs::Pose p = ct.response.positions[i].pose;
                            p.position.z = 0.005;
                            // p = arm->setPoint(p.position.x, p.position.y, p.position.z);
                            p = tools.transPose(p, "tool", "xMate3_link6");
                            // p = tools.calculateTargetPose(p, arm->setPoint(std::vector<double>{0, -0.005, 0, 0, 0, 0}));
                            tools.publishStaticTFwithRot(p, "chess");
                            success = arm->move_l(tools.moveFromPose(p, -0.10));
                            success = arm->move_l(p, success);
                            if (success)
                                arm->Set_Tool_DO(1, false);
                            success = arm->move_l(tools.moveFromPose(p, -0.10), success);
                            i++;
                        }
                        break;
                    }
                    if (success)
                        break;
                }
            }

            // arm->move_j(std::vector<double>{tools.degreesToRadians(0), tools.degreesToRadians(0), tools.degreesToRadians(75),
            //                                 tools.degreesToRadians(0), tools.degreesToRadians(105), tools.degreesToRadians(90)});
            if (success)
                break;
        }
    }

    return success;
}

bool playRobot::Victory()
{
    ROS_INFO("ROBOT Victroy !!!");
    arm->move_j(std::vector<double>{tools.degreesToRadians(0), 0, tools.degreesToRadians(100),
                                    0, tools.degreesToRadians(-100), 0});
    // arm->setMaxVelocity(0.6);
    arm->move_j(std::vector<double>{tools.degreesToRadians(0), 0, tools.degreesToRadians(100),
                                    tools.degreesToRadians(-40), tools.degreesToRadians(-100), 0});
    arm->move_j(std::vector<double>{tools.degreesToRadians(0), 0, tools.degreesToRadians(100),
                                    tools.degreesToRadians(40), tools.degreesToRadians(-100), 0});
    arm->move_j(std::vector<double>{tools.degreesToRadians(0), 0, tools.degreesToRadians(100),
                                    tools.degreesToRadians(-40), tools.degreesToRadians(-100), 0});
    arm->move_j(std::vector<double>{tools.degreesToRadians(0), 0, tools.degreesToRadians(100),
                                    0, tools.degreesToRadians(-100), 0});
    arm->move_j(std::vector<double>{tools.degreesToRadians(J_joint[0]), tools.degreesToRadians(J_joint[1]), tools.degreesToRadians(J_joint[2]),
                                    tools.degreesToRadians(J_joint[3]), tools.degreesToRadians(J_joint[4]), tools.degreesToRadians(J_joint[5])});
    robot_msgs::ChessBoardState states;
    for (int i = 0; i < ROW; i++)
        for (int j = 0; j < COL; j++)
            states.board.push_back(cb.board[i][j]);
    states.turn = cb.round;
    states.iswin = 0;
    boardStatePub.publish(states);
    return true;
}

bool playRobot::startGame()
{

    arm->move_j(std::vector<double>{tools.degreesToRadians(J_joint[0]), tools.degreesToRadians(J_joint[1]), tools.degreesToRadians(J_joint[2]),
                                    tools.degreesToRadians(J_joint[3]), tools.degreesToRadians(J_joint[4]), tools.degreesToRadians(J_joint[5])});
    ros::Duration(1).sleep();
    while (ros::ok())
    {

        if (is_start)
        {
            robot_msgs::ChessBoardState states;
            ROS_INFO("Start Game!");
            isround0 = true;

            while (ros::ok())
            {
                if (!is_start)
                    break;

                if (searchBoard() && cb.round % 2 != 0)
                {

                    for (int i = 0; i < ROW; i++)
                        for (int j = 0; j < COL; j++)
                            states.board.push_back(cb.board[i][j]);
                    states.turn = cb.round;
                    states.iswin = 0;
                    if (isWinningMove(cb, robot))
                    {
                        states.iswin = 1;
                        boardStatePub.publish(states);
                        is_start = false;
                        Victory();
                        break;
                    }
                    if (isWinningMove(cb, human))
                    {
                        states.iswin = 2;
                        boardStatePub.publish(states);
                        is_start = false;
                        Victory();
                        break;
                    }
                    ROS_INFO("Robot's trun");
                    int k1, k2, l1, l2;
                    l1 = last_human_move_x;
                    l2 = last_human_move_y;
                    int p = findBestMove(cb, k1, k2, degree, l1, l2);
                    ROS_INFO("p:%d ", p);
                    getchess();
                    move(poses[p].pose);
                    // cb.board[k1][k2] == robot;
                    // cb.round++;
                }
                else
                {
                    if (isWinningMove(cb, robot))
                    {
                        states.iswin = 1;
                        boardStatePub.publish(states);
                        is_start = false;
                        Victory();
                        break;
                    }
                    if (isWinningMove(cb, human))
                    {
                        states.iswin = 2;
                        boardStatePub.publish(states);
                        is_start = false;
                        Victory();
                        break;
                    }
                }
            }
        }
        else
        {
            checkerboard newb;
            cb = newb;
            poses.clear();
        }
    }
    return true;
}
