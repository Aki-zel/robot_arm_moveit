#include <ros/ros.h>
#include <bits/stdc++.h>
#include <MoveitServer.h>
#include <robotTool.h>
#include <robot_msgs/Get_Board_State.h>
#include <robot_msgs/Hand_Catch.h>
#include <std_msgs/Int32MultiArray.h>
const int BOARD_SIZE = 9;
const int NUM_STATES = 3; // 0: em, 1: human, 2: robot
// 定义模式的最大值
const int MAX_PATTERN = 32; // 5位模式，范围是 0 到 31
// 使用数组存储模式对应的分数
int patternScores[MAX_PATTERN] = {0};
// 定义评分阈值
const int FIVE = 10000;
const int FOUR = 1000;
const int THREE = 100;
const int TWO = 10;
// 定义用于存储分数和深度的结构体
struct TranspositionEntry
{
    int score;
    int depth;
};
struct position
{
    int x;
    int y;
    // 定义比较操作符，用于 std::set 的去重和排序
    bool operator<(const position &other) const
    {
        if (x != other.x)
            return x < other.x;
        return y < other.y;
    }
};
struct checkerboard
{
    int board[BOARD_SIZE][BOARD_SIZE];
    int round; // 回合数
};
enum chess
{
    em = -1,
    human, // O
    robot  // X
};
// Zobrist 表
std::vector<std::vector<std::vector<std::size_t>>> zobristTable(BOARD_SIZE, std::vector<std::vector<std::size_t>>(BOARD_SIZE, std::vector<std::size_t>(NUM_STATES)));
// 置换表
std::unordered_map<std::size_t, TranspositionEntry> transpositionTable;
class playRobot
{
private:
    ros::NodeHandle nh;
    std::unique_ptr<MoveitServer> arm;
    robotTool tools;
    checkerboard cb;
    ros::ServiceClient obj_detection, cube_detection;
    std::vector<geometry_msgs::PoseStamped> poses;
    ros::Publisher boardStatePub;

public:
    playRobot(std::string planGroup)
    {
        arm = std::make_unique<MoveitServer>(planGroup);
        cube_detection = nh.serviceClient<robot_msgs::Hand_Catch>("color_detect");
        obj_detection = nh.serviceClient<robot_msgs::Get_Board_State>("chessboard_detect");
        arm->setMaxVelocity(0.8, 0.8);
        boardStatePub = nh.advertise<std_msgs::Int32MultiArray>("/boardState", 10);
        poses.clear();
    }

    int evaluateLine(const checkerboard &state, int startX, int startY, int dx, int dy);
    bool checkDirection(const checkerboard &state, int player, int startX, int startY, int dx, int dy);
    double calculateDistance(int x1, int y1, int x2, int y2);
    std::vector<position> getPossibleMoves(const checkerboard &state, int player);
    bool isWinningMove(const checkerboard &state, int player);
    int evaluate(const checkerboard &state);
    int evaluatePosition(const checkerboard &state, int player, int x, int y);
    int minimax(checkerboard &state, std::vector<position> possiblemoves, int depth, bool isMax, int alpha, int beta);
    int findBestMove(checkerboard &state, int &k1, int &k2);

    bool move(geometry_msgs::Pose pose);
    bool searchBoard();
    bool getchess();
    bool Victory();
    bool startGame();
    ~playRobot()
    {
    }
};

// 初始化 Zobrist 表
void initializeZobristTable()
{
    std::mt19937_64 rng(std::random_device{}());
    std::uniform_int_distribution<std::size_t> dist;

    for (int i = 0; i < BOARD_SIZE; ++i)
    {
        for (int j = 0; j < BOARD_SIZE; ++j)
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
    patternScores[0b10000] = 10;
    patternScores[0b01000] = 10;
    patternScores[0b00100] = 10;
    patternScores[0b00010] = 10;
    patternScores[0b00001] = 10;

    patternScores[0b11000] = 100;
    patternScores[0b01100] = 100;
    patternScores[0b00110] = 100;
    patternScores[0b00011] = 100;
    patternScores[0b10100] = 100;
    patternScores[0b01010] = 100;
    patternScores[0b00101] = 100;
    patternScores[0b10010] = 100;
    patternScores[0b01001] = 100;
    patternScores[0b10001] = 100;

    patternScores[0b11100] = 1000;
    patternScores[0b01110] = 1000;
    patternScores[0b00111] = 1000;
    patternScores[0b11010] = 1000;
    patternScores[0b01101] = 1000;
    patternScores[0b10110] = 1000;
    patternScores[0b01011] = 1000;
    patternScores[0b11001] = 1000;
    patternScores[0b10011] = 1000;
    patternScores[0b10101] = 1000;

    patternScores[0b11110] = 10000;
    patternScores[0b11101] = 10000;
    patternScores[0b11011] = 10000;
    patternScores[0b10111] = 10000;
    patternScores[0b01111] = 10000;

    patternScores[0b11111] = 1000000;
}
// 计算棋盘状态的哈希值
std::size_t computeHash(const checkerboard &state)
{
    std::size_t hash = 0;
    for (int i = 0; i < BOARD_SIZE; ++i)
    {
        for (int j = 0; j < BOARD_SIZE; ++j)
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
    std::cout << "  ";
    for (int i = 0; i < BOARD_SIZE; ++i)
        std::cout << i << " ";
    std::cout << "\n";

    for (int i = 0; i < BOARD_SIZE; ++i)
    {
        std::cout << i << " ";
        for (int j = 0; j < BOARD_SIZE; ++j)
        {
            char symbol = state.board[i][j] == human ? 'O' : (state.board[i][j] == robot ? 'X' : '.');
            std::cout << symbol << " ";
        }
        std::cout << "\n";
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
    // checkerboard board2 = {{{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    //                         {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    //                         {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    //                         {-1, -1, -1, -1, 1, -1, -1, -1, -1, -1},
    //                         {-1, -1, -1, 0, 0, 0, -1, -1, -1, -1},
    //                         {-1, -1, -1, -1, 0, 1, 1, -1, -1, -1},
    //                         {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    //                         {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    //                         {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    //                         {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
    //                        1};
    // printBoard(board2);
    // int k1, k2;
    // while (true)
    // {
    //     arm.findBestMove(board2, k1, k2);
    //     board2.board[k1][k2] = robot;
    //     printBoard(board2);
    //     // for (int i = 0; i < BOARD_SIZE; ++i)
    //     // {
    //     //     for (int j = 0; j < BOARD_SIZE; ++j)
    //     //     {
    //     //         if (board2.board[i][j] == 0)
    //     //         {
    //     //             board2.board[i][j] = 1;
    //     //         }
    //     //         else if (board2.board[i][j] == 1)
    //     //         {
    //     //             board2.board[i][j] = 0;
    //     //         }
    //     //         // -1 保持不变，不需处理
    //     //     }
    //     // }
    //     // arm.findBestMove(board2, k1, k2);
    //     std::cout<<"输入：";
    //     std::cin>>k1>>k2;
    //     board2.board[k1][k2] = human;
    //     // for (int i = 0; i < BOARD_SIZE; ++i)
    //     // {
    //     //     for (int j = 0; j < BOARD_SIZE; ++j)
    //     //     {
    //     //         if (board2.board[i][j] == 0)
    //     //         {
    //     //             board2.board[i][j] = 1;
    //     //         }
    //     //         else if (board2.board[i][j] == 1)
    //     //         {
    //     //             board2.board[i][j] = 0;
    //     //         }
    //     //     }
    //     // }
    //     printBoard(board2);
    //     bool c = true;
    //     for (int i = 0; i < BOARD_SIZE; ++i)
    //     {
    //         for (int j = 0; j < BOARD_SIZE; ++j)
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

    //         board2 = {{{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    //                    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    //                    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    //                    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    //                    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    //                    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    //                    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    //                    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    //                    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    //                    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1}},
    //                   1};
    //         printBoard(board2);
    //         std::cout << "输入下棋位置：";
    //         std::cin >> k1 >> k2;
    //         board2.board[k1][k2] = human;
    //         printBoard(board2);
    //     }
    // }
    // arm.searchBoard();
    // int board[3][3] =
    //     {
    //         {human, human, robot},
    //         {human, robot, em},
    //         {em, em, robot}};
    // std::cout<<arm.findBestMove();

    // arm.getchess();
    // arm.Victory();
    return 0;
}

// 检查某一方向上是否形成五子连珠
bool playRobot::checkDirection(const checkerboard &state, int player, int startX, int startY, int dx, int dy)
{
    int count = 0;
    for (int i = 0; i < 5; ++i)
    {
        int x = startX + i * dx;
        int y = startY + i * dy;
        if (x < 0 || x >= BOARD_SIZE || y < 0 || y >= BOARD_SIZE)
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
    for (int i = 0; i < BOARD_SIZE; ++i)
    {
        for (int j = 0; j < BOARD_SIZE; ++j)
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

// 计算两点间的欧几里得距离
double playRobot::calculateDistance(int x1, int y1, int x2, int y2)
{
    int dx = x1 - x2;
    int dy = y1 - y2;
    return dx * dx + dy * dy; // 返回平方距离
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
            if (nx >= 0 && nx < BOARD_SIZE && ny >= 0 && ny < BOARD_SIZE)
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
    std::vector<position> fives, comFours, humFours;
    std::vector<position> Threes, HThrees;
    std::vector<position> Twos, neighbors;
    int opponent = (player == human) ? robot : human;
    for (int i = 0; i < BOARD_SIZE; ++i)
    {
        for (int j = 0; j < BOARD_SIZE; ++j)
        {
            if (state.board[i][j] == em && hasNeighbor(state, i, j, 2))
            {
                checkerboard newstate = state;
                // newstate.board[i][j] = player;
                int playerScore = evaluatePosition(newstate, player, i, j);
                // newstate.board[i][j] = 1 - player;
                int opponentScore = evaluatePosition(newstate, 1 - player, i, j);

                position p = {i, j};

                if (playerScore >= FIVE || opponentScore >= FIVE)
                {
                    fives.push_back(p);
                }
                else if (playerScore >= 2 * FOUR || opponentScore >= 2 * FOUR)
                {
                    HThrees.push_back(p);
                }
                else if (playerScore >= FOUR)
                {
                    comFours.push_back(p);
                }
                else if (opponentScore >= FOUR)
                {
                    humFours.push_back(p);
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
    if (!fives.empty())
        return fives;
    if (!HThrees.empty())
    {
        return HThrees;
    }
    if (!comFours.empty() && humFours.empty())
    {
        // comFours.insert(comFours.end(), Threes.begin(), Threes.end());
        return comFours;
    }
    if (comFours.empty() && !humFours.empty())
    {
        humFours.insert(humFours.end(), Threes.begin(), Threes.end());
        return humFours;
    }
    std::vector<position> fours;
    if (!comFours.empty() && !humFours.empty())
    {
        fours.insert(fours.end(), humFours.begin(), humFours.end());
        fours.insert(fours.end(), comFours.begin(), comFours.end());
        if (fours.size() >= 6)
            return fours;
        else
        {
            fours.insert(fours.end(), Threes.begin(), Threes.end());
            return fours;
        }
    }
    std::vector<position> result;
    if (Threes.size() >= 10)
    {
        return Threes;
    }
    else
    {
        result.insert(result.end(), Threes.begin(), Threes.end());
    }

    result.insert(result.end(), Twos.begin(), Twos.end());
    result.insert(result.end(), neighbors.begin(), neighbors.end());
    // 如果低分走法过多，限制返回的数量
    const int countLimit = 20; // 可以通过配置传递
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

                if (nx >= 0 && nx < BOARD_SIZE && ny >= 0 && ny < BOARD_SIZE)
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
                totalScore = std::max(totalScore, patternScores[robotPattern]);
                if (robotPattern == 0b01110)
                {
                    totalScore *= 2;
                }
            }
            if (robotPattern == 0 && player == human)
            {
                totalScore = std::max(totalScore, patternScores[humanPattern]);
                if (humanPattern == 0b01110)
                {
                    totalScore *= 2;
                }
            }
        }
    }

    return totalScore;
}

int playRobot::evaluate(const checkerboard &state)
{
    int score = 0;
    // 遍历每一行、每一列、每一条对角线
    for (int i = 0; i < BOARD_SIZE; ++i)
    {
        score += evaluateLine(state, i, 0, 0, 1); // 横线
        score += evaluateLine(state, 0, i, 1, 0); // 竖线
        if (i <= 5)
        {
            score += evaluateLine(state, 0, i, 1, 1);      // 正对角线
            score += evaluateLine(state, 0, 9 - i, 1, -1); // 反对角线
            score += evaluateLine(state, i, 0, 1, 1);      // 正对角线
            score += evaluateLine(state, i, 9, 1, -1);     // 反对角线
        }
    }

    return score;
}
// 评估一条线的得分
int playRobot::evaluateLine(const checkerboard &state, int startX, int startY, int dx, int dy)
{
    int humanScore = 0, robotScore = 0;
    for (int i = 0; i < 6; ++i)
    { // 评估的起点
        int humanPattern = 0, robotPattern = 0, count = 0;
        int humancount = 0;
        // 表示评估线的长度
        for (int j = 0; j < 5; ++j)
        {
            int x = startX + (i + j) * dx;
            int y = startY + (i + j) * dy;
            if (x < 0 || x >= BOARD_SIZE || y < 0 || y >= BOARD_SIZE)
                break;
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
            humanScore += patternScores[humanPattern];
            if (humancount > 2)
            {
                humanScore *= humancount * 10;
            }
        }
    }
    return (10 * robotScore - humanScore);
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
    if (depth >= 4 || isWinningMove(state, human) || isWinningMove(state, robot) || possiblemoves.empty())
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
                moves = getPossibleMoves(state, robot);
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

                best = std::min(best, minimax(state, possiblemoves, depth + 1, !isMax, alpha, beta));
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

int playRobot::findBestMove(checkerboard &state, int &k1, int &k2)
{
    // 优先考虑中心区域
    int bestValue = std::numeric_limits<int>::min();
    int bestMove = -1;
    int moveVal;
    int move;
    std::vector<position> moves = getPossibleMoves(state, robot);
    std::vector<std::future<std::pair<int, int>>> futures;
    // 异步计算每个可能走法的分数
    for (const auto &pos : moves)
    {
        futures.push_back(std::async(std::launch::async, [this, pos, stateCopy = state, movesCopy = moves]() -> std::pair<int, int>
                                     {
                                         checkerboard localState = stateCopy; // 独立的局部副本，避免并发问题
                                         int i = pos.x;
                                         int j = pos.y;
                                         if (i >= 0 && i < BOARD_SIZE && j >= 0 && j < BOARD_SIZE && localState.board[i][j] == em)
                                         {
                                             localState.board[i][j] = robot;
                                             int moveVal = minimax(localState, movesCopy, 0, false, std::numeric_limits<int>::min(), std::numeric_limits<int>::max());
                                             localState.board[i][j] = em;
                                             return {moveVal, i * BOARD_SIZE + j};
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
                k1 = bestMove / BOARD_SIZE;
                k2 = bestMove % BOARD_SIZE;
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
bool playRobot::searchBoard()
{
    ros::Duration(4).sleep();
    try
    {
        robot_msgs::Get_Board_State states;
        states.request.run = true;
        if (!poses.empty())
        {
            states.request.getpositions = false;
        }
        else
        {
            states.request.getpositions = true;
        }
        if (obj_detection.call(states) && !states.response.board.empty())
        {
            if ((states.response.round >= cb.round && states.response.round < cb.round + 2) || states.response.round == 0)
            {
                for (int i = 0; i < BOARD_SIZE; ++i)
                {
                    for (int j = 0; j < BOARD_SIZE; ++j)
                    {
                        cb.board[i][j] = states.response.board[i * BOARD_SIZE + j];
                    }
                }
                cb.round = states.response.round;
                std_msgs::Int32MultiArray msg;
                msg.data.clear(); // 清空数据
                msg.data.insert(msg.data.end(), states.response.board.begin(), states.response.board.end());
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
    pose = tools.transPose(pose, "tool", "xMate3_link6");
    pose = arm->setPoint(pose.position.x, pose.position.y, pose.position.z);
    pose = tools.calculateTargetPose(pose, arm->setPoint(std::vector<double>{0, 0.01, 0, 0, 0, 0}));
    tools.publishStaticTFwithRot(pose, "board");
    arm->move_l(tools.moveFromPose(pose, -0.25));
    arm->move_l(tools.moveFromPose(pose, -0.02));
    arm->Set_Tool_DO(1, true);
    arm->move_l(tools.moveFromPose(pose, -0.25));
    arm->move_j(std::vector<double>{tools.degreesToRadians(100), tools.degreesToRadians(-13), tools.degreesToRadians(-55),
                                    tools.degreesToRadians(0), tools.degreesToRadians(-110), tools.degreesToRadians(0)});
    return true;
}
bool playRobot::getchess()
{
    // arm->move_j(std::vector<double>{tools.degreesToRadians(90), tools.degreesToRadians(-20), tools.degreesToRadians(-60),
    //                                 tools.degreesToRadians(0), tools.degreesToRadians(-100), tools.degreesToRadians(0)});
    ROS_INFO("Get Chess");
    robot_msgs::Hand_Catch ct;
    ct.request.name = "blue";
    ct.request.run = true;
    bool success;
    while ((true))
    {
        if (cube_detection.call(ct) && !ct.response.positions.empty())
        {
            geometry_msgs::Pose p = ct.response.positions[0].pose;
            p = arm->setPoint(p.position.x, p.position.y, p.position.z);
            p = tools.transPose(p, "tool", "xMate3_link6");
            // p = arm->setPoint(p.position.x, p.position.y, p.position.z);
            tools.publishStaticTFwithRot(p, "chess");
            success = arm->move_l(tools.calculateTargetPose(p, arm->setPoint(std::vector<double>{-0.05, 0, -0.30, 0, 0, 0})));
            ros::Duration(4).sleep();
            while ((true))
            {
                if (cube_detection.call(ct) && !ct.response.positions.empty())
                {
                    p = ct.response.positions[0].pose;
                    p = tools.transPose(p, "tool", "xMate3_link6");
                    tools.publishStaticTFwithRot(p, "chess");
                    success = arm->move_l(tools.moveFromPose(p, -0.10));
                    success = arm->move_l(p, success);
                    arm->Set_Tool_DO(1, false);
                    success = arm->move_l(tools.moveFromPose(p, -0.15), success);
                    break;
                }
            }

            arm->move_j(std::vector<double>{tools.degreesToRadians(100), tools.degreesToRadians(-20), tools.degreesToRadians(-80),
                                            tools.degreesToRadians(0), tools.degreesToRadians(-80), tools.degreesToRadians(0)});
            break;
        }
    }

    return true;
}
bool playRobot::Victory()
{
    ROS_INFO("ROBOT Victroy !!!");
    arm->move_j(std::vector<double>{tools.degreesToRadians(100), 0, tools.degreesToRadians(-100),
                                    0, tools.degreesToRadians(100), 0});
    // arm->setMaxVelocity(0.6);
    arm->move_j(std::vector<double>{tools.degreesToRadians(100), 0, tools.degreesToRadians(-100),
                                    tools.degreesToRadians(-40), tools.degreesToRadians(100), 0});
    arm->move_j(std::vector<double>{tools.degreesToRadians(100), 0, tools.degreesToRadians(-100),
                                    tools.degreesToRadians(40), tools.degreesToRadians(100), 0});
    arm->move_j(std::vector<double>{tools.degreesToRadians(100), 0, tools.degreesToRadians(-100),
                                    tools.degreesToRadians(-40), tools.degreesToRadians(100), 0});
    arm->move_j(std::vector<double>{tools.degreesToRadians(100), 0, tools.degreesToRadians(-100),
                                    0, tools.degreesToRadians(100), 0});
    return true;
}
bool playRobot::startGame()
{
    ROS_INFO("Start Game!");
    arm->move_j(std::vector<double>{tools.degreesToRadians(100), tools.degreesToRadians(-13), tools.degreesToRadians(-55),
                                    tools.degreesToRadians(0), tools.degreesToRadians(-110), tools.degreesToRadians(0)});
    ros::Duration(3).sleep();
    while (true)
    {
        if (searchBoard() && cb.round % 2 != 0)
        {
            ROS_INFO("Robot's trun");
            int k1, k2;
            int p = findBestMove(cb, k1, k2);
            ROS_INFO("p:%d ", p);
            getchess();
            move(poses[p].pose);
            // cb.board[k1][k2] == robot;
            // cb.round++;
        }
        if (isWinningMove(cb, robot))
        {
            Victory();
            break;
        }
    }
    return true;
}
