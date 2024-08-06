#include <ros/ros.h>
#include <bits/stdc++.h>
#include <MoveitServer.h>
#include <robotTool.h>
#include <robot_msgs/Get_Board_State.h>
#include <robot_msgs/Hand_Catch.h>
struct checkerboard
{
    int board[3][3];
    int round;
};
enum chess
{
    em = -1,
    human, // O
    robot  // X
};
class playRobot
{
private:
    ros::NodeHandle nh;
    std::unique_ptr<MoveitServer> arm;
    robotTool tools;
    checkerboard cb;
    ros::ServiceClient obj_detection, cube_detection;
    std::vector<geometry_msgs::PoseStamped> poses;

public:
    playRobot(std::string planGroup)
    {
        arm = std::make_unique<MoveitServer>(planGroup);
        cube_detection = nh.serviceClient<robot_msgs::Hand_Catch>("color_detect");
        arm->setMaxVelocity(0.2);
    }
    int checkVictory(int board[3][3]);
    bool move(geometry_msgs::Pose pose);
    int minimax(int board[3][3], int depth, bool isMax);
    int findBestMove(int board[3][3]);
    bool searchBoard();
    bool getchess();
    bool Victory();
    bool startGame();
    ~playRobot()
    {
    }
};
int playRobot::checkVictory(int board[3][3])
{
    int s = 0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            if (board[i][j] == em)
                s++;
    if (s == 0)
    {
        return 2;
    }
    for (std::size_t i = 0; i < 3; i++)
    {
        if (board[i][0] == board[i][1] && board[i][1] == board[i][2] && board[i][0] != -1)
        {
            if (board[i][0] == robot)
            {
                return 10;
            }
            else
            {
                return -10;
            }
        }
        if (board[0][i] == board[1][i] && board[1][i] == board[2][i] && board[0][i] != -1)
        {

            if (board[0][i] == robot)
            {
                return 10;
            }
            else
            {
                return -10;
            }
        }
    }
    if ((board[0][0] == board[1][1] && board[1][1] == board[2][2]) ||
        board[2][0] == board[1][1] && board[1][1] == board[0][2])
    {
        if (board[1][1] == robot)
        {
            return 10;
        }
        else
        {
            return -10;
        }
    }
    return 0;
}
bool playRobot::searchBoard()
{
    try
    {
        robot_msgs::Get_Board_State states;
        states.request.run = true;
        if (obj_detection.call(states))
        {
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    cb.board[i][j] = states.response.board[i * 3 + j];
                }
            }
            cb.round++;
            poses = states.response.positions;
            return true;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return false;
}
int playRobot::minimax(int board[3][3], int depth, bool isMax)
{
    int score = checkVictory(board);
    if (score == 10)
        return score;
    if (score == -10)
        return score;
    if (score == 2)
        return 0;
    if (isMax)
    {
        int best = -1000;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                if (board[i][j] == em)
                {
                    board[i][j] = robot;

                    best = std::max(best,
                                    minimax(board, depth + 1, !isMax));

                    board[i][j] = em;
                }
            }
        }
        return best;
    }

    else
    {
        int best = 1000;

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                if (board[i][j] == em)
                {

                    board[i][j] = human;
                    best = std::min(best,
                                    minimax(board, depth + 1, !isMax));
                    board[i][j] = em;
                }
            }
        }
        return best;
    }
}
int playRobot::findBestMove(int board[3][3])
{
    int bestVal = -1000;
    int point = 0;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (board[i][j] == em)
            {
                board[i][j] = robot;
                int moveVal = minimax(board, 0, false);

                board[i][j] = em;
                if (moveVal > bestVal)
                {
                    point = i * 3 + j;
                    bestVal = moveVal;
                }
            }
        }
    }
    return point;
}
bool playRobot::move(geometry_msgs::Pose pose)
{
    pose = tools.transPose(pose, "tool", "xMate3_link6");
    arm->move_p(tools.moveFromPose(pose, -0.1));
    arm->move_l(pose);
    arm->Set_Tool_DO(1, true);
    arm->move_l(tools.moveFromPose(pose, -0.1));
    arm->move_j(std::vector<double>{0, tools.degreesToRadians(-30), tools.degreesToRadians(-60),
                                    0, tools.degreesToRadians(-90), 0});
    return true;
}
bool playRobot::getchess()
{
    ROS_INFO("Get Chess");
    arm->move_j(std::vector<double>{tools.degreesToRadians(-50), tools.degreesToRadians(-30), tools.degreesToRadians(-60),
                                    0, tools.degreesToRadians(-90), 0});
    robot_msgs::Hand_Catch ct;
    ct.request.name = "blue";
    ct.request.run = true;
    ros::Duration(5).sleep();
    if (cube_detection.call(ct) && !ct.response.positions.empty())
    {
        geometry_msgs::Pose p = ct.response.positions[0].pose;
        p = tools.transPose(p, "tool", "xMate3_link6");
        tools.publishStaticTFwithRot(p);
        arm->move_p(tools.moveFromPose(p, -0.1));
        arm->move_l(p);
        arm->Set_Tool_DO(1, false);
        arm->move_l(tools.moveFromPose(p, -0.1));
        arm->move_j(std::vector<double>{0, tools.degreesToRadians(-30), tools.degreesToRadians(-60),
                                        0, tools.degreesToRadians(-90), 0});
    }
    return true;
}
bool playRobot::Victory()
{
    ROS_INFO("ROBOT Victroy !!!");
    arm->move_j(std::vector<double>{0, 0, tools.degreesToRadians(-100),
                                    0, tools.degreesToRadians(100), 0});
    arm->setMaxVelocity(0.6);
    arm->move_j(std::vector<double>{0, 0, tools.degreesToRadians(-100),
                                    tools.degreesToRadians(-40), tools.degreesToRadians(100), 0});
    arm->move_j(std::vector<double>{0, 0, tools.degreesToRadians(-100),
                                    tools.degreesToRadians(40), tools.degreesToRadians(100), 0});
    arm->move_j(std::vector<double>{0, 0, tools.degreesToRadians(-100),
                                    tools.degreesToRadians(-40), tools.degreesToRadians(100), 0});
    arm->move_j(std::vector<double>{0, 0, tools.degreesToRadians(-100),
                                    tools.degreesToRadians(0), tools.degreesToRadians(100), 0});
    return true;
}
bool playRobot::startGame()
{
    ROS_INFO("Start Game!");
    arm->move_j(std::vector<double>{0, tools.degreesToRadians(-30), tools.degreesToRadians(-60),
                                    0, tools.degreesToRadians(-90), 0});
    while (true)
    {
        if (searchBoard() && cb.round % 2 != 0)
        {
            ROS_INFO("Robot's trun");
            int p = findBestMove(cb.board);
            getchess();
            move(poses[p].pose);
            cb.board[p / 3][p % 3] == robot;
            if (checkVictory(cb.board) == 10)
            {
                Victory();
            }
        }
    }
    return true;
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "example05");
    ros::NodeHandle nh;
    playRobot arm("rokae_arm");
    // int board[3][3] =
    //     {
    //         {human, human, robot},
    //         {human, robot, em},
    //         {em, em, robot}};
    // std::cout<<arm.findBestMove(board);

    // arm.getchess();
    // arm.Victory();
    return 0;
}