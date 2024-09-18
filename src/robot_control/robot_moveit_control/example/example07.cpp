#include <ros/ros.h>
#include <bits/stdc++.h>
#include <MoveitServer.h>
#include <robotTool.h>
#include <robot_msgs/Board_State.h>
#include <robot_msgs/Hand_Catch.h>

class playRobot
{
private:
    ros::NodeHandle nh;
    std::unique_ptr<MoveitServer> arm;
    robotTool tools;
    ros::ServiceClient cube_detection;
    ros::ServiceServer board_state;

public:
    playRobot(std::string planGroup)
    {
        arm = std::make_unique<MoveitServer>(planGroup);
        cube_detection = nh.serviceClient<robot_msgs::Hand_Catch>("color_detect");
        board_state = nh.advertiseService("/cubegame", &playRobot::handleBoardState, this);
        arm->setMaxVelocity(0.5);
        arm->move_j(std::vector<double>{tools.degreesToRadians(0), tools.degreesToRadians(-20), tools.degreesToRadians(72),
                                    tools.degreesToRadians(0), tools.degreesToRadians(109), tools.degreesToRadians(90)});
    }
    bool move(geometry_msgs::Pose pose);
    bool searchBoard();
    bool getcube(std::string colorname);
    bool handleBoardState(robot_msgs::Board_State::Request &req, robot_msgs::Board_State::Response &res);
    ~playRobot()
    {
    }
};

// 放置
bool playRobot::move(geometry_msgs::Pose pose)
{
    arm->move_l(tools.moveFromPose(pose, -0.1));
    arm->move_l(pose);
    arm->Set_Tool_DO(1, true);
    arm->move_l(tools.moveFromPose(pose, -0.1));
    arm->move_j(std::vector<double>{tools.degreesToRadians(0), tools.degreesToRadians(-20), tools.degreesToRadians(72),
                                    tools.degreesToRadians(0), tools.degreesToRadians(109), tools.degreesToRadians(90)});
}

// 抓取
bool playRobot::getcube(std::string color)
{
    ROS_INFO("Get Chess");
    robot_msgs::Hand_Catch ct;
    ct.request.name = color;
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

// 服务回调函数
bool playRobot::handleBoardState(robot_msgs::Board_State::Request &req, robot_msgs::Board_State::Response &res)
{
    arm->move_j(std::vector<double>{tools.degreesToRadians(0), tools.degreesToRadians(-20), tools.degreesToRadians(72),
                                    tools.degreesToRadians(0), tools.degreesToRadians(109), tools.degreesToRadians(90)});

    geometry_msgs::Pose startPose;
    startPose.position.x = 0.56;
    startPose.position.y = 0.0;
    startPose.position.z = 0.002;
    startPose.orientation.w = 1;
    startPose = tools.calculateTargetPose(startPose, arm->setPoint(std::vector<double>{0, 0, 0, 0, tools.degreesToRadians(180), 0}));
    startPose = tools.transPose(startPose, "tool", "xMate3_link6");
    double blockSize = 0.018; // 方块中心间隔
    int center = 2;

    // 遍历每个方块
    for (std::size_t i = 0; i < req.positions.size(); ++i)
    {
        // 获取方块的位置 (行, 列)
        int row = req.positions[i].row;
        int col = req.positions[i].col;

        // 获取方块的角度和颜色
        double angle = (double)req.angle[i];

        std::string color_name;
        int color = req.color[i];
        if (color == 2)
        {
            color_name = "blue";
        }
        else if (color == 3)
        {
            color_name = "green";
        }
        else if (color == 1)
        {
            color_name = "orange";
        }

        // stackBlocks;
        if (req.type == 1)
        {
            double yOffset = (col - center) * blockSize;
            double zOffset = (5 - row - 1) * 0.0155;

            geometry_msgs::Pose targetPose = startPose;
            targetPose.position.y += yOffset;
            targetPose.position.z += zOffset;
            if (getcube(color_name))
            {
                targetPose = tools.calculateTargetPose(targetPose, arm->setPoint(std::vector<double>{0, 0, 0, 0, 0, tools.degreesToRadians(angle)}));
                move(targetPose); // 放置方块
            }
        }
        // placeBlocks;
        else if (req.type == 2)
        {
            double xOffset = (row - 2) * blockSize;
            double yOffset = (col - 2) * blockSize;

            geometry_msgs::Pose targetPose = startPose;
            targetPose.position.y += xOffset;
            targetPose.position.x -= yOffset;

            if (getcube(color_name))
            {
                targetPose = tools.calculateTargetPose(targetPose, arm->setPoint(std::vector<double>{0, 0, 0, 0, 0, tools.degreesToRadians(angle)}));

                move(targetPose); // 放置方块
            }
        }
        else
        {
            ROS_ERROR("Invalid type received: %d", req.type);
            res.success = false;
            return true;
        }
    }

    res.success = true;
    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "example07");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    playRobot arm("rokae_arm");

    // ros::spin();
    ros::waitForShutdown();
    return 0;
}
