#include <ros/ros.h>
#include <MoveitServer.h>

int main(int argc, char *argv[])
{
    // 支持终端输出中文
    setlocale(LC_ALL, "");
    // ros初始化
    ros::init(argc, argv, "exemple04");
    ros::NodeHandle nh;
    return 0;
}
