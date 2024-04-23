#include "../include/robot_moveit_control/mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    if (!ros::isInitialized())
    {
        ros::init(argc, argv, "myviz", ros::init_options::AnonymousName);
    }
    MainWindow w;
    w.show();
    // ros::waitForShutdown();
    return a.exec();
}
