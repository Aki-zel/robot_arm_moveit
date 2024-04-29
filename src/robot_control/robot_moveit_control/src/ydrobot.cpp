#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ros::init(argc, argv, "myviz");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(3);

    MainWindow w;
    spinner.start();
    w.show();
    // ros::waitForShutdown();
    // ros::spin();
    return a.exec();
}