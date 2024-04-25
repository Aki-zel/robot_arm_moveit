#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ros::init(argc, argv, "myviz");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(3);
    spinner.start();
    MainWindow w;
    w.show();
    // ros::waitForShutdown();
    return a.exec();
}
