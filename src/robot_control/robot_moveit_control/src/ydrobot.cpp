#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "myviz");
    QApplication a(argc, argv);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    MainWindow w;
    w.show();
    return a.exec();;
}