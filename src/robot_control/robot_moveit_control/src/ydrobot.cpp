#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "myviz");
    QApplication a(argc, argv);

    MainWindow w;
    w.show();
    return a.exec();;
}