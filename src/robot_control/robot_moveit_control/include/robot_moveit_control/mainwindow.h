#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/tool.h>
#include <rviz/default_plugin/view_controllers/orbit_view_controller.h>
#include <rviz/view_manager.h>
#include <rviz/yaml_config_reader.h>
#include <fstream>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/assert.h>
#include <QtWidgets>
#include <QGuiApplication>
#include <QScreen>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <MoveitServer.h>
#include <opencv2/highgui.hpp>
#include <QLabel>
#include <thread>
namespace Ui
{
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void addStart();
    void imageCallback(const sensor_msgs::CompressedImageConstPtr &msg);
    void objectionCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    sensor_msgs::ImagePtr convertQPixmapToSensorImage(const QPixmap &pixmap);

private slots:

    void on_closeButton_clicked();

    void on_chooseButton_clicked();

    void on_settingButton_clicked();

    void on_backButton_clicked();

    void on_startButton_clicked();

protected:
    void on_detectButton_clicked();
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    // void paintEvent(QPaintEvent *event) override;

private:
    Ui::MainWindow *ui;
        ros::NodeHandle nh;
    rviz::RenderPanel *render_panel_;     // rviz显示容器
    rviz::VisualizationManager *manager_; // rviz控制器
    QPoint m_startPos;
    QPoint m_endPos;
    bool m_isSelecting;
    bool m_isImage;
    ros::Subscriber image_subscriber_;
    ros::Subscriber objection_subscriber_;
    ros::Publisher image_publisher_;
    MoveitServer *server;
};

#endif // MAINWINDOW_H
