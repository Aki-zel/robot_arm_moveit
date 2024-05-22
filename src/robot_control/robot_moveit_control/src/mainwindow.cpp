#include "mainwindow.h"
#include "ui_mainwindow.h"
#define Pi 3.1415926
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
                                          ui(new Ui::MainWindow)
{
    ui->setupUi(this); // 初始化UI
    setMouseTracking(true); //  鼠标追踪
    this->m_isImage = false; // 默认不显示图像
    this->m_isSelecting = false; // 默认不选择
    this->render_panel_ = new rviz::RenderPanel(); // 创建一个RViz渲染面板
    ui->rivzLayout->addWidget(this->render_panel_); // 将RViz添加到布局中
    // 初始化ROS节点
    ros::NodeHandle nh;
    ros::CallbackQueue queue; // 创建一个回调队列
    ros::AsyncSpinner spinner(3); // 创建异步回调器
    spinner.start();
    std::string PLANNING_GROUP = "arm";
    this->server = new MoveitServer(PLANNING_GROUP); // 创建一个MoveitServer对象规划控制机械臂
    image_subscriber_ = nh.subscribe("/camera/color/image_raw/compressed", 10, &MainWindow::imageCallback, this);
    objection_subscriber_ = nh.subscribe("object_position", 10, &MainWindow::objectionCallback, this);
    image_publisher_ = nh.advertise<sensor_msgs::Image>("/image_template", 10);
    client = nh.serviceClient<robot_msgs::Hand_Catch>("objection_detect"); // 创建目标检测服务客户端
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::objectionCallback(const geometry_msgs::PoseStampedConstPtr &msg) //模版匹配回调函数
{
    if (!msg) {
        ROS_ERROR("Received a null pose message.");
        return; // 提前返回，避免空指针解引用
    }

    try
    {
        std::array<double, 3> position = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
        
        this->server->Set_Tool_DO(2, false);
        ROS_INFO("夹爪开");
        ros::Duration(1.0).sleep();
        
        std::array<double, 3> targetPositionAbove = {position[0], position[1], position[2] + 0.10};
        this->server->move_p(targetPositionAbove);
        ROS_INFO("移动到目标上方");
        // ros::Duration(1.0).sleep();
        
        std::array<double, 3> grabPosition = {position[0], position[1], position[2]};
        this->server->move_p(grabPosition);
        ROS_INFO("移动到夹取位置");
        // ros::Duration(1.0).sleep();

        this->server->Set_Tool_DO(2, true);
        ROS_INFO("夹取目标物体");
        ros::Duration(1.0).sleep();

        this->server->move_p(targetPositionAbove);
        ROS_INFO("抬起目标");
        // ros::Duration(1.0).sleep();

        std::array<double, 3> movePosition = {position[0], position[1] + 0.20, position[2]};
        this->server->move_p(movePosition);
        ROS_INFO("移动到指定位置");
        // ros::Duration(1.0).sleep();

        this->server->Set_Tool_DO(2, false);
        ROS_INFO("夹爪开");
        ros::Duration(1.0).sleep();
        
        movePosition = {position[0], position[1] + 0.20, position[2] + 0.10};
        this->server->move_p(movePosition);
        // ros::Duration(1.0).sleep();
        
        std::vector<double> joint = {0.175, 0.262, -1.152, 0, -1.885, -3.072};
        this->server->move_j(joint);
        ROS_INFO("回到初始位置");
    }
    catch (const std::exception &e)
    {
        std::cerr << "捕获到异常: " << e.what() << '\n';
    }
}

void MainWindow::imageCallback(const sensor_msgs::CompressedImageConstPtr &msg) // 接收相机图像并显示
{
    try
    {
        if (this->m_isImage)
        {
            // 将ROS图像消息转换为OpenCV格式
            // cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

            // 将OpenCV图像转换为Qt格式
            QImage img(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.step, QImage::Format_RGB888);
            QPixmap pixmap = QPixmap::fromImage(img);

            // 在ImageBox中显示图像
            this->ui->imageBox->setPixmap(pixmap);
            this->ui->imageBox->setScaledContents(true);
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert . ");
    }
}

void MainWindow::on_closeButton_clicked()
{
    this->close();
    delete this;
}

// 启动
void MainWindow::on_startButton_clicked()
{
    this->addStart();
    this->m_isImage = true;
    std::vector<double> joint = {0.175, 0.262, -1.152, 0, -1.885, -3.072};
    this->server->move_j(joint);
    this->server->Set_Tool_DO(2, true);
}

void MainWindow::addStart()
{
    this->manager_ = new rviz::VisualizationManager(render_panel_); // 获取可视化rviz的控制对象，后面直接操作这个

    this->render_panel_->initialize(this->manager_->getSceneManager(), this->manager_); // 绑定交互信号(初始化camera实现放大 缩小 平移等操作)

    this->manager_->initialize();
    this->manager_->removeAllDisplays();
    this->manager_->startUpdate();
    this->manager_->setFixedFrame("dummy");
    auto grid_ = this->manager_->createDisplay("rviz/Grid", "adjustable grid", true);
    ROS_ASSERT(grid_ != NULL);
    auto robotmodel_ = this->manager_->createDisplay("rviz/RobotModel", "RobotModel", true);
    ROS_ASSERT(robotmodel_ != NULL);
}

// 选择模版图像区域
void MainWindow::on_chooseButton_clicked()
{
    m_isSelecting = true;
}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && m_isSelecting)
    {
        m_startPos = event->pos();
        m_endPos = m_startPos;
    }
}

void MainWindow::mouseMoveEvent(QMouseEvent *event)
{
    if (m_isSelecting)
    {
        m_endPos = event->pos();
        update(); // 强制重绘以显示选择区域
    }
}

void MainWindow::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && m_isSelecting)
    {
        m_isSelecting = false;
        QScreen *screen = QGuiApplication::primaryScreen();
        if (screen)
        {
            // 计算选择区域的矩形
            QRect selectionRect = QRect(m_startPos, m_endPos).normalized();
            // std::cout<<"re  "<<selectionRect.x()<<"  "<<selectionRect.y()<<std::endl;
            QPixmap screenshot = screen->grabWindow(this->ui->centralwidget->winId(), selectionRect.x(), selectionRect.y(), selectionRect.width(), selectionRect.height());
            image_publisher_.publish(this->convertQPixmapToSensorImage(screenshot));
            // 截取选择区域的屏幕
            QLabel *label = new QLabel;
            label->setPixmap(screenshot);
            label->resize(screenshot.size());
            label->show();
        }
        update();
    }
}

sensor_msgs::ImagePtr MainWindow::convertQPixmapToSensorImage(const QPixmap &pixmap) // 将Qt图像转换为ROS图像信息
{
    // 将QPixmap转换为QImage
    QImage image = pixmap.toImage();

    // 将QImage转换为OpenCV格式
    cv::Mat cv_image(image.height(), image.width(), CV_8UC4, (uchar *)image.bits(), image.bytesPerLine());
    cv::cvtColor(cv_image, cv_image, cv::COLOR_RGBA2BGR); // Qt的QImage默认使用RGBA格式，而OpenCV默认使用BGR格式，需要进行通道转换
    // 创建一个cv_bridge::CvImage对象
    cv_bridge::CvImage cv_bridge_image;
    cv_bridge_image.image = cv_image;
    cv_bridge_image.encoding = "bgr8"; // 设置图像编码为BGR8

    // 将cv_bridge::CvImage对象转换为sensor_msgs::Image
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_bridge_image.image).toImageMsg();
    return msg;
}

// 目标检测
void MainWindow::on_detectButton_clicked()
{
    if (m_cvImage.empty())
    {
        ROS_WARN("No image received yet");
        return;
    }

    if (callDetectService()) 
    {
        // 显示检测图像
        QLabel *label = new QLabel;
        QImage qimage(m_cvImage.data, m_cvImage.cols, m_cvImage.rows, m_cvImage.step, QImage::Format_RGB888);
        label->setPixmap(QPixmap::fromImage(qimage));
        label->resize(qimage.size());
        label->show();
    }
}

bool MainWindow::callDetectService() // 调用检测服务
{
    robot_msgs::Hand_Catch srv;
    srv.request.run = true;  // 设置请求标志位

    if (client.call(srv))
    {
        ROS_INFO("Service call succeeded");
        processDetectionResults(srv.response); // 处理检测结果
        return true;
    }
    else
    {
        ROS_ERROR("Failed to call service objection_detect");
        return false;
    }
}

void MainWindow::processDetectionResults(const robot_msgs::Hand_CatchResponse& response) // 处理检测结果
{
    // 假设response.labels和response.positions是std::vector<std::string>和std::vector<float>类型
    for (size_t i = 0; i < response.labels.size(); ++i)
    {
        std::string label = response.labels[i];
        float x = response.positions[3 * i];
        float y = response.positions[3 * i + 1];
        float z = response.positions[3 * i + 2];
        ROS_INFO("Detected object: %s at (%.2f, %.2f, %.2f)", label.c_str(), x, y, z);
        // 在图像上绘制检测结果或其他处理
        cv::rectangle(m_cvImage, cv::Point(x - 50, y - 50), cv::Point(x + 50, y + 50), cv::Scalar(0, 255, 0), 2);
        cv::putText(m_cvImage, label, cv::Point(x - 50, y - 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

        // controlRobotToGrab(x, y, z);
    }
}

void MainWindow::controlRobotToGrab(float x, float y, float z)
{
    try
    {
        std::array<double, 3> position = {x, y, z};
        
        // 打开夹爪
        this->server->Set_Tool_DO(2, false);
        ROS_INFO("夹爪开");
        ros::Duration(1.0).sleep();
        
        // 移动到目标上方
        std::array<double, 3> targetPositionAbove = {position[0], position[1], position[2] + 0.10};
        this->server->move_p(targetPositionAbove);
        ROS_INFO("移动到目标上方");
        ros::Duration(1.0).sleep();
        
        // 移动到抓取位置
        std::array<double, 3> grabPosition = {position[0], position[1], position[2]};
        this->server->move_p(grabPosition);
        ROS_INFO("移动到抓取位置");
        ros::Duration(1.0).sleep();

        // 关闭夹爪
        this->server->Set_Tool_DO(2, true);
        ROS_INFO("夹取目标物体");
        ros::Duration(2.0).sleep();

        // 抬起目标
        this->server->move_p(targetPositionAbove);
        ROS_INFO("抬起目标");
        ros::Duration(1.0).sleep();

        // 移动到指定位置
        std::array<double, 3> movePosition = {position[0], position[1] + 0.20, position[2]};
        this->server->move_p(movePosition);
        ROS_INFO("移动到指定位置");
        ros::Duration(1.0).sleep();

        // 打开夹爪
        this->server->Set_Tool_DO(2, false);
        ROS_INFO("夹爪开");
        ros::Duration(2.0).sleep();
        
        // 移动到初始位置
        std::vector<double> joint = {0.175, 0.262, -1.152, 0, -1.885, -3.072};
        this->server->move_j(joint);
        ROS_INFO("回到初始位置");
    }
    catch (const std::exception &e)
    {
        std::cerr << "捕获到异常: " << e.what() << '\n';
    }
}


void MainWindow::on_settingButton_clicked()
{
    ui->stackedWidget->setCurrentIndex(1);
}

void MainWindow::on_backButton_clicked()
{
    ui->stackedWidget->setCurrentIndex(0);
}
