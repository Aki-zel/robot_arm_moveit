#include "mainwindow.h"
#include "ui_mainwindow.h"
#define Pi 3.1415926

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
                                          ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setMouseTracking(true);
    this->m_isImage = true;
    this->m_isSelecting = false;
    ros::CallbackQueue queue;
    image_subscriber_ = nh.subscribe("/camera/color/image_raw/compressed", 10, &MainWindow::imageCallback, this);
    objection_subscriber_ = nh.subscribe("object_position", 10, &MainWindow::objectionCallback, this);
    nh.setCallbackQueue(&queue);
    ros::AsyncSpinner spinner(2, &queue);
    spinner.start();
    // this->server = new robotControl();
    std::string PLANNING_GROUP = "arm"; // 你的规划组名称
    this->server = new MoveitServer(PLANNING_GROUP);
    image_publisher_ = nh.advertise<sensor_msgs::Image>("/image_template", 10);
    this->addStart();
    connect(this, SIGNAL(updateImageSignal(QImage)), this, SLOT(updateImageSlot(QImage)));
    object_client_ = nh.serviceClient<robot_msgs::Hand_Catch>("object_detect");
}

MainWindow::~MainWindow()
{
    delete server;
}

void MainWindow::objectionCallback(const geometry_msgs::PoseStampedConstPtr &msg) // 模版匹配回调函数
{
    if (!msg)
    {
        ROS_ERROR("Received a null pose message.");
        return; // 提前返回，避免空指针解引用
    }

    try
    {
        this->server->Set_Tool_DO(2, false);
        this->server->Set_Tool_DO(2, false);
        setEnableButton(false);
        geometry_msgs::Pose pos = msg.get()->pose;
        // pos.position.z += 0.15;
        this->server->move_p(pos);

        // this->server->move_p(msg.get()->pose);
        // this->server->Set_Tool_DO(2, true);
        // std::vector<double> joint = {0, 0, this->server->degreesToRadians(-90), 0, this->server->degreesToRadians(-90), this->server->degreesToRadians(180)};
        // this->server->move_j(joint);
        // this->server->Set_Tool_DO(2, false);
        setEnableButton(true);
    }
    catch (const std::exception &e)
    {
        std::cerr << "捕获到异常: " << e.what() << '\n';
    }
}

void MainWindow::on_closeButton_clicked()
{
    this->close();
}

void MainWindow::setEnableButton(bool enbale)
{
    QList<QPushButton *> allButtons = this->findChildren<QPushButton *>();
    for (auto button : allButtons)
    {
        button->setEnabled(enbale);
    }
}

// 启动
void MainWindow::on_startButton_clicked()
{
    this->m_isImage = true;
    setEnableButton(false);
    this->server->Set_Tool_DO(2, true);
    std::vector<double> joint = {0, 0, this->server->degreesToRadians(-90), 0, this->server->degreesToRadians(-90), this->server->degreesToRadians(180)};
    // this->server->move_j(joint);
    std::thread moveThread([this, joint]()
                           {
        this->server->move_j(joint);
        setEnableButton(true); });
    moveThread.detach(); // 分离线程
}

void MainWindow::addStart()
{
    this->render_panel_ = new rviz::RenderPanel();
    ui->rivzLayout->addWidget(this->render_panel_);
    this->manager_ = new rviz::VisualizationManager(render_panel_); // 获取可视化rviz的控制对象，后面直接操作这个

    this->render_panel_->initialize(this->manager_->getSceneManager(), this->manager_); // 绑定交互信号(初始化camera实现放大 缩小 平移等操作)

    this->manager_->initialize();
    this->manager_->removeAllDisplays();
    this->manager_->startUpdate();
    this->manager_->setFixedFrame("world");
    auto grid_ = this->manager_->createDisplay("rviz/Grid", "adjustable grid", true);
    ROS_ASSERT(grid_ != NULL);
    auto robotmodel_ = this->manager_->createDisplay("rviz/RobotModel", "RobotModel", true);
    ROS_ASSERT(robotmodel_ != NULL);
    auto TF_ = this->manager_->createDisplay("rviz/TF", "TF", true);
    ROS_ASSERT(TF_ != NULL);
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
        this->m_isImage = false;
        m_endPos = event->pos();
        update();
        // std::cout<<"mo "<<m_endPos.x()<<"  "<<m_endPos.y()<<std::endl;
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
        }
        update();
    }
}

void MainWindow::imageCallback(const sensor_msgs::CompressedImageConstPtr &msg)
{
    try
    {
        if (this->m_isImage)
        {
            // 将ROS图像消息转换为OpenCV格式
            // cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
            if (cv_ptr->image.data != nullptr || !cv_ptr->image.empty())
            {
                // 将OpenCV图像转换为Qt格式
                QImage img((unsigned char *)cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.step, QImage::Format_RGB888);
                emit updateImageSignal(img);
            }
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert . ");
    }
}

void MainWindow::updateImageSlot(const QImage &img)
{
    // 在ImageBox中显示图像
    this->ui->imageBox->setPixmap(QPixmap::fromImage(img));
    this->ui->imageBox->setScaledContents(true);
}

sensor_msgs::ImagePtr MainWindow::convertQPixmapToSensorImage(const QPixmap &pixmap) // 将Qt图像转换为ROS图像信息
{
    // 将QPixmap转换为QImage
    QImage image = pixmap.toImage();

    // 将QImage转换为OpenCV格式
    cv::Mat cv_image(image.height(), image.width(), CV_8UC4, (uchar *)image.bits(), image.bytesPerLine());
    cv::cvtColor(cv_image, cv_image, cv::COLOR_RGBA2BGR); // Qt的QImage默认使用RGBA格式，而OpenCV默认使用BGR格式，需要进行通道转换
    std::thread showimage([this, cv_image]()
                          {
                            cv::imshow("Detection Results", cv_image);
                            cv::waitKey(0); // 按下任意键继续
                            cv::destroyAllWindows(); });
    showimage.detach(); // 分离线程
    this->m_isImage = true;
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
    // 移动到检测位置
    std::vector<double> joint = {0, 0, this->server->degreesToRadians(-90), 0, this->server->degreesToRadians(-90), this->server->degreesToRadians(180)};
    // std::vector<double> joint = {0, 0.349, -0.524, 0, -1.047, -3.142};
    this->server->move_j(joint);
    callDetectService();
}

bool MainWindow::callDetectService() // 调用检测服务
{
    robot_msgs::Hand_Catch srv;
    srv.request.run = true; // 设置请求标志位

    if (object_client_.call(srv))
    {
        ROS_INFO("Service call succeeded");
        processDetectionResults(srv.response); // 处理检测结果
        return true;
    }
    else
    {
        ROS_ERROR("Failed to call service object_detect");
        return false;
    }
}

void MainWindow::processDetectionResults(const robot_msgs::Hand_CatchResponse &response) // 处理检测结果
{
    try
    {
        // 显示检测结果图像
        cv::Mat detect_image = cv_bridge::toCvCopy(response.detect_image, sensor_msgs::image_encodings::BGR8)->image;
        cv::imshow("Detection Results", detect_image);
        cv::waitKey(0); // 按下任意键继续
        cv::destroyAllWindows();
    }
    catch (const cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (response.labels.empty() || response.positions.size() < 1)
    {
        ROS_ERROR("Detection response is empty or positions are insufficient.");
        return;
    }

    // 获取置信度最高的物体的位置和标签
    std::string label = response.labels[0];
    auto position = response.positions[0];
    ROS_INFO("Detected object: %s", label.c_str());

    controlRobotToGrab(position);
}

void MainWindow::controlRobotToGrab(geometry_msgs::PoseStamped position)
{
    try
    {
        geometry_msgs::Pose p = position.pose;

        // 移动到目标下方
        p.position.z -= 0.10;
        this->server->move_p(p);
        ROS_INFO("移动到目标下方");
        ros::Duration(1.0).sleep();

        // 打开夹爪
        this->server->Set_Tool_DO(2, false);
        ROS_INFO("夹爪开");
        ros::Duration(1.0).sleep();
        p = position.pose;
        // 移动到抓取位置
        this->server->move_p(p);
        ROS_INFO("移动到抓取位置");
        ros::Duration(1.0).sleep();
        p.position.z -= 0.10;
        // 关闭夹爪
        this->server->Set_Tool_DO(2, true);
        ROS_INFO("夹取目标物体");
        ros::Duration(2.0).sleep();

        // 摘取目标物体
        this->server->move_p(p);
        ROS_INFO("摘取目标");
        ros::Duration(1.0).sleep();

        // 打开夹爪
        this->server->Set_Tool_DO(2, false);
        ROS_INFO("夹爪开");
        ros::Duration(2.0).sleep();

        // 移动到初始位置
        std::vector<double> joint = {0, 0.349, -0.524, 0, -1.047, -3.142};
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
