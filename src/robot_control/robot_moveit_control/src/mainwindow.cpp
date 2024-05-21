#include "mainwindow.h"
#include "ui_mainwindow.h"
#define Pi 3.1415926
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
                                          ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setMouseTracking(true);
    this->m_isImage = false;
    this->m_isSelecting = false;
    this->render_panel_ = new rviz::RenderPanel();
    ui->rivzLayout->addWidget(this->render_panel_);
    // 初始化ROS节点
    ros::NodeHandle nh;
    ros::CallbackQueue queue;
    ros::AsyncSpinner spinner(3);
    spinner.start();
    std::string PLANNING_GROUP = "arm";
    this->server = new MoveitServer(PLANNING_GROUP);
    image_subscriber_ = nh.subscribe("/camera/color/image_raw/compressed", 10, &MainWindow::imageCallback, this);
    objection_subscriber_ = nh.subscribe("object_position", 10, &MainWindow::objectionCallback, this);
    // nh.setCallbackQueue(&queue);

    image_publisher_ = nh.advertise<sensor_msgs::Image>("/image_template", 10);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::objectionCallback(const geometry_msgs::PoseStampedConstPtr &msg)
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

void MainWindow::on_closeButton_clicked()
{
    this->close();
    delete this;
}

void MainWindow::on_chooseButton_clicked()
{
    m_isSelecting = true;
}

void MainWindow::on_settingButton_clicked()
{
    ui->stackedWidget->setCurrentIndex(1);
}

void MainWindow::on_backButton_clicked()
{
    ui->stackedWidget->setCurrentIndex(0);
}

void MainWindow::on_startButton_clicked()
{
    this->addStart();
    this->m_isImage = true;
    std::vector<double> joint = {0.175, 0.262, -1.152, 0, -1.885, -3.072};
    this->server->move_j(joint);
    this->server->Set_Tool_DO(2, true);
}

void MainWindow::on_detectButton_clicked()
{
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

void MainWindow::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && m_isSelecting)
    {
        m_startPos = event->pos();
        m_endPos = m_startPos;
        // std::cout<<"st  "<<m_startPos.x()<<"   "<<m_startPos.y()<<std::endl;
    }
}

void MainWindow::mouseMoveEvent(QMouseEvent *event)
{
    if (m_isSelecting)
    {
        m_endPos = event->pos();
        update(); // 强制重绘以显示选择区域
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
            // 截取选择区域的屏幕
            QLabel *label = new QLabel;
            label->setPixmap(screenshot);
            label->resize(screenshot.size());
            label->show();
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

sensor_msgs::ImagePtr MainWindow::convertQPixmapToSensorImage(const QPixmap &pixmap)
{
    // 将QPixmap转换为QImage
    QImage image = pixmap.toImage();

    // 将QImage转换为OpenCV格式
    cv::Mat cv_image(image.height(), image.width(), CV_8UC4, (uchar *)image.bits(), image.bytesPerLine());
    cv::cvtColor(cv_image, cv_image, cv::COLOR_RGBA2BGR); // Qt的QImage默认使用RGBA格式，而OpenCV默认使用BGR格式，需要进行通道转换
    // cv::resize()
    // 创建一个cv_bridge::CvImage对象
    cv_bridge::CvImage cv_bridge_image;
    cv_bridge_image.image = cv_image;
    cv_bridge_image.encoding = "bgr8"; // 设置图像编码为BGR8

    // 将cv_bridge::CvImage对象转换为sensor_msgs::Image
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_bridge_image.image).toImageMsg();
    return msg;
}
