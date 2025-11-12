#include "mainwindow.h"

using std::placeholders::_1;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , nh_(new rclcpp::Node("pumanoids_gui_node"))
{
    image_raw_sub_      = nh_->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 1, std::bind(&MainWindow::raw_image_callback, this, _1));
    vision_image_sub_   = nh_->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 1, std::bind(&MainWindow::vision_image_callback, this, _1));
    pub_cmd_vel_        = nh_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    //UI SETUP
    ui->setupUi(this);
    ros_timer = new QTimer(this);
    connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
    connect(ui->button_front        ,SIGNAL(pressed()), this, SLOT(button_front_pressed()));
    connect(ui->button_back         ,SIGNAL(pressed()), this, SLOT(button_back_pressed()));
    connect(ui->button_center       ,SIGNAL(pressed()), this, SLOT(button_center_pressed()));
    connect(ui->button_left         ,SIGNAL(pressed()), this, SLOT(button_left_pressed()));
    connect(ui->button_right        ,SIGNAL(pressed()), this, SLOT(button_right_pressed()));
    connect(ui->button_turn_left    ,SIGNAL(pressed()), this, SLOT(button_turn_left_pressed()));
    connect(ui->button_turn_right   ,SIGNAL(pressed()), this, SLOT(button_turn_right_pressed()));

    connect(ui->button_front        ,SIGNAL(released()), this, SLOT(button_front_released()));
    connect(ui->button_back         ,SIGNAL(released()), this, SLOT(button_back_released()));
    connect(ui->button_center       ,SIGNAL(released()), this, SLOT(button_center_released()));
    connect(ui->button_left         ,SIGNAL(released()), this, SLOT(button_left_released()));
    connect(ui->button_right        ,SIGNAL(released()), this, SLOT(button_right_released()));
    connect(ui->button_turn_left    ,SIGNAL(released()), this, SLOT(button_turn_left_released()));
    connect(ui->button_turn_right   ,SIGNAL(released()), this, SLOT(button_turn_right_released()));
    ros_timer->start(100);
}

MainWindow::~MainWindow()
{
    std::cout<<"Destructor Detected"<<std::endl;
    delete ui;
}

void MainWindow::raw_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat conversion_mat_;
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    conversion_mat_ = cv_ptr->image;
    QImage tmpimage = QImage(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step, QImage::Format_RGB888);
    QPixmap pixmap=QPixmap::fromImage(tmpimage);
    QLabel *label=ui->label_camera_upper;
    label->setPixmap(pixmap);
    label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    label->setScaledContents(true);
}
void MainWindow::vision_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat conversion_mat_;
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    conversion_mat_ = cv_ptr->image;
    QImage tmpimage = QImage(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step, QImage::Format_RGB888);
    QPixmap pixmap=QPixmap::fromImage(tmpimage);
    QLabel *label=ui->label_camera_lower;
    label->setPixmap(pixmap);
    label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    label->setScaledContents(true);
}

void MainWindow::spinOnce()
{
    rclcpp::spin_some(nh_);
}

void MainWindow::button_front_pressed()
{
    RCLCPP_INFO(nh_->get_logger(), "Received front pressed");
    publish_cmd_vel(0.3, 0, 0);
}

void MainWindow::button_back_pressed()
{
    RCLCPP_INFO(nh_->get_logger(), "Received back pressed");
    publish_cmd_vel(-0.3, 0, 0);
}

void MainWindow::button_left_pressed()
{
    RCLCPP_INFO(nh_->get_logger(), "Received left pressed");
    publish_cmd_vel(0, 0.3, 0);
}

void MainWindow::button_right_pressed()
{
    RCLCPP_INFO(nh_->get_logger(), "Received right pressed");
    publish_cmd_vel(0, -0.3, 0);
}

void MainWindow::button_turn_right_pressed()
{
    RCLCPP_INFO(nh_->get_logger(), "Received turn right pressed");
    publish_cmd_vel(0, 0, -0.5);
}

void MainWindow::button_turn_left_pressed()
{
    RCLCPP_INFO(nh_->get_logger(), "Received turn left pressed");
    publish_cmd_vel(0, 0, 0.5);
}

void MainWindow::button_center_pressed()
{
    RCLCPP_INFO(nh_->get_logger(), "Received center pressed");
    publish_cmd_vel(0, 0, 0);
}

void MainWindow::button_front_released()
{
    RCLCPP_INFO(nh_->get_logger(), "Received front released");
    publish_cmd_vel(0, 0, 0);
}

void MainWindow::button_back_released()
{
    RCLCPP_INFO(nh_->get_logger(), "Received back released");
    publish_cmd_vel(0, 0, 0);
}

void MainWindow::button_left_released()
{
    RCLCPP_INFO(nh_->get_logger(), "Received left released");
    publish_cmd_vel(0, 0, 0);
}

void MainWindow::button_right_released()
{
    RCLCPP_INFO(nh_->get_logger(), "Received right released");
    publish_cmd_vel(0, 0, 0);
}

void MainWindow::button_turn_right_released()
{
    RCLCPP_INFO(nh_->get_logger(), "Received turn right released");
    publish_cmd_vel(0, 0, 0);
}

void MainWindow::button_turn_left_released()
{
    RCLCPP_INFO(nh_->get_logger(), "Received turn left released");
    publish_cmd_vel(0, 0, 0);
}

void MainWindow::button_center_released()
{
    RCLCPP_INFO(nh_->get_logger(), "Received center released");
    publish_cmd_vel(0, 0, 0);
}

void MainWindow::publish_cmd_vel(double linear_x, double linear_y, double angular)
{
    cmd_vel_.linear.x = linear_x;
    cmd_vel_.linear.y = linear_y;
    cmd_vel_.angular.z =  angular;
    pub_cmd_vel_->publish(cmd_vel_);
}