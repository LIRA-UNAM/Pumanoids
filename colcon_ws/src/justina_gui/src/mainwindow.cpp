#include "mainwindow.hpp"
#include "ui_mainwindow.h"

unsigned int last_index_value = 0;
QFileInfoList ptc_images_list;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    //UI SETUP
    ui->setupUi(this);
 
    ros_timer = new QTimer(this);
    connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
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