#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//C++
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <deque>
#include <cmath>
//QT
#include <QMainWindow>
#include <QTimer>
#include <QThread>
#include <QString>
#include <QPainter>
#include <qtimer.h>
#include <QDateTime>
#include <QTcpSocket>
#include <QFileDialog>
#include <QLayoutItem>
#include <QTimerEvent>
#include <QPen>
#include <QRectF>
#include <QSlider>
#include <QPicture>
#include <QLineEdit>
#include <QMessageBox>
#include <QPropertyAnimation>
#include <QFile>
#include <QProcess>
#include <QTextStream>
#include <QDirIterator>
#include <QtMath>

//ROS
// #include "rclcomm.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp> // Include CvBridge for conversion
#include <opencv2/opencv.hpp> // Include OpenCV
#include "ui_mainwindow.h"

namespace Ui { class MainWindow; }

class MainWindow : public QMainWindow
{
    Q_OBJECT
    public:
        MainWindow(QWidget *parent = nullptr);
        ~MainWindow();
        void raw_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void vision_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    private slots:
        void spinOnce();

    private:
        Ui::MainWindow *ui;
        // ROS Variables
        rclcpp::Node::SharedPtr nh_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_raw_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr vision_image_sub_;
        QTimer *ros_timer;        
};

#endif // MAINWINDOW_H
