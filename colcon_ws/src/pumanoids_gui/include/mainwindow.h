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
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
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

        //Pressed
        void button_front_pressed();
        void button_back_pressed();
        void button_left_pressed();
        void button_right_pressed();
        void button_turn_right_pressed();
        void button_turn_left_pressed();
        void button_center_pressed();

        //Released
        void button_front_released();
        void button_back_released();
        void button_left_released();
        void button_right_released();
        void button_turn_right_released();
        void button_turn_left_released();
        void button_center_released();

    private:
        Ui::MainWindow *ui;
        // ROS Variables
        rclcpp::Node::SharedPtr nh_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_raw_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr vision_image_sub_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  pub_cmd_vel_;
        geometry_msgs::msg::Twist cmd_vel_;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_enable_head_ball_follower;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_enable_base_ball_follower;

        void publish_cmd_vel(double linear_x, double linear_y, double angular);
        void publish_toggle_enable_head_ball_follower();
        void publish_toggle_enable_base_ball_follower();
        void publish_toggle_enable_ball_follower();

        QTimer *ros_timer;        
};

#endif // MAINWINDOW_H
