#include "mainwindow.h"

using std::placeholders::_1;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , nh_(new rclcpp::Node("pumanoids_gui_node"))
{
    image_raw_sub_ = nh_->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 1, std::bind(&MainWindow::raw_image_callback, this, _1));   
    vision_image_sub_ = nh_->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 1, std::bind(&MainWindow::vision_image_callback, this, _1));
    pub_cmd_vel_ = nh_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pub_enable_head_ball_follower = nh_->create_publisher<std_msgs::msg::Bool>("/planning/head_ball_follower/enable", 10);
    pub_enable_base_ball_follower = nh_->create_publisher<std_msgs::msg::Bool>("/planning/base_ball_follower/enable", 10);
    
    pub_enable_gui = nh_->create_publisher<std_msgs::msg::Bool>("gui/enable", 10); //AÑADIDO: PUBLICADOR PARA BOTÓN BOOLEANO

    ui->setupUi(this);
    
    ros_timer = new QTimer(this);
    connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
    
    connect(ui->button_front,      SIGNAL(pressed()), this, SLOT(button_front_pressed()));
    connect(ui->button_back,       SIGNAL(pressed()), this, SLOT(button_back_pressed()));
    connect(ui->button_center,     SIGNAL(pressed()), this, SLOT(button_center_pressed()));
    connect(ui->button_left,       SIGNAL(pressed()), this, SLOT(button_left_pressed()));
    connect(ui->button_right,      SIGNAL(pressed()), this, SLOT(button_right_pressed()));
    connect(ui->button_turn_left,  SIGNAL(pressed()), this, SLOT(button_turn_left_pressed()));
    connect(ui->button_turn_right, SIGNAL(pressed()), this, SLOT(button_turn_right_pressed()));
    
    connect(ui->button_front,      SIGNAL(released()), this, SLOT(button_front_released()));
    connect(ui->button_back,       SIGNAL(released()), this, SLOT(button_back_released()));
    connect(ui->button_center,     SIGNAL(released()), this, SLOT(button_center_released()));
    connect(ui->button_left,       SIGNAL(released()), this, SLOT(button_left_released()));
    connect(ui->button_right,      SIGNAL(released()), this, SLOT(button_right_released()));
    connect(ui->button_turn_left,  SIGNAL(released()), this, SLOT(button_turn_left_released()));
    connect(ui->button_turn_right, SIGNAL(released()), this, SLOT(button_turn_right_released()));
    
    connect(ui->btn_head_ball_follow_enable, SIGNAL(clicked()), this, SLOT(publish_toggle_enable_head_ball_follower()));
    connect(ui->btn_base_ball_follow_enable, SIGNAL(clicked()), this, SLOT(publish_toggle_enable_base_ball_follower()));
    connect(ui->btn_ball_follow_enable,      SIGNAL(clicked()), this, SLOT(publish_toggle_enable_ball_follower()));
    connect(ui->toggleEnableButton, SIGNAL(clicked()), this, SLOT(on_toggle_enable_clicked())); // AÑADIDO: CONEXIÓN DEL BOTÓN TOGGLE
    ros_timer->start(100);
}

MainWindow::~MainWindow()
{
    std::cout << "Destructor Detected" << std::endl;
    delete ui;
}

void MainWindow::raw_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat conversion_mat_;
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    conversion_mat_ = cv_ptr->image;
    QImage tmpimage = QImage(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step, QImage::Format_RGB888);
    QPixmap pixmap = QPixmap::fromImage(tmpimage);
    QLabel *label = ui->label_camera_upper;
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
    QPixmap pixmap = QPixmap::fromImage(tmpimage);
    QLabel *label = ui->label_camera_lower;
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
    cmd_vel_.angular.z = angular;
    pub_cmd_vel_->publish(cmd_vel_);
}


void MainWindow::publish_toggle_enable_head_ball_follower()
{
    if(ui->btn_head_ball_follow_enable->text().toStdString().find("enable") != std::string::npos)
    {
    }
}

void MainWindow::publish_toggle_enable_base_ball_follower()
{
}

void MainWindow::publish_toggle_enable_ball_follower()
{
}

// ========== AÑADIDO: MANEJO DEL BOTÓN BOOLEANO ==========
void MainWindow::on_toggle_enable_clicked()
{
    static bool gui_enable_state_ = false;     // false = DISABLE (estado inicial), true = ENABLE
    
    // Cambia de DISABLE→ENABLE o ENABLE→DISABLE
    gui_enable_state_ = !gui_enable_state_;
    
    // ===== 2. ACTUALIZAR INTERFAZ GRÁFICA =====
    if (gui_enable_state_) {
        // --- ESTADO: ENABLED (HABILITADO) ---
        
        ui->toggleEnableButton->setText("ENABLE");
        
        // Cambiar estilo a VERDE
        ui->toggleEnableButton->setStyleSheet(
            "QPushButton {"
            "background-color: rgb(40, 167, 69);"  // Verde brillante
            "color: rgb(255, 255, 255);"          
            "border-color: rgb(255, 255, 255);"   
            "}"
            "QPushButton:pressed {"
            "background-color: rgb(30, 126, 52);" // Verde oscuro al presionar
            "}"
        );
        
        // Log informativo
        RCLCPP_INFO(nh_->get_logger(), "GUI Enable: ENABLED (true)");
        
    } else {
        // --- ESTADO: DISABLED (DESHABILITADO) ---
        
        ui->toggleEnableButton->setText("DISABLE");
        
        // Cambiar estilo a ROJO
        ui->toggleEnableButton->setStyleSheet(
            "QPushButton {"
            "background-color: rgb(220, 53, 69);"  // Rojo
            "color: rgb(255, 255, 255);"          
            "border-color: rgb(255, 255, 255);"   
            "}"
            "QPushButton:pressed {"
            "background-color: rgb(180, 35, 50);" // Rojo oscuro al presionar
            "}"
        );
        
        RCLCPP_INFO(nh_->get_logger(), "GUI Enable: DISABLED (false)");// Log informativo
    }
    
    publish_gui_enable_state(gui_enable_state_);
}

void MainWindow::publish_gui_enable_state(bool state)
{
    auto msg = std_msgs::msg::Bool();// Crear mensaje ROS2 de tipo Bool
    msg.data = state;  // Asignar el estado recibido
    
    pub_enable_gui->publish(msg);// Publicar en el topic "gui/enable"
    
    // Mensaje de log para depuración
    RCLCPP_INFO(nh_->get_logger(), "Publicado en gui/enable: %s", state ? "true (HABILITADO)" : "false (DESHABILITADO)");
}
// ========== FIN AÑADIDO ==========