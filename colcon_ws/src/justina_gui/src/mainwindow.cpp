#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QIcon icoFwd(":/images/btnUp");
    QIcon icoBwd(":/images/btnDown");
    QIcon icoLeft(":/images/btnLeft");
    QIcon icoRight(":/images/btnRight");
    QIcon icoTurnLeft(":/images/btnTurnLeft");
    QIcon icoTurnRight(":/images/btnTurnRight");
    ui->btnFwd->setIcon(icoFwd);
    ui->btnBwd->setIcon(icoBwd);
    ui->btnTurnLeft->setIcon(icoTurnLeft);
    ui->btnTurnRight->setIcon(icoTurnRight);
    //ui->plainTextEdit->setPlainText("hello");
    this->commNode = new RclComm();
    QObject::connect(ui->btnFwd, SIGNAL(pressed()), this, SLOT(btnFwdPressed()));
    QObject::connect(ui->btnFwd, SIGNAL(released()), this, SLOT(btnFwdReleased()));
    QObject::connect(ui->btnBwd, SIGNAL(pressed()), this, SLOT(btnBwdPressed()));
    QObject::connect(ui->btnBwd, SIGNAL(released()), this, SLOT(btnBwdReleased()));
    QObject::connect(ui->btnTurnLeft, SIGNAL(pressed()), this, SLOT(btnTurnLeftPressed()));
    QObject::connect(ui->btnTurnLeft, SIGNAL(released()), this, SLOT(btnTurnLeftReleased()));
    QObject::connect(ui->btnTurnRight, SIGNAL(pressed()), this, SLOT(btnTurnRightPressed()));
    QObject::connect(ui->btnTurnRight, SIGNAL(released()), this, SLOT(btnTurnRightReleased()));
    
    QObject::connect(ui->navTxtStartPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navTxtGoalPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navBtnCalcPath, SIGNAL(clicked()), this, SLOT(navBtnCalcPath_pressed()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::btnFwdPressed()
{
    //commNode->start_publishing_cmd_vel(0.3, 0, 0);
    commNode->publish_cmd_vel(0.3, 0, 0);
}

void MainWindow::btnFwdReleased()
{
    //commNode->stop_publishing_cmd_vel();
    commNode->publish_cmd_vel(0.0, 0, 0);
}

void MainWindow::btnBwdPressed()
{
    //qtRosNode->start_publishing_cmd_vel(-0.3, 0, 0);
    commNode->publish_cmd_vel(-0.3, 0, 0);
}

void MainWindow::btnBwdReleased()
{
    //qtRosNode->stop_publishing_cmd_vel();
    commNode->publish_cmd_vel(0, 0, 0);
}

void MainWindow::btnTurnLeftPressed()
{
    //qtRosNode->start_publishing_cmd_vel(0, 0, 0.5);
    commNode->publish_cmd_vel(0, 0, 0.5);
}

void MainWindow::btnTurnLeftReleased()
{
    //qtRosNode->stop_publishing_cmd_vel();
    commNode->publish_cmd_vel(0, 0, 0);
}

void MainWindow::btnTurnRightPressed()
{
    //qtRosNode->start_publishing_cmd_vel(0, 0, -0.5);
    commNode->publish_cmd_vel(0, 0, -0.5);
}

void MainWindow::btnTurnRightReleased()
{
    //qtRosNode->stop_publishing_cmd_vel();
    commNode->publish_cmd_vel(0, 0, 0);
}


void MainWindow::navBtnCalcPath_pressed()
{
    float startX = 0;
    float startY = 0;
    float startA = 0;
    float goalX = 0;
    float goalY = 0;
    float goalA = 0;
    std::vector<std::string> parts;
    
    std::string str = this->ui->navTxtStartPose->text().toStdString();
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    
    // if(str.compare("") == 0 || str.compare("robot") == 0) //take robot pose as start position
    // {
    //     this->ui->navTxtStartPose->setText("Robot");
    //     commNode->get_robot_pose(startX, startY, startA);
    // }
    // else
    if(parts.size() >= 2) //Given data correspond to numbers
    {
        std::stringstream ssStartX(parts[0]);
        std::stringstream ssStartY(parts[1]);
        if(!(ssStartX >> startX) || !(ssStartY >> startY))
        {
            this->ui->navTxtStartPose->setText("Invalid format");
            return;
        }
    }
    else
    {
	this->ui->navTxtStartPose->setText("Invalid format");
	return;
    }

    str = this->ui->navTxtGoalPose->text().toStdString();
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    if(parts.size() >= 2)
    {
        std::stringstream ssGoalX(parts[0]);
        std::stringstream ssGoalY(parts[1]);
        if(!(ssGoalX >> goalX) || !(ssGoalY >> goalY))
        {
            this->ui->navTxtGoalPose->setText("Invalid format");
            return;
        }
    }
    else
    {
	this->ui->navTxtGoalPose->setText("Invalid format");
	return;
    }
    nav_msgs::msg::Path path, smooth_path;
    if(!commNode->call_plan_path(startX, startY, goalX, goalY, path))
        return;
    //commNode->call_smooth_path(path, smooth_path);
}

void MainWindow::navBtnExecPath_pressed()
{
}

void MainWindow::armSbAnglesValueChanged(double d)
{
}

void MainWindow::armSbGripperValueChanged(double d)
{
}

void MainWindow::armTxtArticularGoalReturnPressed()
{
}

void MainWindow::armTxtCartesianGoalReturnPressed()
{
}

void MainWindow::armBtnXpPressed()
{
}

void MainWindow::armBtnXmPressed()
{
}

void MainWindow::armBtnYpPressed()
{
}

void MainWindow::armBtnYmPressed()
{
}

void MainWindow::armBtnZpPressed()
{
}

void MainWindow::armBtnZmPressed()
{
}

void MainWindow::armBtnRollpPressed()
{
}

void MainWindow::armBtnRollmPressed()
{
}

void MainWindow::armBtnPitchpPressed()
{
}

void MainWindow::armBtnPitchmPressed()
{
}

void MainWindow::armBtnYawpPressed()
{
}

void MainWindow::armBtnYawmPressed()
{
}

void MainWindow::arm_get_IK_and_update_ui(std::vector<double> cartesian)
{
}


void MainWindow::spgTxtSayReturnPressed()
{
}

void MainWindow::sprTxtFakeRecogReturnPressed()
{
}
