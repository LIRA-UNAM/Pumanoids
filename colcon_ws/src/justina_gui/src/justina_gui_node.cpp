#include <QApplication>
#include <QIcon>
#include "mainwindow.hpp"


int main(int argc, char *argv[])
{
  std::cout << "INITIALIZING PUMANOIDS GUI NODE ..." << std::endl;
  rclcpp::init(argc, argv);
  QApplication a(argc, argv);

  MainWindow w; 
  w.show();
  
  return a.exec();
}