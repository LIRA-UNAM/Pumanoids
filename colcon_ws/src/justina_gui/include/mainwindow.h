#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "rclcomm.h"
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

QT_BEGIN_NAMESPACE
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void btnFwdPressed();
    void btnFwdReleased();
    void btnBwdPressed();    
    void btnBwdReleased();
    void btnTurnLeftPressed();
    void btnTurnLeftReleased();
    void btnTurnRightPressed();
    void btnTurnRightReleased();

    void navBtnCalcPath_pressed();
    void navBtnExecPath_pressed();

    void armSbAnglesValueChanged(double d);
    void armSbGripperValueChanged(double d);
    void armTxtArticularGoalReturnPressed();
    void armTxtCartesianGoalReturnPressed();
    void armBtnXpPressed();
    void armBtnXmPressed();
    void armBtnYpPressed();
    void armBtnYmPressed();
    void armBtnZpPressed();
    void armBtnZmPressed();
    void armBtnRollpPressed();
    void armBtnRollmPressed();
    void armBtnPitchpPressed();
    void armBtnPitchmPressed();
    void armBtnYawpPressed();
    void armBtnYawmPressed();
    void arm_get_IK_and_update_ui(std::vector<double> cartesian);

    void spgTxtSayReturnPressed();
    void sprTxtFakeRecogReturnPressed();

private:
    Ui::MainWindow *ui;
    RclComm *commNode;
};
#endif // MAINWINDOW_H
