#ifndef IOCONTROL_H
#define IOCONTROL_H

#include <QDialog>
//#include "robot.h"

namespace Ui {
class IOControl;
}

class IOControl : public QDialog
{
    Q_OBJECT

public:
    explicit IOControl(QWidget *parent = nullptr);
    ~IOControl();

private slots:
    void on_checkBox_DO1_stateChanged(int arg1);

    void on_checkBox_DO2_stateChanged(int arg1);

    void on_checkBox_DO3_stateChanged(int arg1);

    void on_checkBox_DO4_stateChanged(int arg1);

    void on_checkBox_DO5_stateChanged(int arg1);

    void on_checkBox_DO6_stateChanged(int arg1);

    void on_checkBox_DO7_stateChanged(int arg1);

    void on_checkBox_DO8_stateChanged(int arg1);

    void on_pushButton_AO2OK_clicked();

    void on_pushButton_AO1OK_clicked();

private:
    Ui::IOControl *ui;
    //robotControl robot;
};

#endif // IOCONTROL_H
