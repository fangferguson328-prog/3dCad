#include "iocontrol.h"
#include "ui_iocontrol.h"

IOControl::IOControl(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::IOControl)
{
   ui->setupUi(this);
    //if (robot.sendLoginCommandPack() == false)
    //{
    //    qDebug()<<"pCTcpClientControlOperate.robotServiceLogin() == false\n";
    //    return;
    //}
    //else {
    //    qDebug()<<"Contro 登录成功 \n";
    //}

    //if (robot.sendResetErrorCommandPack(1) == false)
    //    {
    //        qDebug()<<"pCTcpClientControlOperate.sendResetErrorCommandPack() == false\n";
    //        return;
    //    }
    //    else {
    //        qDebug()<<"清错 \n";
    //    }
}

IOControl::~IOControl()
{
    delete ui;
}

void IOControl::on_checkBox_DO1_stateChanged(int arg1)
{
    //if(arg1)
    //{
    //    if(robot.sendIoWriteCommandPack(0, 2, 1))
    //        qDebug()<<"DO1 = high";
    //    else
    //        qDebug()<<"DO1 high defeat";
    //}
    //else
    //    if(robot.sendIoWriteCommandPack(0, 2, 0))
    //        qDebug()<<"DO1 = low";
    //    else
    //        qDebug()<<"DO1 low defeat";

}

void IOControl::on_checkBox_DO2_stateChanged(int arg1)
{
    //if(arg1)
    //{
    //    if(robot.sendIoWriteCommandPack(1, 2, 1))
    //        qDebug()<<"DO2 = high";
    //    else
    //        qDebug()<<"DO2 high defeat";
    //}
    //else
    //    if(robot.sendIoWriteCommandPack(1, 2, 0))
    //        qDebug()<<"DO2 = low";
    //    else
    //        qDebug()<<"DO2 low defeat";

}

void IOControl::on_checkBox_DO3_stateChanged(int arg1)
{
    //if(arg1)
    //{
    //    if(robot.sendIoWriteCommandPack(2, 2, 1))
    //        qDebug()<<"DO3 = high";
    //    else
    //        qDebug()<<"DO3 high defeat";
    //}
    //else
    //    if(robot.sendIoWriteCommandPack(2, 2, 0))
    //        qDebug()<<"DO3 = low";
    //    else
    //        qDebug()<<"DO3 low defeat";
}

void IOControl::on_checkBox_DO4_stateChanged(int arg1)
{
    //if(arg1)
    //{
    //    if(robot.sendIoWriteCommandPack(3, 2, 1))
    //        qDebug()<<"DO4 = high";
    //    else
    //        qDebug()<<"DO4 high defeat";
    //}
    //else
    //    if(robot.sendIoWriteCommandPack(3, 2, 0))
    //        qDebug()<<"DO4 = low";
    //    else
    //        qDebug()<<"DO4 low defeat";
}

void IOControl::on_checkBox_DO5_stateChanged(int arg1)
{
    //if (arg1)
    //{
    //    if (robot.sendIoWriteCommandPack(4, 2, 1))
    //        qDebug() << "DO5 = high";
    //    else
    //        qDebug() << "DO5 high defeat";
    //}
    //else
    //    if (robot.sendIoWriteCommandPack(4, 2, 0))
    //        qDebug() << "DO5 = low";
    //    else
    //        qDebug() << "DO5 low defeat";
}

void IOControl::on_checkBox_DO6_stateChanged(int arg1)
{
    //if (arg1)
    //{
    //    if (robot.sendIoWriteCommandPack(5, 2, 1))
    //        qDebug() << "DO6 = high";
    //    else
    //        qDebug() << "DO6 high defeat";
    //}
    //else
    //    if (robot.sendIoWriteCommandPack(5, 2, 0))
    //        qDebug() << "DO6 = low";
    //    else
    //        qDebug() << "DO6 low defeat";
}

void IOControl::on_checkBox_DO7_stateChanged(int arg1)
{
    //if (arg1)
    //{
    //    if (robot.sendIoWriteCommandPack(6, 2, 1))
    //        qDebug() << "DO7 = high";
    //    else
    //        qDebug() << "DO7 high defeat";
    //}
    //else
    //    if (robot.sendIoWriteCommandPack(6, 2, 0))
    //        qDebug() << "DO7 = low";
    //    else
    //        qDebug() << "DO7 low defeat";
}

void IOControl::on_checkBox_DO8_stateChanged(int arg1)
{
  /*  if (arg1)
    {
        if (robot.sendIoWriteCommandPack(7, 2, 1))
            qDebug() << "DO8 = high";
        else
            qDebug() << "DO8 high defeat";
    }
    else
        if (robot.sendIoWriteCommandPack(7, 2, 0))
            qDebug() << "DO8 = low";
        else
            qDebug() << "DO8 low defeat";*/
}




void IOControl::on_pushButton_AO1OK_clicked()
{
    //double val; //AO1的值
    //val = ui->doubleSpinBox_AO1->value();
    //if (ui->comboBox_3->currentIndex() == 0)
    //{
    //    if (robot.sendIoWriteCommandPack(0, 6, 0))
    //    {
    //        qDebug() << "AO1 电压型输出 " << val;
    //        if (robot.sendIoWriteCommandPack(0, 4, val))
    //            qDebug() << "AO1 = " << val;
    //        else 
    //            qDebug() << "AO1  电压 defeat";

    //    }
    //}
    //else
    //{
    //    if (robot.sendIoWriteCommandPack(0, 6, 1))
    //    {
    //        qDebug() << "AO1 电流型输出 " << val;
    //        if (robot.sendIoWriteCommandPack(0, 4, val))
    //            qDebug() << "AO1 = " << val;
    //        else
    //            qDebug() << "AO1 电流 defeat";

    //    }
    //}
}

void IOControl::on_pushButton_AO2OK_clicked()
{
    //float val; //AO2的值
    //val = ui->doubleSpinBox_AO2->value();
    //if (ui->comboBox_4->currentIndex() == 0)
    //{
    //    if (robot.sendIoWriteCommandPack(1, 6, 0))
    //    {
    //        qDebug() << "AO2 电压型输出 " << val;
    //        if (robot.sendIoWriteCommandPack(1, 4, val))
    //            qDebug() << "AO2 = " << val;
    //        else
    //            qDebug() << "AO2  defeat";

    //    }
    //}
    //else
    //{
    //    if (robot.sendIoWriteCommandPack(1, 6, 1))
    //    {
    //        qDebug() << "AO2 电流型输出 " << val;
    //        if (robot.sendIoWriteCommandPack(1, 4, val))
    //            qDebug() << "AO2 = " << val;
    //        else
    //            qDebug() << "AO2 电流 defeat";

    //    }
    }


