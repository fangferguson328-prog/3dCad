#include "mainwindow.h"
#include <QCoreApplication>
#include <QApplication> // 或 QGuiApplication

int main(int argc, char *argv[])
{
   

    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}


//
//#include"XPLGenerator.h"
//#include<iostream>
//#include <string>
//
////测试XPLGenerator
//int main() {
//    XPLGenerator generator; // 创建XPLGenerator实例
//	generator.addJointMotion({ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }, "v100", "fine"); // 添加关节运动指令
//
//	generator.addJointMotion({ 1.0, 2.0, 3.0, 4.0, 5.0, 0.0 }, "v100", "fine"); // 添加关节运动指令
//	generator.addArcStart(1); // 添加焊接起弧指令
//	generator.addLinearMotion({ 100.0, 200.0, 300.0 }, { 0.0, 0.0, 0.0 }, "v100", "fine"); // 添加直线运动指令
//	generator.addCircularMotion({ 150.0, 250.0, 350.0 }, { 0.0, 0.0, 0.0 }, { 200.0, 300.0, 400.0 }, { 0.0, 0.0, 0.0 }, "v100", "fine"); // 添加圆弧运动指令
//	
//	generator.addArcEnd(); // 添加焊接收弧指令
//
//	generator.saveToFile("output.xpl"); // 保存到文件
//
//	
//	generator.modifyJointMotion({ 10.0, 20.0, 30.0, 40.0, 50.0, 60.0 }, "v90", "fine"); // 修改关节运动指令
//	generator.saveToFile("modified_output.xpl"); // 保存修改后的文件
//    return 0;
//}