#include "weldiosetdlg.h"
#include "ui_weldiosetdlg.h"

WeldIOSetDlg::WeldIOSetDlg(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::WeldIOSetDlg)
{
    ui->setupUi(this);
}

WeldIOSetDlg::~WeldIOSetDlg()
{
    delete ui;
}
