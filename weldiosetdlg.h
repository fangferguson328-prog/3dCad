#ifndef WELDIOSETDLG_H
#define WELDIOSETDLG_H

#include <QDialog>

namespace Ui {
class WeldIOSetDlg;
}

class WeldIOSetDlg : public QDialog
{
    Q_OBJECT

public:
    explicit WeldIOSetDlg(QWidget *parent = nullptr);
    ~WeldIOSetDlg();

private:
    Ui::WeldIOSetDlg *ui;
};

#endif // WELDIOSETDLG_H
