#include "1.h"

1::1(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui_1)
{
    ui->setupUi(this);
}

1::~1()
{
    delete ui; 
}