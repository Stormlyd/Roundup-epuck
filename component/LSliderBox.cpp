#include "LSliderBox.h"

#include <QPainter>
#include <QMouseEvent>
#include <QGraphicsDropShadowEffect>
#include <QDebug>


#define BUTTON_QSS "QPushButton{font-weight: bold;background-color:rgb(134,183,200,0);border:2px solid #68d8fe;border-radius:5px;color:#68D8fe; padding:8px 45px;}"

#define CLOSE_BUTTON_QSS "background:transparent;margin-right:20px;margin-top:10px;"
#define TITLE_LABEL_QSS "font-size:22px;font-weight: 600; color:#68d8fe; margin:10px 0px 0px 10px;text-ailgn;center;"
#define MR_2 "margin-right:20px; background:transparent;"
#define ML_2 "margin-left:20px; background:transparent;"
#define SLIDER_QSS "QSlider::handle:horizontal{width:30px;height:30px;background-color:#68d8fe;margin:-10px 1px -10px 1px;border-radius:15px;}"\
                            "QSlider::groove:horizontal{height:10px;background-color:rgb(219,219,219);border-radius:10px;}"\
                            "QSlider::add-page:horizontal{background-color:rgb(104,216,254,100);}"\
                            "QSlider::sub-page:horizontal{background-color:rgb(104,216,254,100);}"\


LSliderBox::LSliderBox(QWidget *parent)
    : QWidget(parent), m_eventLoop(NULL), m_chooseResult(ID_CLOSE)
{
    init();
}

LSliderBox::~LSliderBox()
{
    delete pSlider;
    delete sureBtn;
    delete closeBtn;
    delete increaseBtn;
    delete reduceBtn;
}

void LSliderBox::setRange(int min, int max, int step)
{
    setMin(min);
    setMax(max);
    pSlider->setRange(min,max);
    pSlider->setSingleStep(step);
}

void LSliderBox::setRange(int min, int max)
{
     setMin(min);
     setMax(max);
     pSlider->setRange(min,max);
}

int LSliderBox::getValue()
{
    return pSlider->value();
}

void LSliderBox::setTitleText(QString title, int titleFontSize)
{
       titleLab->setText(title);
}

void LSliderBox::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    painter.drawImage(QRect(5, 5, this->width()-10, this->height()-10), QImage(":/new/ofapp/res/images/messagebox_bg.png"));

    painter.drawImage(QRect(0, 0, 60, 61), QImage(":/new/ofapp/res/images/messagebox_left_top.png"));
    painter.drawImage(QRect(width()-122, 2, 122, 65), QImage(":/new/ofapp/res/images/messagebox_right_top.png"));
    painter.drawImage(QRect(0, height()-65, 122, 63), QImage(":/new/ofapp/res/images/messagebox_left_bottom.png"));
    painter.drawImage(QRect(width()-60, height()-61, 60, 61), QImage(":/new/ofapp/res/images/messagebox_right_bottom.png"));

    painter.drawImage(QRect(60, 0, width()-182, 7), QImage(":/new/ofapp/res/images/messagebox_top.png"));
    painter.drawImage(QRect(0, 61, 14, height()-126), QImage(":/new/ofapp/res/images/messagebox_left.png"));
    painter.drawImage(QRect(122, height()-7, width()-182, 7), QImage(":/new/ofapp/res/images/messagebox_bottom.png"));
    painter.drawImage(QRect(width()-14,67, 14, height()-128), QImage(":/new/ofapp/res/images/messagebox_right.png"));
    return QWidget::paintEvent(event);
}

void LSliderBox::closeEvent(QCloseEvent *event)
{
    if (m_eventLoop != NULL)
    {
        m_eventLoop->exit();
    }
    event->accept();
}

void LSliderBox::onCloseClicked()
{
    close();
}

//--slot
void LSliderBox::onSureClicked()
{
    m_chooseResult=ID_OK;
    close();
}

void LSliderBox::onIncreaseClicked()
{
      if(pSlider->value()<getMax())
      {
          pSlider->setValue(pSlider->value()+1);
      }
}

void LSliderBox::onReduceClicked()
{
    if(pSlider->value()>getMin())
    {
        pSlider->setValue(pSlider->value()-1);
    }
}

void LSliderBox::setChangeValue( int )
{
    showIdLable->setText(QString::number(pSlider->value()));
    showIdLable->setAlignment(Qt::AlignCenter);
}//slot  end


int LSliderBox::showBox( const QString &titleText, bool isModelWindow)
{
    setTitleText(titleText);

    if (isModelWindow)
    {
        return exec();
    }
    else
    {
        show();
    }

    return -1;
}

void LSliderBox::init()
{
    setWindowFlags(Qt::Window|Qt::FramelessWindowHint);
    setAttribute(Qt::WA_TranslucentBackground, true);
    resize(400,230);
    //整体垂直布局
    QVBoxLayout *layout = new QVBoxLayout();
    setLayout(layout);


    //标题
    QHBoxLayout *topLayout = new QHBoxLayout();
    layout->addLayout(topLayout);

    titleLab = new QLabel(/*QStringLiteral("提示")*/);
    titleLab->setStyleSheet(TITLE_LABEL_QSS);

    //关闭按钮
    closeBtn = new QPushButton();
    QIcon closeBtnIco(":/new/ofapp/res/images/close.png");
    closeBtn->setIcon(closeBtnIco);
    closeBtn->setCursor(Qt::PointingHandCursor);
    closeBtn->setStyleSheet(CLOSE_BUTTON_QSS);

    topLayout->addWidget(titleLab);
    topLayout->addStretch();
    topLayout->addWidget(closeBtn);

    //ID 显示
    QHBoxLayout *idShowLoyout= new QHBoxLayout();
    layout->addLayout(idShowLoyout);

    showIdLable=new QLabel();
    showIdLable->setStyleSheet(TITLE_LABEL_QSS);
    showIdLable->setText("0");
    idShowLoyout->addStretch();
    idShowLoyout->addWidget(showIdLable);
    idShowLoyout->addStretch();

    //增加按钮 减少按钮  slider布局
     QHBoxLayout *middleLoyout= new QHBoxLayout();
     layout->addLayout(middleLoyout);

    //增加按钮
    increaseBtn=new QPushButton();
    QIcon increaseBtnIco(":/new/ofapp/res/images/add.png");
    increaseBtn->setIcon(increaseBtnIco);
    increaseBtn->setCursor(Qt::PointingHandCursor);
    increaseBtn->setStyleSheet(MR_2);

    //减少按钮
    reduceBtn = new QPushButton();
    QIcon reduceBtnIco(":/new/ofapp/res/images/sub.png");
    reduceBtn->setIcon(reduceBtnIco);
    reduceBtn->setCursor(Qt::PointingHandCursor);
    reduceBtn->setStyleSheet(ML_2);

    //slider
    pSlider = new QSlider(this);
    pSlider->setOrientation(Qt::Horizontal);
    pSlider->setFixedHeight(35);
    pSlider->setStyleSheet(SLIDER_QSS);


    middleLoyout->addWidget(reduceBtn);
    middleLoyout->addWidget(pSlider);
    middleLoyout->addWidget(increaseBtn);

    layout->addStretch();

    //确定按钮
    QHBoxLayout *bottomLayout = new QHBoxLayout();
    layout->addLayout(bottomLayout);

    sureBtn=new QPushButton();
    sureBtn->setText(QStringLiteral("确定"));
    sureBtn->setFlat(true);
    sureBtn->setStyleSheet(BUTTON_QSS);
    sureBtn->setCursor(Qt::PointingHandCursor);

    bottomLayout->addStretch();
    bottomLayout->addWidget(sureBtn);
    bottomLayout->addStretch();
    layout->addStretch();  //底部加一弹簧

    //--
    QObject::connect(sureBtn, SIGNAL(clicked()), this, SLOT(onSureClicked()));
    QObject::connect(closeBtn, SIGNAL(clicked()), this, SLOT(onCloseClicked()));
    QObject::connect(pSlider, SIGNAL(valueChanged(int)), this, SLOT(setChangeValue(int)));
    QObject::connect(increaseBtn, SIGNAL(clicked()), this, SLOT(onIncreaseClicked()));
    QObject::connect(reduceBtn, SIGNAL(clicked()), this, SLOT(onReduceClicked()));
}

int LSliderBox::exec()
{
    this->setWindowModality(Qt::WindowModal);
    show();
    m_eventLoop = new QEventLoop(this);
    m_eventLoop->exec();
    return m_chooseResult;
}



