#include "component/LFollowModeBox.h"
#include <QVBoxLayout>
#include <QMessageBox>
#include <QPainter>

#define TITLE_LABEL_QSS "font-size:22px;font-weight: 600; color:#68d8fe; margin:10px 0px 0px 10px;"
#define CLOSE_BUTTON_QSS "background:transparent;margin-right:20px;margin-top:10px;"
#define OK_BUTTON_QSS "font-weight: bold;font-size:18px;background-color:rgb(134,183,200,0);border:2px solid #68d8fe;border-radius:18px;color:#68D8fe; padding:8px 20px;"
#define LOOK_BUTTON_QSS  "font-weight: bold;font-size:16px;color:#68D8fe;background-color:rgb(134,183,200,0);"

/**
 * @brief LFollowModelBox::LFollowModelBox
 * @param parent
 * @author ZengXiang
 */

LFollowModelBox::LFollowModelBox(QWidget *parent) : QWidget(parent)
, m_chooseResult(BUTTON_ZONE)
{
    init();
}

/**
 * @brief LFollowModelBox::~LFollowModelBox
 */

LFollowModelBox::~LFollowModelBox()
{

}

void LFollowModelBox::setTitle(QString title)
{
    titlelabel->setText(title);
}

int LFollowModelBox::showBox(const QString & titleText, bool isModelWindow)
{
    setTitle(titleText);
    if (isModelWindow)
    {
        return  exec();
    }
    else
    {
        show();
    }
    return -1;
}

void LFollowModelBox::init()
{
    setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
     setAttribute(Qt::WA_TranslucentBackground, true);
    resize(638, 178);

    //整体网格布局
    QGridLayout *gridLayout = new QGridLayout(this);

    //标题-关闭-部分
    QHBoxLayout *titleCloseLay = new QHBoxLayout();

    titlelabel = new QLabel(this);
    titlelabel->setText(QStringLiteral("titlelabel"));
    titlelabel->setStyleSheet(TITLE_LABEL_QSS);
    titleCloseLay->addWidget(titlelabel);  //将title标签放入水平布局

    //在title 和button之间加上弹簧
    QSpacerItem *titleAndCloseSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
    titleCloseLay->addItem(titleAndCloseSpacer);  //将之放入水平布局中

     //关闭
    closeBtn = new QPushButton(this);
    QIcon closeBtnIco(":/new/ofapp/res/images/close.png");
    closeBtn->setIcon(closeBtnIco);
    closeBtn->setCursor(Qt::PointingHandCursor);
    closeBtn->setStyleSheet(CLOSE_BUTTON_QSS);
    titleCloseLay->addWidget(closeBtn);

    gridLayout->addLayout(titleCloseLay, 0, 0, 1, 3);  //网格

    //确定按钮与标题-关闭-部分
    //上弹簧
    QSpacerItem *okBtnTopSpacer = new QSpacerItem(20, 29, QSizePolicy::Minimum, QSizePolicy::Expanding);
    gridLayout->addItem(okBtnTopSpacer, 1, 1, 1, 1); //放入网格中

    //okBtn 左边弹簧
    QSpacerItem *okBtnLeftSpacer = new QSpacerItem(264, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
    gridLayout->addItem(okBtnLeftSpacer, 2, 0, 1, 1);

    QPushButton *okBtn = new QPushButton(this);
    okBtn->setText(QStringLiteral("确定"));
    okBtn->setCursor(Qt::PointingHandCursor);
    okBtn->setStyleSheet(OK_BUTTON_QSS);
    gridLayout->addWidget(okBtn, 2, 1, 1, 1); //将okBtn放入网格布局

    //okBtn 右边弹簧
    QSpacerItem *okBtnRightSpacer = new QSpacerItem(263, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
    gridLayout->addItem(okBtnRightSpacer, 2, 2, 1, 1);

    //okBtn  下边弹簧
    QSpacerItem *okBtnBottomSpacer = new QSpacerItem(20, 28, QSizePolicy::Minimum, QSizePolicy::Expanding);
    gridLayout->addItem(okBtnBottomSpacer, 3, 1, 1, 1);

    //底部
    QHBoxLayout *horBottomLayout = new QHBoxLayout();
    //弹簧
    QSpacerItem *lookSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
    horBottomLayout->addItem(lookSpacer);
    //按钮
    QPushButton *lookBtn = new QPushButton(this);
    lookBtn->setText(QString("查看演示"));
    lookBtn->setCursor(Qt::PointingHandCursor);
    lookBtn->setStyleSheet(LOOK_BUTTON_QSS);
    horBottomLayout->addWidget(lookBtn);

    gridLayout->addLayout(horBottomLayout, 4, 0, 1, 3);

    QObject::connect(closeBtn, SIGNAL(clicked()), this, SLOT(onCloseClicked()));
    QObject::connect(okBtn, SIGNAL(clicked()), this, SLOT(onOkBtnClicked()));
    QObject::connect(lookBtn, SIGNAL(clicked()), this, SLOT(onLookClicked()));

}


//--------------------------------------------------------------
void LFollowModelBox::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

    painter.drawImage(QRect(5, 5, this->width() - 10, this->height() - 10), QImage(":/new/ofapp/res/images/messagebox_bg.png"));

    painter.drawImage(QRect(0, 0, 60, 61), QImage(":/new/ofapp/res/images/messagebox_left_top.png"));
    painter.drawImage(QRect(width() - 122, 2, 122, 65), QImage(":/new/ofapp/res/images/messagebox_right_top.png"));
    painter.drawImage(QRect(0, height() - 65, 122, 63), QImage(":/new/ofapp/res/images/messagebox_left_bottom.png"));
    painter.drawImage(QRect(width() - 60, height() - 61, 60, 61), QImage(":/new/ofapp/res/images/messagebox_right_bottom.png"));

    painter.drawImage(QRect(60, 0, width() - 182, 7), QImage(":/new/ofapp/res/images/messagebox_top.png"));
    painter.drawImage(QRect(0, 61, 14, height() - 126), QImage(":/new/ofapp/res/images/messagebox_left.png"));
    painter.drawImage(QRect(122, height() - 7, width() - 182, 7), QImage(":/new/ofapp/res/images/messagebox_bottom.png"));
    painter.drawImage(QRect(width() - 14, 67, 14, height() - 128), QImage(":/new/ofapp/res/images/messagebox_right.png"));
}
//--slot
void LFollowModelBox::onCloseClicked()
{
    close();
}

void LFollowModelBox::onOkBtnClicked()
{
    m_chooseResult = BUTTON_OK;
    close();
}

void LFollowModelBox::onLookClicked()
{
    QMessageBox::critical(NULL, "critical", QString("查看演示"), QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
} //slot end


int LFollowModelBox::exec()
{
    this->setWindowModality(Qt::WindowModal);
    show();
    m_eventLoop = new QEventLoop(this);
    m_eventLoop->exec();
    return  m_chooseResult;
}


void LFollowModelBox::closeEvent(QCloseEvent *event)
{
    if (m_eventLoop != nullptr)
    {
        m_eventLoop->exit();
    }
    event->accept();
}
