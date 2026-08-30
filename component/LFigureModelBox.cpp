#include "LFigureModelBox.h"

LFigureModelBox::LFigureModelBox(QWidget *parent) : QWidget(parent),
    m_eventLoop(nullptr)
{

    m_chooseResult = Cirecul;
    init();
}

LFigureModelBox::~LFigureModelBox()
{
    delete m_eventLoop;
    m_eventLoop = nullptr;

    delete titleLab;
    titleLab = nullptr;

    delete closeBtn;
    closeBtn = nullptr;

    delete okayBtn;
    okayBtn = nullptr;
}

void LFigureModelBox::init()
{
    setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
    setWindowModality(Qt::WindowModal);
    resize(960, 600);
    move(50,118);

    //标题
    titleLab = new QLabel(QStringLiteral("图形模式"));
    titleLab->setParent(this);
    titleLab->setStyleSheet(font_size_lg + font_weight_600 + text_info);
    titleLab->move(18,15);

    //关闭按钮
    closeBtn = new QPushButton(this);
    closeBtn->setIcon(QIcon(":/new/ofapp/res/images/close.png"));
    closeBtn->setCursor(Qt::PointingHandCursor);
    closeBtn->setStyleSheet(transparent_bg);
    closeBtn->move(width()-50,15);

    //确定按钮
    okayBtn = new QPushButton(QStringLiteral("确定"));
    okayBtn->setParent(this);
    okayBtn->setCursor(Qt::PointingHandCursor);
    okayBtn->setStyleSheet(definiteBtnStyle);
    okayBtn->setFixedSize(128,48);
    okayBtn->move(416, 530);
    okayBtn->hide();

    btn1 = new QPushButton(this);
    btn1->setParent(this);
    btn1->setFixedSize(160,160);
    btn1->move(80, 60);
    this->setButtonBackground(btn1, ":/new/ofapp/res/images/circul2.png");
    lab1 = new QLabel(QStringLiteral("圆形"));
    lab1->setParent(this);
    lab1->setFixedSize(160,32);
    lab1->move(80, 220);
    lab1->setAlignment(Qt::AlignCenter);
    lab1->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);

    btn2 = new QPushButton(this);
    btn2->setParent(this);
    btn2->setFixedSize(160,160);
    btn2->move(400, 60);
    this->setButtonBackground(btn2, ":/new/ofapp/res/images/triangle2.png");
    lab2 = new QLabel(QStringLiteral("三角形"));
    lab2->setParent(this);
    lab2->setFixedSize(160,32);
    lab2->move(400, 220);
    lab2->setAlignment(Qt::AlignCenter);
    lab2->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);

    btn3 = new QPushButton(this);
    btn3->setParent(this);
    btn3->setFixedSize(160,160);
    btn3->move(720, 60);
    this->setButtonBackground(btn3, ":/new/ofapp/res/images/react2.png");
    lab3 = new QLabel(QStringLiteral("矩形"));
    lab3->setParent(this);
    lab3->setFixedSize(160,32);
    lab3->move(720, 220);
    lab3->setAlignment(Qt::AlignCenter);
    lab3->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);

    btn4 = new QPushButton(this);
    btn4->setParent(this);
    btn4->setFixedSize(160,160);
    btn4->move(80, 300);
    this->setButtonBackground(btn4, ":/new/ofapp/res/images/cross2.png");
    lab4 = new QLabel(QStringLiteral("十字"));
    lab4->setParent(this);
    lab4->setFixedSize(160,32);
    lab4->move(80, 460);
    lab4->setAlignment(Qt::AlignCenter);
    lab4->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);

    btn5 = new QPushButton(this);
    btn5->setParent(this);
    btn5->setFixedSize(160,160);
    btn5->move(400, 300);
    this->setButtonBackground(btn5, ":/new/ofapp/res/images/hexagon2.png");
    lab5 = new QLabel(QStringLiteral("六边形"));
    lab5->setParent(this);
    lab5->setFixedSize(160,32);
    lab5->move(400, 460);
    lab5->setAlignment(Qt::AlignCenter);
    lab5->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);

    btn6 = new QPushButton(this);
    btn6->setParent(this);
    this->setButtonBackground(btn6, ":/new/ofapp/res/images/fivepointed2.png");
    btn6->setFixedSize(160,160);
    btn6->move(720, 300);
    lab6 = new QLabel(QStringLiteral("五角星"));
    lab6->setParent(this);
    lab6->setFixedSize(160,32);
    lab6->move(720, 460);
    lab6->setAlignment(Qt::AlignCenter);
    lab6->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);

    QObject::connect(closeBtn, SIGNAL(clicked()), this, SLOT(onCloseClicked()));
    QObject::connect(okayBtn, SIGNAL(clicked()), this, SLOT(onOkayClicked()));

    QObject::connect(btn1, SIGNAL(clicked()), this, SLOT(onBtn1Clicked()));
    QObject::connect(btn2, SIGNAL(clicked()), this, SLOT(onBtn2Clicked()));
    QObject::connect(btn3, SIGNAL(clicked()), this, SLOT(onBtn3Clicked()));
    QObject::connect(btn4, SIGNAL(clicked()), this, SLOT(onBtn4Clicked()));
    QObject::connect(btn5, SIGNAL(clicked()), this, SLOT(onBtn5Clicked()));
    QObject::connect(btn6, SIGNAL(clicked()), this, SLOT(onBtn6Clicked()));
}

/**
 * @brief LFigureModelBox::setButtonBackground
 * @param btn
 * @param url
 */
void LFigureModelBox::setButtonBackground(QPushButton *btn, QString url)
{
    btn->setStyleSheet( transparent_bg + "border-image:url("+ url +");" );
}

void LFigureModelBox::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    QPainterPath pathBack;
    pathBack.setFillRule(Qt::WindingFill);
    pathBack.addRect(QRect(0, 0, this->width(), this->height()));
    painter.fillPath(pathBack, QBrush(QColor("#101129")));
    painter.drawPixmap(QRect(0, 0,42, 42), QPixmap(":/new/ofapp/res/images/border2_lt.png"));
    painter.drawPixmap(QRect(width()-42, 0,42, 42), QPixmap(":/new/ofapp/res/images/border2_rt.png"));
    painter.drawPixmap(QRect(0, height()-42,42, 42), QPixmap(":/new/ofapp/res/images/border2_lb.png"));
    painter.drawPixmap(QRect(width()-42, height()-42,42, 42), QPixmap(":/new/ofapp/res/images/border2_rb.png"));
    painter.drawPixmap(QRect(42, 0,width()-84, 20), QPixmap(":/new/ofapp/res/images/line_t.png"));
    painter.drawPixmap(QRect(0, 42,20, height()-84), QPixmap(":/new/ofapp/res/images/line_l.png"));
    painter.drawPixmap(QRect(width()-20, 42,20, height()-84), QPixmap(":/new/ofapp/res/images/line_r.png"));
    painter.drawPixmap(QRect(42, height()-20,width()-84, 20), QPixmap(":/new/ofapp/res/images/line_b.png"));
}

void LFigureModelBox::setSelectPixmap(QString url){
    QPixmap pixmap(url);
    selectPixmap = pixmap;
}

QPixmap LFigureModelBox::getSelectPixmap(){
    return selectPixmap;
}

void LFigureModelBox::closeEvent(QCloseEvent *event)
{
    if (m_eventLoop != nullptr)
    {
        m_eventLoop->exit();
    }
    event->accept();
}

int LFigureModelBox::showBox()
{
    show();
    m_eventLoop = new QEventLoop(this);
    m_eventLoop->exec();
    return m_buttonResult;
}

void LFigureModelBox::onCloseClicked()
{
    m_buttonResult = Close;
    close();
}

void LFigureModelBox::onOkayClicked()
{
    m_buttonResult = Ok;
    close();
}

void LFigureModelBox::onBtn1Clicked(){
    okayBtn->show();
    setChooseResult(Cirecul);
    lab1->setStyleSheet( transparent_bg + font_size_md + text_select + font_weight_500);
    lab2->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab3->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab4->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab5->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab6->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);

    this->setButtonBackground(btn1,":/new/ofapp/res/images/circul.png");
    this->setButtonBackground(btn2,":/new/ofapp/res/images/triangle2.png");
    this->setButtonBackground(btn3,":/new/ofapp/res/images/react2.png");
    this->setButtonBackground(btn4,":/new/ofapp/res/images/cross2.png");
    this->setButtonBackground(btn5,":/new/ofapp/res/images/hexagon2.png");
    this->setButtonBackground(btn6,":/new/ofapp/res/images/fivepointed2.png");
    setSelectPixmap(":/new/ofapp/res/images/circul2.png");
}

void LFigureModelBox::onBtn2Clicked(){
    okayBtn->show();
    setChooseResult(Triangle);
    lab1->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab2->setStyleSheet( transparent_bg + font_size_md + text_select + font_weight_500);
    lab3->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab4->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab5->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab6->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);

    this->setButtonBackground(btn1,":/new/ofapp/res/images/circul2.png");
    this->setButtonBackground(btn2,":/new/ofapp/res/images/triangle.png");
    this->setButtonBackground(btn3,":/new/ofapp/res/images/react2.png");
    this->setButtonBackground(btn4,":/new/ofapp/res/images/cross2.png");
    this->setButtonBackground(btn5,":/new/ofapp/res/images/hexagon2.png");
    this->setButtonBackground(btn6,":/new/ofapp/res/images/fivepointed2.png");

    setSelectPixmap(":/new/ofapp/res/images/triangle2.png");
}
void LFigureModelBox::onBtn3Clicked(){
    okayBtn->show();
    setChooseResult(React);
    lab1->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab2->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab3->setStyleSheet( transparent_bg + font_size_md + text_select + font_weight_500);
    lab4->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab5->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab6->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);

    this->setButtonBackground(btn1,":/new/ofapp/res/images/circul2.png");
    this->setButtonBackground(btn2,":/new/ofapp/res/images/triangle2.png");
    this->setButtonBackground(btn3,":/new/ofapp/res/images/react.png");
    this->setButtonBackground(btn4,":/new/ofapp/res/images/cross2.png");
    this->setButtonBackground(btn5,":/new/ofapp/res/images/hexagon2.png");
    this->setButtonBackground(btn6,":/new/ofapp/res/images/fivepointed2.png");
    setSelectPixmap(":/new/ofapp/res/images/react2.png");
}
void LFigureModelBox::onBtn4Clicked(){
    okayBtn->show();
    setChooseResult(Cross);
    lab1->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab2->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab3->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab4->setStyleSheet( transparent_bg + font_size_md + text_select + font_weight_500);
    lab5->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab6->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);

    this->setButtonBackground(btn1,":/new/ofapp/res/images/circul2.png");
    this->setButtonBackground(btn2,":/new/ofapp/res/images/triangle2.png");
    this->setButtonBackground(btn3,":/new/ofapp/res/images/react2.png");
    this->setButtonBackground(btn4,":/new/ofapp/res/images/cross.png");
    this->setButtonBackground(btn5,":/new/ofapp/res/images/hexagon2.png");
    this->setButtonBackground(btn6,":/new/ofapp/res/images/fivepointed2.png");
    setSelectPixmap(":/new/ofapp/res/images/cross2.png");
}

void LFigureModelBox::onBtn5Clicked(){
    okayBtn->show();
    setChooseResult(Hexagon);
    lab1->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab2->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab3->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab4->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab5->setStyleSheet( transparent_bg + font_size_md + text_select + font_weight_500);
    lab6->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    this->setButtonBackground(btn1,":/new/ofapp/res/images/circul2.png");
    this->setButtonBackground(btn2,":/new/ofapp/res/images/triangle2.png");
    this->setButtonBackground(btn3,":/new/ofapp/res/images/react2.png");
    this->setButtonBackground(btn4,":/new/ofapp/res/images/cross2.png");
    this->setButtonBackground(btn5,":/new/ofapp/res/images/hexagon.png");
    this->setButtonBackground(btn6,":/new/ofapp/res/images/fivepointed2.png");
    setSelectPixmap(":/new/ofapp/res/images/hexagon2.png");

}
void LFigureModelBox::onBtn6Clicked(){
    okayBtn->show();
    setChooseResult(Fivepointed);
    lab1->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab2->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab3->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab4->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab5->setStyleSheet( transparent_bg + font_size_md + text_info + font_weight_500);
    lab6->setStyleSheet( transparent_bg + font_size_md + text_select + font_weight_500);

    this->setButtonBackground(btn1,":/new/ofapp/res/images/circul2.png");
    this->setButtonBackground(btn2,":/new/ofapp/res/images/triangle2.png");
    this->setButtonBackground(btn3,":/new/ofapp/res/images/react2.png");
    this->setButtonBackground(btn4,":/new/ofapp/res/images/cross2.png");
    this->setButtonBackground(btn5,":/new/ofapp/res/images/hexagon2.png");
    this->setButtonBackground(btn6,":/new/ofapp/res/images/fivepointed.png");
     setSelectPixmap(":/new/ofapp/res/images/fivepointed2.png");
}

