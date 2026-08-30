#include "LCountdownBox.h"

LCountdownBox::LCountdownBox(QWidget *parent) : QWidget(parent)
{
   resize(50,50);
   timeView = new QLabel(this);
   timeView->setFixedSize(260,95);
   timeView->setAlignment(Qt::AlignCenter);
   timeView->setStyleSheet(rounded_lg + border_info_2 +  font_size_lg + font_weight_900 + text_info);

   secondLab= new QLabel(this);
   secondLab->setFixedSize(this->width(),this->height());
   secondLab->setAlignment(Qt::AlignCenter);
   secondLab->setText(QStringLiteral("秒"));
   secondLab->move(210,44);
   secondLab->setStyleSheet(rounded_lg + border_info_2 +  font_size_sm + font_weight_500 + text_info);

   countDownTimer = new QTimer();
   countDownTimer->setInterval(1000);
   QObject::connect(countDownTimer, SIGNAL(timeout()), this, SLOT(countDownTimerRun()));
   hide();

   countTime = 0;
}

LCountdownBox::~LCountdownBox()
{
    delete timeView;
    timeView = nullptr;

    delete secondLab;
    secondLab = nullptr;
}

void LCountdownBox::startCountdown(int time)
{
    show();
    countDownTimer->start();
    countTime = time;
    timeView->setText(getTimeText());

}

int LCountdownBox::getCountTime()
{
    return countTime;
}

QString LCountdownBox::getTimeText()
{
    if(countTime >= 60)
    {
        return QStringLiteral("剩余:") + QString::number(countTime/60) + QStringLiteral("分") + QString::number(countTime%60);
    }
    else
    {
        return QStringLiteral("剩余:") + QString::number(countTime);
    }
}

void LCountdownBox::endCountdown()
{
    countDownTimer->stop();
    hide();
    countTime = 0;
}

void LCountdownBox::countDownTimerRun()
{
    countTime--;
    timeView->setText(getTimeText());
    if(countTime <= 0)
    {
        countDownTimer->stop();
        hide();
        emit countDownEvent();
    }
}
