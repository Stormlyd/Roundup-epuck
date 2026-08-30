#ifndef LSLIDERBOX_H
#define LSLIDERBOX_H


#include <QWidget>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <QEventLoop>
#include <QVBoxLayout>
#include <QHBoxLayout>


class LSliderBox : public QWidget
{
    Q_OBJECT

public:
    LSliderBox(QWidget *parent = 0);
    ~LSliderBox();

    enum ChosseResult
    {
        ID_OK = 0,                      // 确定;
        ID_CLOSE                        // 取消;
    };

    //--
    void setMax(int max) { pSliderMaxValue = max; }
    int  getMax() { return  pSliderMaxValue; }
    void setMin(int min) { pSliderMinValue = min; }
    int  getMin() { return  pSliderMinValue; }

    void setRange(int min, int max, int step);
    void setRange(int min, int max);   
    int getValue();

    void setTitleText(QString title, int titleFontSize = 10);
    int showBox(const QString &titleText, bool isModelWindow = false);

private:
    void init();
    int exec();

    void paintEvent(QPaintEvent *event);
    void closeEvent(QCloseEvent *event);

private slots:
     void onCloseClicked();
     void onSureClicked();
     void onIncreaseClicked();
     void onReduceClicked();
     void setChangeValue(int);

private:
    QSlider * pSlider;
    QLabel *titleLab;
    QLabel *showIdLable;
    QPushButton *sureBtn;
    QPushButton *closeBtn;
    QPushButton *increaseBtn;
    QPushButton *reduceBtn;

    QEventLoop* m_eventLoop;
    ChosseResult m_chooseResult;

    int pSliderMaxValue;
    int pSliderMinValue;

};


#endif // LSLIDERBOX_H
