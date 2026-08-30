#ifndef LNUMBER_INPUT_BOX
#define LNUMBER_INPUT_BOX

#include <QWidget>
#include <QEventLoop>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QLineEdit>

#include "public/style.h"

class LNumberInputBox : public QWidget
{
    Q_OBJECT
public:
    /**
     * @brief LNumberInputBox
     * @param parent
     */
    LNumberInputBox(QWidget *parent = nullptr);

    ~LNumberInputBox();

    /**
     * @brief The ChosseResult enum
     */
    enum ChosseResult
    {
        ID_OK = 0,                      // 确定;
        ID_CLOSE                        // 取消;
    };

    /**
     * @brief 设置标题
     * @param title 要设置的标题值
     */
    void setTitleText(QString title);

    /**
     * @brief 返回当前值
     * @return 要返回的值
     */
    int getValue();

    /**
     * @brief 设置输入的最大值
     * @param _maxValue  要设置的最大值
     */
    void setMaxValue(int _maxValue);

    /**
     * @brief showBox
     * @param titleText
     * @param isModelWindow
     * @return
     */
    int showBox( const QString &titleText, bool isModelWindow = false);

private:
    /**
     * @brief 初始化
     */
    void init();

    /**
     * @brief exec
     * @return
     */
    int exec();

    /**
     * @brief paintEvent
     * @param event
     */
    void paintEvent(QPaintEvent *event);

    /**
     * @brief closeEvent
     * @param event
     */
    void closeEvent(QCloseEvent *event);

private slots:

    /**
     * @brief 确定按钮单击事件
     */
    void onOkayClicked();

    /**
     * @brief 关闭按钮单击事件
     */
    void onCloseClicked();

    /**
     * @brief 退格按钮单击事件
     */
    void onBackspaceClicked();

    /**
     * @brief 数字按钮单击事件
     */
    void onDigitClicked();

private:
    int maxValue;
    QEventLoop* m_eventLoop;
    ChosseResult m_chooseResult;
    QLabel *titleLab;
    QPushButton *closeBtn;
    QPushButton *okayBtn;
    QPushButton *backspaceBtn;

    QLabel * display;
    enum { NumDigitButtons = 10 };
    QPushButton *digitButtons[NumDigitButtons];


};


#endif // LNUMBER_INPUT_BOX

