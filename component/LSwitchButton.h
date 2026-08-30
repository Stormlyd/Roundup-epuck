#ifndef LSWITCH_BUTTON
#define LSWITCH_BUTTON

#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include <QDebug>

class LSwitchButton : public QWidget
{
    Q_OBJECT
public:
    explicit LSwitchButton(QWidget *parent = nullptr);

    /**
     * @brief 返回开关状态
     * @return  返回 打开时true;关闭时false
     */
    bool isToggled() const;

    /**
     * @brief 设置开关状态
     * @param checked   要设置的状态
     */
    void setToggle(bool checked);

    /**
     * @brief 设置背景颜色
     * @param color 要设置的颜色
     */
    void setBackgroundColor(QColor color);

    /**
     * @brief 设置选中颜色
     * @param color 要设置的颜色
     */
    void setCheckedColor(QColor color);

    /**
     * @brief 设置禁用时的颜色
     * @param color 要设置的颜色
     */
    void setDisbaledColor(QColor color);

protected:
    /**
     * @brief 重绘样式
     * @param event
     */
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;

    /**
     * @brief 鼠标按下事件
     * @param event
     */
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

    /**
     * @brief 鼠标释放事件切换开关状态发射toggled()信号
     * @param event
     */
    void mouseReleaseEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

    /**
     * @brief 大小改变事件
     * @param event
     */
    void resizeEvent(QResizeEvent *event) Q_DECL_OVERRIDE;

signals:
    /**
     * @brief 状态改变时，发射信号
     * @param checked
     */
    void toggled(bool checked);

private:
    bool _checked;           // 选中状态
    QColor _background;      // 滑轨颜色
    QColor _thumbColor;      // 滑块颜色
    QColor _checkedColor;    // 选中颜色
    QColor _disabledColor;   // 禁用颜色
};

#endif // LSWITCH_BUTTON

