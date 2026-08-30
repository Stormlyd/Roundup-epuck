#include "LSlideSelector.h"

#define ITERM_SPACE 160

LSlideSelector::LSlideSelector(QWidget *parent) :QWidget(parent)
{
    setup();
    initGUI();
}

LSlideSelector::~LSlideSelector()
{
    delete indexNameLab;
    indexNameLab = nullptr;
}

void LSlideSelector::setup()
{
    currentIndex = -1;
    setMinimumSize(460, 150);
}

void LSlideSelector::initGUI()
{
    m_animationGroup = new QParallelAnimationGroup(this);
    indexNameLab = new QLabel(this);
    indexNameLab->setFixedSize(width(), 20);
    indexNameLab->move(0, height() - indexNameLab->height());
    indexNameLab->setAlignment(Qt::AlignCenter);
    indexNameLab->setStyleSheet(transparent_bg + text_white + font_size_md + font_weight_900);
}

void LSlideSelector::init()
{
    if(m_widgetList.count() > 3)
    {
        currentIndex = 0;
        indexNameLab->setText(names[currentIndex]);
        initFistSight();

        initAnimation();
    }
}

void LSlideSelector::wheelEvent(QWheelEvent *event)
{
    if (m_animationGroup->state() == QAnimationGroup::Running) {
        return;
    }

    if (event->delta() < 0) {
        scrollNext();
    } else {
        scrollPre();
    }
}

void LSlideSelector::mousePressEvent(QMouseEvent *event)
{
    posx = event->x();

}

void LSlideSelector::mouseReleaseEvent(QMouseEvent *event)
{
    if (m_animationGroup->state() == QAnimationGroup::Running) {
        return;
    }

    int diffx = posx - event->x();
    if(diffx >= 50)
    {
        scrollNext();
    }
    else if(diffx <= -50)
    {
        scrollPre();
    }

    indexNameLab->setText(names[currentIndex]);
}

void LSlideSelector::addItem(int value, QString name, QString iconPath)
{
    QLabel *lab = new QLabel(this);
    lab->setFixedSize(128,128);
    lab->setStyleSheet("border-image:url(" + iconPath + ");");
    lab->setVisible(false);
    m_widgetList << lab;

    names << name;
    values << value;
}

void LSlideSelector::addItem(QString iconPath)
{
    QLabel *lab = new QLabel(this);
    lab->setFixedSize(128,128);
    lab->setStyleSheet("border-image:url(" + iconPath + ");");
    lab->setVisible(false);
    m_widgetList2 << lab;
}

QPoint LSlideSelector::getPos(LSlideSelector::PosType type)
{
    auto wgt = m_widgetList.at(0);

    switch (type) {
    case Type_LL:
        return QPoint(width() / 2 - wgt->width() / 2 - ITERM_SPACE *  2, 0);
    case Type_L:
        return QPoint(width() / 2 - wgt->width() / 2 - ITERM_SPACE, 0);
    case Type_M:
        return QPoint(width() / 2 - wgt->width() / 2, 0);
    case Type_R:
        return QPoint(width() / 2 - wgt->width() / 2 + ITERM_SPACE, 0);
    case Type_RR:
        return QPoint(width() / 2 - wgt->width() / 2 + ITERM_SPACE * 2, 0);
    }
}

QWidget *LSlideSelector::getWidget(LSlideSelector::PosType type)
{
    QWidget *w;
    int index;

    switch (type) {
    case Type_LL:
        index = (currentIndex - 2) % m_widgetList.count();
        index = index >= 0 ? index : m_widgetList.count() + index;
        w = m_widgetList.at(index);
        m_widgetList2.at(index)->setVisible(false);
        break;
    case Type_L:
        index = (currentIndex - 1);
        index = index >= 0 ? index : m_widgetList.count() + index;
        w = m_widgetList2.at(index);
        m_widgetList.at(index)->setVisible(false);
        break;
    case Type_M:
        w = m_widgetList.at(currentIndex);
        m_widgetList2.at(currentIndex)->setVisible(false);
        break;
    case Type_R:
        index = (currentIndex + 1) % m_widgetList.count();
        w = m_widgetList2.at(index);
        m_widgetList.at(index)->setVisible(false);
        break;
    case Type_RR:
        index = (currentIndex + 2) % m_widgetList.count();
        w = m_widgetList.at(index);
        m_widgetList2.at(index)->setVisible(false);
        break;
    }

    return w;

}
QWidget *LSlideSelector::getFirstWidget(LSlideSelector::PosType type)
{

    QWidget *w;
    int index;

    switch (type) {
    case Type_LL:
        index = (currentIndex - 2) % m_widgetList.count();
        index = index >= 0 ? index : m_widgetList.count() + index;
        w = m_widgetList.at(index);
        break;
    case Type_L:
        index = (currentIndex - 1);
        index = index >= 0 ? index : m_widgetList.count() + index;
        w = m_widgetList.at(index);
        break;
    case Type_M:
        w = m_widgetList2.at(currentIndex);
        break;
    case Type_R:
        index = (currentIndex + 1) % m_widgetList.count();
        w = m_widgetList.at(index);
        break;
    case Type_RR:
        index = (currentIndex + 2) % m_widgetList.count();
        w = m_widgetList.at(index);
        break;
    }

    return w;

}

void LSlideSelector::initFistSight()
{
    for (int i = 0; i < 3; i++)
    {
        //-----modify by chenlu-------
        //auto w = getWidget(static_cast<PosType>(Type_L + i));
        auto w = getFirstWidget(static_cast<PosType>(Type_L + i));
        w->move(getPos(static_cast<PosType>(Type_L + i)));
        w->setVisible(true);
    }
}


void LSlideSelector::initAnimation()
{
    for (int i = 0; i < 4; i++)
    {
        QPropertyAnimation *animation = new QPropertyAnimation(m_animationGroup);
        animation->setDuration(150);
        animation->setPropertyName("pos");
        m_animationGroup->addAnimation(animation);
    }
}

void LSlideSelector::scrollPre()
{
    for (int i = 0; i < 4; i++)
    {
        QPropertyAnimation *a1 = static_cast<QPropertyAnimation *>(m_animationGroup->animationAt(i));
        a1->setStartValue(getPos(static_cast<PosType>(Type_LL + i)));
        a1->setEndValue(getPos(static_cast<PosType>(Type_L + i)));
        auto w1 = getWidget(static_cast<PosType>(Type_LL + i));

        w1->setVisible(true);
        a1->setTargetObject(w1);
    }

    m_animationGroup->start();
    currentIndex = (currentIndex - 1);
    currentIndex = currentIndex >= 0 ? currentIndex : m_widgetList.count() + currentIndex;
}

void LSlideSelector::scrollNext()
{
    for (int i = 0; i < 4; i++)
    {
        QPropertyAnimation *a1 = static_cast<QPropertyAnimation *>(m_animationGroup->animationAt(i));
        a1->setStartValue(getPos(static_cast<PosType>(Type_L + i)));
        a1->setEndValue(getPos(static_cast<PosType>(Type_LL + i)));
        auto w1 = getWidget(static_cast<PosType>(Type_L + i));
        w1->setVisible(true);
        a1->setTargetObject(w1);
    }

    m_animationGroup->start();
    currentIndex = (currentIndex + 1) % m_widgetList.count();
}

void LSlideSelector::scrollLeft()
{
     scrollPre();
     indexNameLab->setText(names[currentIndex]);
}
void LSlideSelector::scrollRight()
{
     scrollNext();
     indexNameLab->setText(names[currentIndex]);
}
