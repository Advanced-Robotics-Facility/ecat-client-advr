#ifndef JOINT_WIDGET_H
#define JOINT_WIDGET_H

#include <QWidget>
#include <functional>
#include <QProgressBar>
#include <QLabel>
#include <QTimer>

class JointWidget;

/**
 * @brief The Blinker class implements the red-green
 * blinking of the fault square box
 */
class Blinker : public QObject
{
    Q_OBJECT

public:

    Blinker(JointWidget * parent);

    void stop();

    void blink(int nblinks);

    bool blinking() const;


public slots :

    void on_timeout();

private:


    QTimer _timer;

    int _blinks;
    int _state;
    JointWidget * _parent;
};

/**
 * @brief The JointWidget class implements the single bar for a
 * single joint
 */
class JointWidget : public QWidget
{

    Q_OBJECT

public:

    friend class Blinker;

    explicit JointWidget(const QString& jname, QWidget * parent = nullptr);

    void setRange(double min, double max);
    void setValue(double x);
    void setValue(double xbar, double xtext);
    void setStatus(QString status);
    void setSafe(bool force = false);
    void setDanger(bool force = false);
    void updateStatus();
    void setActive();
    void setInactive();
    QString getJointName() const;

signals:

    void doubleLeftClicked();
    void doubleRightClicked();

private:

    void setColor(Qt::GlobalColor color);

    bool eventFilter(QObject * obj, QEvent * ev) override;

    std::function<void(void)> _on_double_click;

    QProgressBar * _bar;
    QLabel * _jname;
    QLabel * _status;

    Blinker _blinker;
    int _state;

    std::chrono::high_resolution_clock::time_point _last_fault_time;



};

#endif // JOINT_WIDGET_H
