#ifndef SLIDERWINDOW_H
#define SLIDERWINDOW_H


#include <QtUiTools/QtUiTools>
#include <QWidget>



class SliderWindow : public QWidget
{
Q_OBJECT
public:

    explicit SliderWindow (const QStringList&  control_mode,
                           const std::vector<int> control_mode_hex,
                           QWidget * parent = 0);
    
    QVBoxLayout* get_layout();
    int get_control_mode();
    void enable_control_mode();
    void disable_control_mode();

    ~SliderWindow();

public slots:
     void readControlMode();

private:
    QComboBox *_control_mode;
    std::vector<int> _control_mode_hex;
    int _actual_control_mode;
    QVBoxLayout* _layout;
};

#endif // SLIDERWINDOW_H
