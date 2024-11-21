#ifndef SLIDERWINDOW_H
#define SLIDERWINDOW_H


#include <QtUiTools/QtUiTools>
#include <QWidget>



class SliderWindow : public QWidget
{
Q_OBJECT
public:

    explicit SliderWindow (const QStringList& control_mode,
                           QWidget * parent = 0);
    
    QVBoxLayout* get_layout();

    ~SliderWindow();

private:
    QComboBox *_control_mode;
    QVBoxLayout* _layout;
};

#endif // SLIDERWINDOW_H
