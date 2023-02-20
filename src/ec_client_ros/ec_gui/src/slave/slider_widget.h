#ifndef SLIDERWIDGET_H
#define SLIDERWIDGET_H


#include <QtUiTools/QtUiTools>
#include <QWidget>
#include <QLabel>
#include <functional>

class SliderWidget : public QWidget
{
Q_OBJECT
public:

    explicit SliderWidget (const QString& joint_name,
                           std::vector<double> init_value,
                           std::vector<std::string> sliders_type,
                           QWidget * parent = 0);

    void setRange(std::vector<double> min,
                  std::vector<double> max);

    void setInitialValue(std::vector<double> x_0);

    double get_slider_value(int type);
    double get_radiobtn_value(int type);
    
    ~SliderWidget();

private:

    void on_slider_changed(int i);
    void on_spinbox_changed(int i);

    bool _callback_enabled;
    QLabel *_jname;

    std::vector<QLabel *>           _slidertype;
    //std::vector<QSlider *>          _valueslide;
    std::vector<QDoubleSpinBox *>   _valuebox;
    std::vector<QRadioButton *>   _radiobox;
    
};

#endif // SLIDERWIDGET_H
