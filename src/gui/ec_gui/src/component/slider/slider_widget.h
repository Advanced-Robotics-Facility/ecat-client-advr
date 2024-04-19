#ifndef SLIDERWIDGET_H
#define SLIDERWIDGET_H


#include <QtUiTools/QtUiTools>
#include <QWidget>
#include <QLabel>
#include <functional>
#include "slider_widget_calib.h"

class SliderWidget : public QWidget
{
Q_OBJECT
public:

    explicit SliderWidget (const QString& name,
                           double init_value,
                           const QString& min,
                           const QString& max,
                           const QString& unit,
                           std::vector<std::string> pid_string,
                           std::vector<double> gains,
                           QWidget * parent = 0);



    double get_spinbox_value();
    double get_actual_slider_value();
    void   set_actual_slider_value(double actual_slider_value);
    void align_spinbox(double value);
    void align_spinbox();
    void disable_slider();
    void enable_slider();
    void hide_slider_enabled();
    void enable_slider_enabled();
    void disable_slider_enabled();

    bool is_slider_enabled();
    void check_slider_enabled();
    void uncheck_slider_enabled();
    std::string get_slider_name();
    SliderWidgetCalib *get_wid_calibration();

    SecondOrderFilter<double>::Ptr get_filer();

    ~SliderWidget();

private:

    void on_slider_changed();
    void on_spinbox_changed();

    bool _callback_enabled;

    QHBoxLayout* _pid_layout;


    QSlider *          _valueslider;
    QDoubleSpinBox *   _valuebox;
    QCheckBox* _slider_enabled;
    int _slider_spinbox_fct;

    double _actual_slider_value;

    SecondOrderFilter<double>::Ptr _slider_filtered;
    SliderWidgetCalib *_wid_calib;
    std::string _slider_name;
    
};

#endif // SLIDERWIDGET_H
