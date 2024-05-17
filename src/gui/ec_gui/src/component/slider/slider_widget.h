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
    void remove_calibration();
    SecondOrderFilter<double>::Ptr get_filter();
    void set_filter(double st);
    double compute_wave(double t);

    ~SliderWidget();

private:

    void on_slider_changed();
    void on_spinbox_changed();

    void enable_tab_wave();
    void disable_tab_wave();
    void tab_wave_selected();

    bool _callback_enabled;

    QVBoxLayout* _calibration_layout;
    QHBoxLayout* _pid_layout;

    QSlider *          _valueslider;
    QDoubleSpinBox *   _valuebox;
    QTabWidget * _tab_wave;
    QDoubleSpinBox *_sine_a,*_sine_f,*_sine_t;
    QDoubleSpinBox *_square_a,*_square_f,*_square_t;
    QCheckBox* _slider_enabled;
    int _slider_spinbox_fct;

    double _actual_slider_value;

    SecondOrderFilter<double>::Ptr _slider_filtered;
    SliderWidgetCalib *_wid_calib;
    std::string _slider_name;

    std::vector<QLabel *>           _slidertype_v;
    std::vector<QDoubleSpinBox *>   _valuebox_v;
    std::vector<SecondOrderFilter<double>::Ptr> _valuebox_filtered;
    int _slider_numb;
    
};

#endif // SLIDERWIDGET_H
