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

    explicit SliderWidget (const QString& joint_name,
                           double init_value,
                           const QString& min,
                           const QString& max,
                           const QString& unit,
                           float ctrl_type,
                           std::vector<std::string> pid_string,
                           std::vector<double> gains,
                           QWidget * parent = 0);

    void setRange(std::vector<double> min,
                  std::vector<double> max);

    void setInitialValue(std::vector<double> x_0);

    double get_spinbox_value();

    double get_actual_slider_value();
    void   set_actual_slider_value(double actual_slider_value);
    void align_spinbox(double value);
    void align_spinbox();
    void disable_slider();
    void enable_slider();
    void enable_joint_enabled();
    void disable_joint_enabled();
    void hide_joint_enabled();
    void unhide_joint_enabled();
    bool is_joint_enabled();
    void check_joint_enabled();
    void uncheck_joint_enabled();
    bool is_joint_braked();
    void check_joint_braked();
    void uncheck_joint_braked();
    void hide_led_on_off_btn();
    void unhide_led_on_off_btn();
    std::string get_joint_name();
    QPushButton* get_led_on_off_btn();
    SliderWidgetCalib *get_wid_calibration();

    SecondOrderFilter<double>::Ptr get_filer();

    ~SliderWidget();

private:

    void on_slider_changed();
    void on_spinbox_changed();

    bool _callback_enabled;
    QLabel *_jname,*_j_braked;
    QLabel *_min;
    QLabel *_max;
    QLabel *_unit;

    QHBoxLayout* _pid_layout;


    QSlider *          _valueslider;
    QDoubleSpinBox *   _valuebox;
    QCheckBox* _joint_enabled;
    QCheckBox* _joint_is_braked;
    QPushButton* _led_on_off;
    int _slider_spinbox_fct;

    double _actual_slider_value;

    SecondOrderFilter<double>::Ptr _slider_filtered;
    SliderWidgetCalib *_wid_calib;
    std::string _joint_name;
    
};

#endif // SLIDERWIDGET_H
