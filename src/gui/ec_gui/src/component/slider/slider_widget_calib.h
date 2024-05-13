#ifndef SLIDERWIDGETCALIB_H
#define SLIDERWIDGETCALIB_H


#include <QtUiTools/QtUiTools>
#include <QWidget>
#include <QLabel>
#include <functional>

#include <memory>
#include "ec_gui_utils.h"


class SliderWidgetCalib : public QWidget
{
Q_OBJECT
public:

    explicit SliderWidgetCalib (const QString& joint_name,
                                std::vector<double> init_value,
                                std::vector<std::string> sliders_type,
                                QWidget * parent = 0);

    double get_slider_value(int type);
    int get_slider_numb();
    void disable_slider_calib(int index);
    void enable_slider_calib(int index);
    void hide_slider_calib(int index);
    void set_filter(double st);
    void filtering(std::vector<float>& );
    SecondOrderFilter<double>::Ptr get_slider_filter(int index);

    
    ~SliderWidgetCalib();

private:

    void on_spinbox_changed(int i);

    bool _callback_enabled;
    QLabel *_jname;

    std::vector<QLabel *>           _slidertype;
    std::vector<QDoubleSpinBox *>   _valuebox;
    std::vector<SecondOrderFilter<double>::Ptr> _valuebox_filtered;
    int _slider_numb;

    
};

#endif // SLIDERWIDGETCALIB_H
