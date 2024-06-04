#ifndef WAVEWIDGET_H
#define WAVEWIDGET_H


#include <QtUiTools/QtUiTools>
#include <QWidget>
#include <QLabel>
#include <functional>
#include "ec_gui_utils.h"

class WaveWidget : public QWidget
{
Q_OBJECT
public:

    explicit WaveWidget (QDoubleSpinBox *valuebox,
                         const QString& min,
                         const QString& max,
                         uint8_t decimal_value,
                         const QString& unit,
                         QWidget * parent = 0);

    double get_spinbox_value();
    double get_actual_slider_value();
    void   set_actual_slider_value(double actual_slider_value);
    void align_spinbox(double value);
    void align_spinbox();
    void disable_slider();
    void enable_slider();

    void set_filter(double st);
    double compute_wave(double t);

    ~WaveWidget();

private:

    void on_slider_changed();
    void on_spinbox_changed();
    void on_spinbox_clicked(int i);
    

    void enable_tab_wave();
    void disable_tab_wave();
    void tab_wave_selected();

    QSlider *          _valueslider;
    QDoubleSpinBox *   _valuebox;
    QTabWidget * _tab_wave,*_tab_wave_type;
    QDoubleSpinBox *_wave_a,*_wave_f,*_wave_t;
    int _slider_spinbox_fct;

    double _actual_slider_value;
    double _min_slider_value,_max_slider_value;

    SecondOrderFilter<double>::Ptr _slider_filtered;
};

#endif
