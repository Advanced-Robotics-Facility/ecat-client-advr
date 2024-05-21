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

    explicit WaveWidget (double init_value,
                         QDoubleSpinBox *valuebox,
                         const QString& min,
                         const QString& max,
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
    QTabWidget * _tab_wave;
    QDoubleSpinBox *_sine_a,*_sine_f,*_sine_t;
    QDoubleSpinBox *_square_a,*_square_f,*_square_t;
    int _slider_spinbox_fct;

    double _actual_slider_value;

    SecondOrderFilter<double>::Ptr _slider_filtered;
};

#endif
