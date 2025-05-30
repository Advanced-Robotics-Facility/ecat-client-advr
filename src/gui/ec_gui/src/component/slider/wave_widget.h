#ifndef WAVEWIDGET_H
#define WAVEWIDGET_H


#include <QtUiTools/QtUiTools>
#include <QWidget>
#include <QLabel>
#include <functional>
#include "ec_gui_utils.h"


enum SliderProperty : uint8_t
{
    NOT_AVAILABLE       = 0,        // slider not available for changing.
    AVAILABLE           = 1,        // slider available for changing.
    ALWAYS_AVAILABLE    = 2,        // slider always available.
    PARTIAL_AVAILABLE   = 3,        // slider partial available.
};


class WaveWidget : public QWidget
{
Q_OBJECT
public:

    explicit WaveWidget (QDoubleSpinBox *valuebox,
                         uint8_t valuebox_property,
                         const QString& min,
                         const QString& max,
                         uint8_t decimal_value,
                         const QString& unit,
                         QWidget * parent = 0);

    double get_spinbox_value();
    void set_spinbox_value(double actual_spinbox_value);
    void align_spinbox(double value);
    void align_spinbox();
    void align_wave_spinbox();
    void change_spinbox(double value);
    void disable_slider();
    void enable_slider();

    bool set_wave_info(double st,bool stopping_wave);
    double compute_wave(double t);

    ~WaveWidget();

private:

    void on_slider_changed();
    void on_spinbox_changed();
    void wave_param_changed();
    void set_wave_param();

    void enable_tab_wave();
    void disable_tab_wave();
    void tab_wave_selected();

    QSlider *          _valueslider;
    QDoubleSpinBox *   _valuebox;
    QTabWidget * _tab_wave,*_tab_wave_type;
    QDoubleSpinBox *_wave_a,*_wave_f,*_wave_t;
    int _slider_spinbox_fct;

    double _actual_spinbox_value;
    double _min_slider_value,_max_slider_value;

    SecondOrderFilter<double>::Ptr _slider_filtered;
    uint8_t _valuebox_property;
    uint64_t _trj_counter=0;
    bool _init_trj;
    double _trj_start_t,_trj_t;
    double _chirp_w_start,_chirp_w_diff;
    bool _chirp_inv=false;
    uint8_t _chirp_dur_s;
    double _fx,_x0;
    double _amp,_freq,_theta;
    int _wave_type;
    bool _stopping_wave=false;
    bool _is_wave=false;
};

#endif
