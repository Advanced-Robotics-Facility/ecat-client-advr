#ifndef SLIDERWIDGET_H
#define SLIDERWIDGET_H


#include <QtUiTools/QtUiTools>
#include <QWidget>
#include <QLabel>
#include <functional>
#include "wave_widget.h"


class SliderWidget : public QWidget
{
Q_OBJECT
public:

    typedef struct slider_info_t{
        std::vector<std::string> slider_name;
        std::vector<std::string> slider_unit;
        std::vector<uint8_t> slider_decimal;
        std::vector<std::string> slider_min;
        std::vector<std::string> slider_max;
        std::vector<uint8_t> slider_property;
    }slider_info_s;

    explicit SliderWidget (const QString& name,
                           const slider_info_t slider_info_s,
                           QWidget * parent = 0);



    double get_spinbox_value(int i);
    void   set_spinbox_value(int i,double actual_spinbox_value);
    void align_spinbox(int i,double value);
    void align_all_spinbox(double value);
    void align_spinbox(int i);
    void align_all_spinbox();
    void disable_slider();
    void enable_slider();
    void hide_slider_enabled();
    void enable_slider_enabled();
    void disable_slider_enabled();
    QCheckBox* get_slider_enabled();

    bool is_slider_enabled();
    bool is_slider_checked();
    void check_slider_enabled();
    void uncheck_slider_enabled();
    std::string get_slider_name();

    void set_wave_info(double st, bool stopping_wave);
    double compute_wave(int i,double t);

    ~SliderWidget();

public slots:
    void align_wave_value();

private:

    void on_spinbox_clicked(int i);
    
    bool _callback_enabled;

    QTabWidget *_tab_name_wid;
    QCheckBox* _slider_enabled;
    std::string _slider_name;
    std::vector<WaveWidget *> _wave_v;
    QTimer *_update_value_timer;
    
};

#endif // SLIDERWIDGET_H
