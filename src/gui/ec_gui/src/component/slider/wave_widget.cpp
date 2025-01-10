#include "wave_widget.h"
#include <iostream>

inline void initSlidersResource()
{
     Q_INIT_RESOURCE(ec_gui_start_resources);
}

namespace
{

QWidget * LoadUiFile(QWidget * parent)
{
    
    initSlidersResource();
    
    QUiLoader loader;

    QFile file(":/component/slider/wave_widget.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;
    
    
}
}

WaveWidget::WaveWidget(QDoubleSpinBox *valuebox,
                       uint8_t valuebox_property,
                       const QString&  min,
                       const QString&  max,
                       uint8_t decimal_value,
                       const QString& unit,
                       QWidget *parent) :
    QWidget(parent),
    _valuebox(valuebox),
    _valuebox_property(valuebox_property)
{
    /* Create GUI layout */
    auto * ui = ::LoadUiFile(this);

    auto * layout = new QVBoxLayout;
    layout->addWidget(ui);

    setLayout(layout);

    _slider_spinbox_fct=std::pow(10,decimal_value);
    
    auto min_label = findChild<QLabel *>("Min");
    min_label->setText(min+"   "+unit+"       ");

    auto max_label = findChild<QLabel *>("Max");
    max_label->setText(max+"   "+unit+"       ");

    _valueslider=findChild<QSlider *>("ValueSlider");
    _valueslider->setMaximum(_slider_spinbox_fct*max.toDouble());
    _valueslider->setMinimum(_slider_spinbox_fct*min.toDouble());
    _valueslider->setTickInterval(_slider_spinbox_fct);

    _actual_spinbox_value=0.0;
    _min_slider_value=min.toDouble();
    _max_slider_value=max.toDouble();

    if(_actual_spinbox_value>_max_slider_value){
        _valueslider->setValue(_max_slider_value);
    }
    else if(_actual_spinbox_value<_min_slider_value){
        _valueslider->setValue(_min_slider_value);
    }
    else{
        _valueslider->setValue(_actual_spinbox_value);
    }

    // connect slider to spinbox
    connect(_valueslider, &QSlider::valueChanged,
             std::bind(&WaveWidget::on_slider_changed, this));

    _valuebox->setMaximum(_max_slider_value);
    _valuebox->setMinimum(_min_slider_value);

    if(_actual_spinbox_value>_max_slider_value){
        _valuebox->setValue(_max_slider_value);
    }
    else if(_actual_spinbox_value<_min_slider_value){
        _valuebox->setValue(_min_slider_value);
    }
    else{
        _valuebox->setValue(_actual_spinbox_value);
    }

    _valuebox->setDecimals(decimal_value);
    _valuebox->setSingleStep(1/((double)_slider_spinbox_fct));

     // connect spinbox to slider

    _valuebox->setKeyboardTracking(false);

    connect(_valuebox, &QDoubleSpinBox::editingFinished,
           std::bind(&WaveWidget::on_spinbox_changed, this)
           );


    _slider_filtered=std::make_shared<SecondOrderFilter<double>>(12.0,1.0,1.0,_actual_spinbox_value);

    // find wave tab.
    _tab_wave = findChild<QTabWidget *>("tabWave");
    _tab_wave_type = findChild<QTabWidget *>("tabWaveType");

    _wave_a=findChild<QDoubleSpinBox *>("Wave_A");
    _wave_a->setMaximum(_max_slider_value);
    _wave_a->setMinimum(_min_slider_value);
    _wave_a->setDecimals(decimal_value);
    _wave_a->setSingleStep(1/((double)_slider_spinbox_fct));
    
    _wave_f=findChild<QDoubleSpinBox *>("Wave_F");
    _wave_t=findChild<QDoubleSpinBox *>("Wave_T");
}

void WaveWidget::on_slider_changed()
{
    double value = _valueslider->value();
    // update spinbox
    _valuebox->setValue(value/((double)_slider_spinbox_fct));

}

void WaveWidget::on_spinbox_changed()
{
    double value = _valuebox->value();

    // update slider
    _valueslider->blockSignals(true);
    _valueslider->setValue(int(_slider_spinbox_fct*value));
    _valueslider->blockSignals(false);   
}

void WaveWidget::align_spinbox(double value)
{
    _actual_spinbox_value = value;
    _valuebox->setValue(_actual_spinbox_value);

    // update slider
    _valueslider->blockSignals(true);
    _valueslider->setValue(int(_slider_spinbox_fct*_actual_spinbox_value));
    _valueslider->blockSignals(false);
}

void WaveWidget::align_spinbox()
{
    _valuebox->setValue(_actual_spinbox_value);

    // update slider
    _valueslider->blockSignals(true);
    _valueslider->setValue(int(_slider_spinbox_fct*_actual_spinbox_value));
    _valueslider->blockSignals(false);
}


void WaveWidget::disable_slider()
{
    enable_tab_wave();
    _valueslider->setEnabled(false);
    if(_valuebox_property!=SliderProperty::ALWAYS_AVAILABLE){
        _valuebox->setEnabled(false);
    }
}
void WaveWidget::enable_slider()
{
    tab_wave_selected();
    _valueslider->setEnabled(true);
    if(_valuebox_property!=SliderProperty::NOT_AVAILABLE){
        _valuebox->setEnabled(true);
    }
}

double WaveWidget::get_spinbox_value()
{
    return _valuebox->value();
}

void WaveWidget::set_spinbox_value(double actual_spinbox_value)
{
    _actual_spinbox_value=actual_spinbox_value;
}


void WaveWidget::set_filter(double st)
{
    _slider_filtered->reset(_valuebox->value());
    _slider_filtered->setTimeStep(st);

    _chirp_counter=0;
    _chirp_dur_s=10; // 10s
    _chirp_inv=false;
    _chirp_w1=2*M_PI*_wave_f->value();
    _chirp_w2=2*M_PI*_wave_f->value()*10;
}

double WaveWidget::compute_wave(double t)
{
    double fx=0;
    if(_tab_wave->currentIndex()==0){
        fx = _slider_filtered->process(_valuebox->value());
    }
    else{
        if(t==0){
            fx=_valuebox->value();
        }
        else{
            if(_tab_wave_type->currentIndex()==0){
                fx = _valuebox->value() +_wave_a->value() * std::sin (2*M_PI*_wave_f->value()*t + _wave_t->value());
            }
            else if(_tab_wave_type->currentIndex()==1){
                fx=-1*_wave_a->value();
                if(std::signbit(std::sin (2*M_PI*_wave_f->value()*t + _wave_t->value()))){
                    fx=1*_wave_a->value();
                }
                fx=_valuebox->value()+ fx;
            }
            else if(_tab_wave_type->currentIndex()==2){
                fx = _valuebox->value() + (2*_wave_a->value()/M_PI) * std::asin(std::sin (2*M_PI*_wave_f->value()*t + _wave_t->value()));
            }
            else if(_tab_wave_type->currentIndex()==4){
                if(_chirp_counter>=200*_chirp_dur_s){ // 200Hz since p = 0.005 ms
                    _chirp_counter=0;
                    if(!_chirp_inv){
                        _chirp_inv=true;
                    }
                    else{
                        _chirp_inv=false;
                    }
                }
                if(_chirp_counter==0){
                    _chirp_start_t=t;
                }
                _chirp_t = t - _chirp_start_t;

                if(!_chirp_inv){
                    fx = _valuebox->value()+_wave_a->value()* std::sin(_chirp_w1*_chirp_t+(_chirp_w2-_chirp_w1)*_chirp_t*_chirp_t/(2*_chirp_dur_s));
                }else{
                    fx = _valuebox->value()+_wave_a->value()* std::sin(_chirp_w2*_chirp_t+(_chirp_w1-_chirp_w2)*_chirp_t*_chirp_t/(2*_chirp_dur_s));
                }

                _chirp_counter++;
            }
        }
    }

    if(fx >= _max_slider_value){
        fx=_max_slider_value;
    }
    else if (fx <= _min_slider_value){
        fx=_min_slider_value;
    }

    return fx;
}

void WaveWidget::enable_tab_wave()
{
    _tab_wave->setEnabled(true);
    _tab_wave->setTabEnabled(0,true);
    _tab_wave->setTabEnabled(1,true);

    _tab_wave_type->setTabEnabled(0,true);
    _tab_wave_type->setTabEnabled(1,true);
}

void WaveWidget::disable_tab_wave()
{
    _tab_wave->setEnabled(false);
    _tab_wave->setTabEnabled(0,false);
    _tab_wave->setTabEnabled(1,false);

    _tab_wave_type->setTabEnabled(0,false);
    _tab_wave_type->setTabEnabled(1,false);
}

void WaveWidget::tab_wave_selected()
{
    int curr_tab= _tab_wave->currentIndex();
    if(curr_tab==0){
        _tab_wave->setTabEnabled(0,true);
        _tab_wave->setTabEnabled(1,false);
    }
    else{
        _tab_wave->setTabEnabled(0,false);
        _tab_wave->setTabEnabled(1,true);

        int wave_type= _tab_wave_type->currentIndex();
        if(wave_type==0){
            _tab_wave_type->setTabEnabled(0,true);
            _tab_wave_type->setTabEnabled(1,false);
        }
        else{
            _tab_wave_type->setTabEnabled(0,false);
            _tab_wave_type->setTabEnabled(1,true);
        }
    }
}

WaveWidget::~WaveWidget()
{
    
}
