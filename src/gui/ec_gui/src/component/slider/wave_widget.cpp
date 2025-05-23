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
    connect(_valueslider, &QSlider::valueChanged,std::bind(&WaveWidget::on_slider_changed, this));

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

    connect(_valuebox, &QDoubleSpinBox::editingFinished,std::bind(&WaveWidget::on_spinbox_changed, this));

    _slider_filtered=std::make_shared<SecondOrderFilter<double>>(12.0,1.0,1.0,_actual_spinbox_value);

    // find wave tab.
    _tab_wave = findChild<QTabWidget *>("tabWave");
    _tab_wave_type = findChild<QTabWidget *>("tabWaveType");
    connect(_tab_wave_type, &QTabWidget::currentChanged,std::bind(&WaveWidget::wave_param_changed, this));

    _wave_a=findChild<QDoubleSpinBox *>("Wave_A");
    _wave_a->setMaximum(_max_slider_value);
    _wave_a->setMinimum(_min_slider_value);
    _wave_a->setDecimals(decimal_value);
    _wave_a->setSingleStep(1/((double)_slider_spinbox_fct));
    _wave_a->setKeyboardTracking(false);
    connect(_wave_a, &QDoubleSpinBox::editingFinished,std::bind(&WaveWidget::wave_param_changed, this));
    _wave_f=findChild<QDoubleSpinBox *>("Wave_F");
    _wave_f->setKeyboardTracking(false);
    connect(_wave_f, &QDoubleSpinBox::editingFinished,std::bind(&WaveWidget::wave_param_changed, this));
    _wave_t=findChild<QDoubleSpinBox *>("Wave_T");
    _wave_t->setKeyboardTracking(false);
    connect(_wave_t, &QDoubleSpinBox::editingFinished,std::bind(&WaveWidget::wave_param_changed, this));

    set_wave_param();
}

void WaveWidget::wave_param_changed()
{
    if(_amp==_wave_a->value() &&
       _freq==_wave_f->value() &&
       _theta==_wave_t->value() &&
       _wave_type==_tab_wave_type->currentIndex()){
        return;
    }

    set_wave_param();
}

void WaveWidget::set_wave_param()
{
    _amp=   _wave_a->value();
    _freq=  _wave_f->value();
    _theta= _wave_t->value();
    _wave_type=_tab_wave_type->currentIndex();

    _init_trj=true; 
    _trj_counter=0;

    _chirp_dur_s=10; // 10s
    _chirp_inv=false;
    _chirp_w_start= 2*M_PI*_freq;
    _chirp_w_diff=  2*M_PI*(10*_freq-_freq);

    _x0=_actual_spinbox_value;
    _fx=0;
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
    change_spinbox(_actual_spinbox_value);
}

void WaveWidget::align_spinbox()
{
    change_spinbox(_actual_spinbox_value);
}

void WaveWidget::align_wave_spinbox()
{
    if(_tab_wave->currentIndex()!=0 && !_stopping_wave){
        change_spinbox(_fx);
    }
}

void WaveWidget::change_spinbox(double value)
{
    _valuebox->setValue(value);
    on_spinbox_changed();
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
    if(_valuebox_property!=SliderProperty::NOT_AVAILABLE &&  _tab_wave->currentIndex()==0){
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


bool WaveWidget::set_wave_info(double st, bool stopping_wave)
{
    _stopping_wave=stopping_wave;
    _slider_filtered->reset(_valuebox->value());
    _slider_filtered->setTimeStep(st);

    set_wave_param();

    return _is_wave;
}

void WaveWidget::enable_tab_wave()
{
    _tab_wave->setEnabled(true);
    for(int i=0; i<_tab_wave->count();i++){
        _tab_wave->setTabEnabled(i,true);
    }
     for(int i=0; i<_tab_wave_type->count();i++){
        _tab_wave_type->setTabEnabled(i,true);
    }
}

void WaveWidget::disable_tab_wave()
{
    _tab_wave->setEnabled(false);
    for(int i=0; i<_tab_wave->count();i++){
        _tab_wave->setTabEnabled(i,false);
    }

    for(int i=0; i<_tab_wave_type->count();i++){
        _tab_wave_type->setTabEnabled(i,false);
    }
}

void WaveWidget::tab_wave_selected()
{
    int curr_tab= _tab_wave->currentIndex();
    if(curr_tab==0){
        _tab_wave->setTabEnabled(1,false);
        _is_wave=false;
    }
    else{
        _tab_wave->setTabEnabled(0,false);
        _is_wave=true;
    }
}

// done into the send thread.
double WaveWidget::compute_wave(double t)
{
    if(!_is_wave || _stopping_wave){
        _fx = _slider_filtered->process(_valuebox->value());
    }
    else{
        if(t!=0){
            if(_trj_counter>=200*_chirp_dur_s){ // 200Hz since p = 0.005 ms
                _trj_counter=0;
                _init_trj=true;
                if(!_chirp_inv){
                    _chirp_inv=true;
                    _chirp_w_start= 2*M_PI*10*_freq;
                    _chirp_w_diff=  2*M_PI*(_freq-10*_freq);
                }
                else{
                    _chirp_inv=false;
                    _chirp_w_start= 2*M_PI*_freq;
                    _chirp_w_diff=  2*M_PI*(10*_freq-_freq);
                }
            }

            if(_init_trj){
                _init_trj=false;
                _trj_start_t=t;
            }
            _trj_t = t - _trj_start_t;

            if(_wave_type!=4){
                double sine_fx = std::sin (2*M_PI*_freq*_trj_t + _theta);
                if(_wave_type==0){
                    _fx = _amp * sine_fx ;
                }
                else if(_wave_type==1){
                    _fx=_amp;
                    if(std::signbit(sine_fx)){
                        _fx=-_amp;
                    }
                }
                else if(_wave_type==2){
                    _fx = 2*_amp/M_PI * std::asin(sine_fx);
                }
                else if(_wave_type==3){
                    _fx = 4*_amp/M_PI * std::asin(sine_fx);
                    if(std::abs(_fx)>=std::abs(_amp)){
                        if(std::signbit(_fx)){
                            _fx=-std::abs(_amp);
                        }
                        else{
                           _fx=std::abs(_amp); 
                        }
                    }
                }
            }
            else{
                _fx = _amp* std::sin(_chirp_w_start*_trj_t+_chirp_w_diff*_trj_t*_trj_t/(2*_chirp_dur_s));
                _trj_counter++;
            }
        }
        _fx=_fx + _x0;
    }

    if(_fx >= _max_slider_value){
        _fx=_max_slider_value;
    }
    else if (_fx <= _min_slider_value){
        _fx=_min_slider_value;
    }

    return _fx;
}

WaveWidget::~WaveWidget()
{
    
}
