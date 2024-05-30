#include "wave_widget.h"

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
                       const QString&  min,
                       const QString&  max,
                       uint8_t decimal_value,
                       const QString& unit,
                       QWidget *parent) :
    _valuebox(valuebox),
    QWidget(parent)
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
    _valueslider->setTickInterval(1);

    _actual_slider_value=0.0;

    if(_actual_slider_value>max.toDouble()){
        _valueslider->setValue(max.toDouble());
    }
    else if(_actual_slider_value<min.toDouble()){
        _valueslider->setValue(min.toDouble());
    }
    else{
        _valueslider->setValue(_actual_slider_value);
    }

    // connect slider to spinbox
    connect(_valueslider, &QSlider::valueChanged,
             std::bind(&WaveWidget::on_slider_changed, this));

    _valuebox->setMaximum(max.toDouble());
    _valuebox->setMinimum(min.toDouble());

    if(_actual_slider_value>max.toDouble()){
        _valuebox->setValue(max.toDouble());
    }
    else if(_actual_slider_value<min.toDouble()){
        _valuebox->setValue(min.toDouble());
    }
    else{
        _valuebox->setValue(_actual_slider_value);
    }

    _valuebox->setDecimals(decimal_value);
    _valuebox->setSingleStep(1/((double)_slider_spinbox_fct));

     // connect spinbox to slider

    _valuebox->setKeyboardTracking(false);

    connect(_valuebox, &QDoubleSpinBox::editingFinished,
           std::bind(&WaveWidget::on_spinbox_changed, this)
           );


    _slider_filtered=std::make_shared<SecondOrderFilter<double>>(12.0,1.0,1.0,_actual_slider_value);

    // find wave tab.
    _tab_wave = findChild<QTabWidget *>("tabWave");

    _sine_a=findChild<QDoubleSpinBox *>("Sine_A");
    _sine_a->setMaximum(max.toDouble());
    _sine_f=findChild<QDoubleSpinBox *>("Sine_F");
    _sine_t=findChild<QDoubleSpinBox *>("Sine_T");


    _square_a=findChild<QDoubleSpinBox *>("Square_A");
    _square_a->setMaximum(max.toDouble());
    _square_f=findChild<QDoubleSpinBox *>("Square_F");
    _square_t=findChild<QDoubleSpinBox *>("Square_T");

    //disable_slider();

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
    _valuebox->setValue(value);

    // update slider
    _valueslider->blockSignals(true);
    _valueslider->setValue(int(_slider_spinbox_fct*value));
    _valueslider->blockSignals(false);   
}

void WaveWidget::align_spinbox(double value)
{
    _valuebox->setValue(value);

    // update slider
    _valueslider->blockSignals(true);
    _valueslider->setValue(int(_slider_spinbox_fct*value));
    _valueslider->blockSignals(false);
}

void WaveWidget::align_spinbox()
{
    _valuebox->setValue(_actual_slider_value);

    // update slider
    _valueslider->blockSignals(true);
    _valueslider->setValue(int(_slider_spinbox_fct*_actual_slider_value));
    _valueslider->blockSignals(false);
}


void WaveWidget::disable_slider()
{
    enable_tab_wave();
    _valueslider->setEnabled(false);
    _valuebox->setEnabled(false);
}
void WaveWidget::enable_slider()
{
    tab_wave_selected();
    _valueslider->setEnabled(true);
    _valuebox->setEnabled(true);
}

double WaveWidget::get_spinbox_value()
{
    return _valuebox->value();
}

double WaveWidget::get_actual_slider_value()
{
    return _actual_slider_value;
}

void WaveWidget::set_actual_slider_value(double actual_slider_value)
{
    _actual_slider_value=actual_slider_value;
}


void WaveWidget::set_filter(double st)
{
    _slider_filtered->reset(_valuebox->value());
    _slider_filtered->setTimeStep(st);
}

double WaveWidget::compute_wave(double t)
{
    double fx=0;
    switch (_tab_wave->currentIndex()){
    case 0:
        fx = _slider_filtered->process(_valuebox->value());
        break;
    case 1:
        if(t==0){
            fx=_valuebox->value();
        }
        else{
            fx = _valuebox->value() +_sine_a->value() * std::sin (2*M_PI*_sine_f->value()*t + _sine_t->value());
        }
        break;
    case 2:
        if(t==0){
            fx=_valuebox->value();
        }
        else{
            fx=-1*_square_a->value();
            if(std::signbit(std::sin (2*M_PI*_square_f->value()*t + _square_t->value()))){
                fx=1*_square_a->value();
            }
            fx=_valuebox->value()+ fx;
        }
        break;
    
    default:
        break;
    }
    return fx;
}

void WaveWidget::enable_tab_wave()
{
    _tab_wave->setEnabled(true);
    _tab_wave->setTabEnabled(0,true);
    _tab_wave->setTabEnabled(1,true);
    _tab_wave->setTabEnabled(2,true);
}

void WaveWidget::disable_tab_wave()
{
    _tab_wave->setEnabled(false);
    _tab_wave->setTabEnabled(0,false);
    _tab_wave->setTabEnabled(1,false);
    _tab_wave->setTabEnabled(2,false);
}

void WaveWidget::tab_wave_selected()
{
    int curr_tab= _tab_wave->currentIndex();
    if(curr_tab==0){
        _tab_wave->setTabEnabled(0,true);
        _tab_wave->setTabEnabled(1,false);
        _tab_wave->setTabEnabled(2,false);
    }
    else if(curr_tab==1){
        _tab_wave->setTabEnabled(0,false);
        _tab_wave->setTabEnabled(1,true);
        _tab_wave->setTabEnabled(2,false);
    }
    else{
        _tab_wave->setTabEnabled(0,false);
        _tab_wave->setTabEnabled(1,false);
        _tab_wave->setTabEnabled(2,true);
    }
}

WaveWidget::~WaveWidget()
{
    
}
