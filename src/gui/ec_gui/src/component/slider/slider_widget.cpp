#include "slider_widget.h"
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

    QFile file(":/component/slider/slider_widget.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;
    
    
}
}

SliderWidget::SliderWidget (const QString&  name,
                            double init_value,
                            const QString&  min,
                            const QString&  max,
                            const QString& unit,
                            std::vector<std::string> pid_string,
                            std::vector<double> gains,
                            QWidget *parent) :
    QWidget(parent),
    _callback_enabled(true)
{
    /* Create GUI layout */
    auto * ui = ::LoadUiFile(this);

    auto * layout = new QVBoxLayout;
    layout->addWidget(ui);

    setLayout(layout);

    auto name_label = findChild<QLabel *>("SliderLabel");
    name_label->installEventFilter(this);
    name_label->setText(name);
    _slider_name = name.toStdString();

    _slider_enabled = findChild<QCheckBox *>("SliderEnabled");

    _slider_spinbox_fct=100;
    
    auto min_label = findChild<QLabel *>("Min");
    min_label->setText(min+"   "+unit+"       ");

    auto max_label = findChild<QLabel *>("Max");
    max_label->setText(max+"   "+unit+"       ");

    auto unit_label = findChild<QLabel *>("Unit");
    unit_label->setText(unit);

    _valueslider=findChild<QSlider *>("ValueSlider");
    _valueslider->setMaximum(_slider_spinbox_fct*max.toDouble());
    _valueslider->setMinimum(_slider_spinbox_fct*min.toDouble());
    _valueslider->setTickInterval(1);

    if(init_value>max.toDouble()){
        _valueslider->setValue(max.toDouble());
    }
    else if(init_value<min.toDouble()){
        _valueslider->setValue(min.toDouble());
    }
    else{
        _valueslider->setValue(init_value);
    }

    // connect slider to spinbox
    connect(_valueslider, &QSlider::valueChanged,
             std::bind(&SliderWidget::on_slider_changed, this));


    _valuebox=findChild<QDoubleSpinBox *>("ValueBox");
    _valuebox->setMaximum(max.toDouble());
    _valuebox->setMinimum(min.toDouble());

    if(init_value>max.toDouble()){
        _valuebox->setValue(max.toDouble());
    }
    else if(init_value<min.toDouble()){
        _valuebox->setValue(min.toDouble());
    }
    else{
        _valuebox->setValue(init_value);
    }

    _valuebox->setDecimals(6);
    _valuebox->setSingleStep(1/((double)_slider_spinbox_fct));

     // connect spinbox to slider

    _valuebox->setKeyboardTracking(false);

    connect(_valuebox, &QDoubleSpinBox::editingFinished,
           std::bind(&SliderWidget::on_spinbox_changed, this)
           );


    _slider_filtered=std::make_shared<SecondOrderFilter<double>>(12.0,1.0,1.0,init_value);

    _calibration_layout= findChild<QVBoxLayout *>("calibrationLayout");
    if(!gains.empty()){
        _pid_layout=findChild<QHBoxLayout *>("PID_Layout");

        _wid_calib= new SliderWidgetCalib("",gains,pid_string);

        _pid_layout->addWidget(_wid_calib,0, Qt::AlignTop);
    }

    _actual_slider_value=init_value;

    // find wave tab.
    _tab_wave = parent->findChild<QTabWidget *>("tabWave");

    _sine_a=findChild<QDoubleSpinBox *>("Sine_A");
    _sine_a->setMaximum(max.toDouble());
    _sine_f=findChild<QDoubleSpinBox *>("Sine_F");
    _sine_t=findChild<QDoubleSpinBox *>("Sine_T");


    _square_a=findChild<QDoubleSpinBox *>("Square_A");
    _square_a->setMaximum(max.toDouble());
    _square_f=findChild<QDoubleSpinBox *>("Square_F");
    _square_t=findChild<QDoubleSpinBox *>("Square_T");

    disable_slider();

}

void SliderWidget::on_slider_changed()
{

    double value = _valueslider->value();
        
    // update spinbox
    _valuebox->setValue(value/((double)_slider_spinbox_fct));

}

void SliderWidget::on_spinbox_changed()
{
    double value = _valuebox->value();
    _valuebox->setValue(value);

    // update slider
    _valueslider->blockSignals(true);
    _valueslider->setValue(int(_slider_spinbox_fct*value));
    _valueslider->blockSignals(false);   
}
void SliderWidget::align_spinbox(double value)
{
    _valuebox->setValue(value);

    // update slider
    _valueslider->blockSignals(true);
    _valueslider->setValue(int(_slider_spinbox_fct*value));
    _valueslider->blockSignals(false);
}

void SliderWidget::align_spinbox()
{
    _valuebox->setValue(_actual_slider_value);

    // update slider
    _valueslider->blockSignals(true);
    _valueslider->setValue(int(_slider_spinbox_fct*_actual_slider_value));
    _valueslider->blockSignals(false);
}



void SliderWidget::disable_slider()
{
    enable_tab_wave();

    _valueslider->setEnabled(false);
    _valuebox->setEnabled(false);
}
void SliderWidget::enable_slider()
{
      disable_tab_wave();

    if(_slider_enabled->isChecked()){
        _valueslider->setEnabled(true);
        _valuebox->setEnabled(true);
    }
}

void SliderWidget::hide_slider_enabled()
{
    _slider_enabled->hide();
}

void SliderWidget::enable_slider_enabled()
{
    _slider_enabled->setEnabled(true);
}
void SliderWidget::disable_slider_enabled()
{
    _slider_enabled->setEnabled(false);
}


bool SliderWidget::is_slider_enabled()
{
    return _slider_enabled->isChecked();
}

void SliderWidget::check_slider_enabled()
{
     _slider_enabled->setChecked(true);
}

void SliderWidget::uncheck_slider_enabled()
{
    _slider_enabled->setChecked(false);
}

double SliderWidget::get_spinbox_value()
{
    return _valuebox->value();
}

double SliderWidget::get_actual_slider_value()
{
    return _actual_slider_value;
}

void SliderWidget::set_actual_slider_value(double actual_slider_value)
{
    _actual_slider_value=actual_slider_value;
}

std::string SliderWidget::get_slider_name()
{
    return _slider_name;
}

SecondOrderFilter<double>::Ptr SliderWidget::get_filter()
{
   return _slider_filtered;
}


SliderWidgetCalib* SliderWidget::get_wid_calibration()
{
    return _wid_calib;
}

void SliderWidget::remove_calibration()
{
    delete _calibration_layout;
}

double SliderWidget::compute_wave(double t)
{
    double fx=0;
    switch (_tab_wave->currentIndex()){
    case 0:
        fx = _slider_filtered->process(_valuebox->value());
        break;
    case 1:
        fx =  _sine_a->value() * std::sin (2*M_PI*_sine_f->value()*t + _sine_t->value());
        break;
    case 2:
        fx=-1*_square_a->value();
        if(std::signbit(std::sin (2*M_PI*_square_f->value()*t + _square_t->value()))){
            fx=1*_square_a->value();
        }
        break;
    
    default:
        break;
    }
    return fx;
}

void SliderWidget::enable_tab_wave()
{
    _tab_wave->setTabEnabled(0,true);
    _tab_wave->setTabEnabled(1,true);
    _tab_wave->setTabEnabled(2,true);
}
void SliderWidget::disable_tab_wave()
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

SliderWidget::~SliderWidget()
{
    
}
