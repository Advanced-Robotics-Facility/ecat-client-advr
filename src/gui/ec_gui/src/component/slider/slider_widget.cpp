#include "slider_widget.h"

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

SliderWidget::SliderWidget (const QString&  joint_name,
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

    _jname = findChild<QLabel *>("JointLabel");
    _jname->installEventFilter(this);
    _jname->setText(joint_name);
    _joint_name = joint_name.toStdString();
    
    _min = findChild<QLabel *>("Min");
    _min->setText(min+"   "+unit+"       ");

    _max = findChild<QLabel *>("Max");
    _max->setText(max+"   "+unit+"       ");

    _unit = findChild<QLabel *>("Unit");
    _unit->setText(unit);

    _slider_spinbox_fct=100;

    _valueslider=findChild<QSlider *>("ValueSlider");
    _valueslider->setMaximum(_slider_spinbox_fct*max.toDouble());
    _valueslider->setMinimum(_slider_spinbox_fct*min.toDouble());
    _valueslider->setTickInterval(1);

    if(init_value>max.toDouble())
    {
        _valueslider->setValue(max.toDouble());
    }
    else if(init_value<min.toDouble())
    {
        _valueslider->setValue(min.toDouble());
    }
    else
    {
        _valueslider->setValue(init_value);
    }

    _valuebox=findChild<QDoubleSpinBox *>("ValueBox");
    _valuebox->setMaximum(max.toDouble());
    _valuebox->setMinimum(min.toDouble());

    if(init_value>max.toDouble())
    {
        _valuebox->setValue(max.toDouble());
    }
    else if(init_value<min.toDouble())
    {
        _valuebox->setValue(min.toDouble());
    }
    else
    {
        _valuebox->setValue(init_value);
    }

    _valuebox->setDecimals(6);

    _valuebox->setSingleStep(1/((double)_slider_spinbox_fct));

    // connect slider to spinbox
     connect(_valueslider, &QSlider::valueChanged,
            std::bind(&SliderWidget::on_slider_changed, this)
            );

     // connect spinbox to slider

    _valuebox->setKeyboardTracking(false);

    connect(_valuebox, &QDoubleSpinBox::editingFinished,
           std::bind(&SliderWidget::on_spinbox_changed, this)
           );

    _joint_enabled = findChild<QCheckBox *>("JointEnabled");

    _slider_filtered=std::make_shared<SecondOrderFilter<double>>(12.0,1.0,1.0,init_value);

    _pid_layout=findChild<QHBoxLayout *>("PID_Layout");

    _wid_calib= new SliderWidgetCalib("",gains,pid_string);

    _pid_layout->addWidget(_wid_calib,0, Qt::AlignTop);

    _actual_slider_value=init_value;


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
    _valueslider->setEnabled(false);
    _valuebox->setEnabled(false);
}
void SliderWidget::enable_slider()
{
    _valueslider->setEnabled(true);
    _valuebox->setEnabled(true);
}

void SliderWidget::enable_joint_enabled()
{
    _joint_enabled->setEnabled(true);
}
void SliderWidget::disable_joint_enabled()
{
    _joint_enabled->setEnabled(false);
}


bool SliderWidget::is_joint_enabled()
{
    return _joint_enabled->isChecked();
}

void SliderWidget::check_joint_enabled()
{
     _joint_enabled->setChecked(true);
}

void SliderWidget::uncheck_joint_enabled()
{
    _joint_enabled->setChecked(false);
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

std::string SliderWidget::get_joint_name()
{
    return _joint_name;
}

SecondOrderFilter<double>::Ptr SliderWidget::get_filer()
{
   return _slider_filtered;
}


SliderWidgetCalib* SliderWidget::get_wid_calibration()
{
    return _wid_calib;
}


SliderWidget::~SliderWidget()
{
    
}
