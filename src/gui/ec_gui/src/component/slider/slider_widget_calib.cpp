#include "slider_widget_calib.h"
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

    QFile file(":/component/slider/slider_widget_calib.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;
    
    
}
}

SliderWidgetCalib::SliderWidgetCalib (const QString&  joint_name,
                                      std::vector<std::string> sliders_type,
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
    _jname->hide();

    auto slider_name_layout = findChild<QVBoxLayout *>("sliderNameLayout");
    auto valuebox_name_layout = findChild<QVBoxLayout *>("valueboxNameLayout");

    _slider_numb=sliders_type.size();        
    for(int i=0;i<_slider_numb;i++){
        QLabel *slider_label = new QLabel(this);
        QDoubleSpinBox *value_box = new QDoubleSpinBox(this);

        slider_label->setText(QString::fromStdString(sliders_type[i]));
        slider_label->setMaximumWidth(150);
        slider_label->setMaximumHeight(25);
        slider_name_layout->addWidget(slider_label,0, Qt::AlignTop);

        // connect spinbox to slider
        connect(value_box, &QDoubleSpinBox::editingFinished,
                std::bind(&SliderWidgetCalib::on_spinbox_changed, this, i)
                );

        value_box->setMaximum(1000);
        value_box->setMinimum(0);
        value_box->setDecimals(3);
        value_box->setSingleStep(0.001);
        value_box->setValue(0.0);
        value_box->setKeyboardTracking(false);
        value_box->setMaximumWidth(150);
        value_box->setMaximumHeight(28);
        valuebox_name_layout->addWidget(value_box,0, Qt::AlignTop);

        _slidertype.push_back(slider_label);
        _valuebox.push_back(value_box);
        _valuebox_filtered.push_back(std::make_shared<SecondOrderFilter<double>>(12.0,1.0,1.0,0.0));
    }
    

}

void SliderWidgetCalib::on_spinbox_changed(int i)
{

      double value = _valuebox[i]->value();
      _valuebox[i]->setValue(value);
}

SliderWidgetCalib::~SliderWidgetCalib()
{
    
}

double SliderWidgetCalib::get_slider_value(int type)
{
    return _valuebox[type]->value();
}

 SecondOrderFilter<double>::Ptr SliderWidgetCalib::get_slider_filter(int index)
{
   return _valuebox_filtered[index];
}

int SliderWidgetCalib::get_slider_numb()
{
    return _slider_numb;
}

void SliderWidgetCalib::disable_slider_calib(int index)
{
    _slidertype[index]->setDisabled(true);
    _valuebox[index]->setDisabled(true);
}

void SliderWidgetCalib::enable_slider_calib(int index)
{
    _slidertype[index]->setDisabled(false);
    _valuebox[index]->setDisabled(false);
}

void SliderWidgetCalib::hide_slider_calib(int index)
{
    _slidertype[index]->hide();
    _valuebox[index]->hide();
}


void SliderWidgetCalib::set_filter(double st)
{
    for(int i=0;i<_slider_numb;i++){
        _valuebox_filtered[i]->reset(_valuebox[i]->value());
        _valuebox_filtered[i]->setTimeStep(st);
    }
}

void SliderWidgetCalib::filtering(std::vector<float>& calib_filtered)
{
    calib_filtered.clear();
    for(int i=0;i<_slider_numb;i++){
        calib_filtered.push_back(_valuebox_filtered[i]->process(_valuebox[i]->value()));
    }
}
