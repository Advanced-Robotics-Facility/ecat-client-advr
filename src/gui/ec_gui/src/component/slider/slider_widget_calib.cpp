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
                                      std::vector<double> init_value,
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

    _slidertype.push_back(findChild<QLabel *>("SliderType1"));
    _slidertype.push_back(findChild<QLabel *>("SliderType2"));
    _slidertype.push_back(findChild<QLabel *>("SliderType3"));
    _slidertype.push_back(findChild<QLabel *>("SliderType4"));
    _slidertype.push_back(findChild<QLabel *>("SliderType5"));
    _slidertype.push_back(findChild<QLabel *>("SliderType6"));
    _slidertype.push_back(findChild<QLabel *>("SliderType7"));
    _slidertype.push_back(findChild<QLabel *>("SliderType8"));

    _valuebox.push_back(findChild<QDoubleSpinBox *>("ValueBox1"));
    _valuebox.push_back(findChild<QDoubleSpinBox *>("ValueBox2"));
    _valuebox.push_back(findChild<QDoubleSpinBox *>("ValueBox3"));
    _valuebox.push_back(findChild<QDoubleSpinBox *>("ValueBox4"));
    _valuebox.push_back(findChild<QDoubleSpinBox *>("ValueBox5"));
    _valuebox.push_back(findChild<QDoubleSpinBox *>("ValueBox6"));
    _valuebox.push_back(findChild<QDoubleSpinBox *>("ValueBox7"));
    _valuebox.push_back(findChild<QDoubleSpinBox *>("ValueBox8"));

    for(int i=0; i < _valuebox.size();i++)
    {
        _slidertype[i]->hide();
        _valuebox[i]->setKeyboardTracking(false);
        _valuebox[i]->hide();
    }

    if(sliders_type.size()>_slidertype.size())
    {
        throw std::runtime_error("Error valuebox requested greater than 8!");
        return;
    }
    else
    {
        _slider_numb=sliders_type.size();        
        for(int i=0;i<_slider_numb;i++)
        {
            _slidertype[i]->setText(QString::fromStdString(sliders_type[i]));

            // connect spinbox to slider
            connect(_valuebox[i], &QDoubleSpinBox::editingFinished,
                   std::bind(&SliderWidgetCalib::on_spinbox_changed, this, i)
                   );

            _slidertype[i]->show();
            _valuebox[i]->show();
            _valuebox[i]->setMaximum(1000);
            _valuebox[i]->setMinimum(0);
            _valuebox[i]->setDecimals(3);
            _valuebox[i]->setSingleStep(0.001);
            _valuebox[i]->setValue(init_value[i]);
            _valuebox_filtered.push_back(std::make_shared<SecondOrderFilter<double>>(12.0,1.0,1.0,init_value[i]));
        }
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

