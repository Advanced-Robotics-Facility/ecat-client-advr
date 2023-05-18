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

    QFile file(":/components/slider_widget_calib.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;
    
    
}
}

SliderWidgetCalib::SliderWidgetCalib (const QString&  joint_name,
                                      float ctrl_type,
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
        _valuebox[i]->setKeyboardTracking(false);
    }

    if(sliders_type.size()>_slidertype.size())
    {
        throw std::runtime_error("Error _slidertype");
        return;
    }
    else
    {
        int i=0,k=0;
        bool set_init_value=false;

        if((!init_value.empty())&&(init_value.size()==sliders_type.size()))
            set_init_value=true;

        for(i;i<sliders_type.size();i++)
        {
            _slidertype[i]->setText(QString::fromStdString(sliders_type[i]));

            // connect spinbox to slider
            connect(_valuebox[i], &QDoubleSpinBox::editingFinished,
                   std::bind(&SliderWidgetCalib::on_spinbox_changed, this, i)
                   );

            if(_slidertype[i]->text().toStdString()=="P")
            {
                if(ctrl_type == 0x3B)
                {
                    _valuebox[i]->setMaximum(250);
                    _valuebox[i]->setMinimum(0);
                    _valuebox[i]->setDecimals(0);
                    _valuebox[i]->setSingleStep(1);
                }
                else if(ctrl_type == 0x71)
                {
                    _valuebox[i]->setMaximum(50);
                    _valuebox[i]->setMinimum(0);
                    _valuebox[i]->setDecimals(0);
                    _valuebox[i]->setSingleStep(1);
                }
                else if(ctrl_type == 0xD4)
                {
                    _valuebox[i]->setMaximum(3000);
                    _valuebox[i]->setMinimum(0);
                    _valuebox[i]->setDecimals(0);
                    _valuebox[i]->setSingleStep(1);
                }
                else
                {
                    throw std::runtime_error("Error: Control mode not recognized");
                }
                    
            }
            
            else if(_slidertype[i]->text().toStdString()=="I")
            {
                if(ctrl_type == 0x3B)
                {
                    _valuebox[i]->setMaximum(0);
                    _valuebox[i]->setMinimum(0);
                    _valuebox[i]->setDecimals(0);
                    _valuebox[i]->setSingleStep(1);
                }
                else if(ctrl_type == 0x71)
                {
                    _valuebox[i]->setMaximum(0);
                    _valuebox[i]->setMinimum(0);
                    _valuebox[i]->setDecimals(0);
                    _valuebox[i]->setSingleStep(1);
                }
                else if(ctrl_type == 0xD4)
                {
                    _valuebox[i]->setMaximum(0);
                    _valuebox[i]->setMinimum(0);
                    _valuebox[i]->setDecimals(0);
                    _valuebox[i]->setSingleStep(1);
                }
                else
                {
                    throw std::runtime_error("Error: Control mode not recognized");
                }
                    
            }
            
            else if(_slidertype[i]->text().toStdString()=="D")
            {
                if(ctrl_type == 0x3B)
                {
                    _valuebox[i]->setMaximum(40);
                    _valuebox[i]->setMinimum(0);
                    _valuebox[i]->setDecimals(0);
                    _valuebox[i]->setSingleStep(1);
                }
                else if(ctrl_type == 0x71)
                {
                    _valuebox[i]->setMaximum(0);
                    _valuebox[i]->setMinimum(0);
                    _valuebox[i]->setDecimals(0);
                    _valuebox[i]->setSingleStep(1);
                }
                else if(ctrl_type == 0xD4)
                {
                    _valuebox[i]->setMaximum(35);
                    _valuebox[i]->setMinimum(0);
                    _valuebox[i]->setDecimals(0);
                    _valuebox[i]->setSingleStep(1);
                }
                else
                {
                    throw std::runtime_error("Error: Control mode not recognized");
                }
                    
            }
          
            else if(_slidertype[i]->text().toStdString()=="Tau_p")
            {
                _valuebox[i]->setMaximum(1.5);
                _valuebox[i]->setMinimum(1);
                _valuebox[i]->setDecimals(1);
                _valuebox[i]->setSingleStep(0.1);
            }
            else if(_slidertype[i]->text().toStdString()=="Tau_d")
            {
                _valuebox[i]->setMaximum(0.008);
                _valuebox[i]->setMinimum(0.005);
                _valuebox[i]->setDecimals(3);
                _valuebox[i]->setSingleStep(0.001);
            }
            else if(_slidertype[i]->text().toStdString()=="Tau_fc")
            {
                _valuebox[i]->setMaximum(0.85);
                _valuebox[i]->setMinimum(0);
                _valuebox[i]->setDecimals(2);
                _valuebox[i]->setSingleStep(0.01);
            }

           
            if(set_init_value)
            {
                _valuebox[i]->setValue(init_value[i]);
                _valuebox_filtered.push_back(std::make_shared<SecondOrderFilter<double>>(12.0,1.0,1.0,init_value[i]));
            }

        }
        for(k=i;k<_slidertype.size();k++)
        {
            _slidertype[k]->hide();
            _valuebox[k]->hide();
        }

        _slider_numb=sliders_type.size();
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

