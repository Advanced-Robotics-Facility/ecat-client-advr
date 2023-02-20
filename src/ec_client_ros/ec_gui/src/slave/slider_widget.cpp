#include "slider_widget.h"
#include <iostream>


inline void initSlidersResource()
{
    Q_INIT_RESOURCE(slave_resources);
}

namespace
{

QWidget * LoadUiFile(QWidget * parent)
{
    
    initSlidersResource();
    
    QUiLoader loader;

    QFile file(":/slider_widget.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;
    
    
}
}

SliderWidget::SliderWidget (const QString&  joint_name,
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

    _slidertype.push_back(findChild<QLabel *>("SliderType1"));
    _slidertype.push_back(findChild<QLabel *>("SliderType2"));
    _slidertype.push_back(findChild<QLabel *>("SliderType3"));
    _slidertype.push_back(findChild<QLabel *>("SliderType4"));
    _slidertype.push_back(findChild<QLabel *>("SliderType5"));
    _slidertype.push_back(findChild<QLabel *>("SliderType6"));
    _slidertype.push_back(findChild<QLabel *>("SliderType7"));
    _slidertype.push_back(findChild<QLabel *>("SliderType8"));

//    _valueslide.push_back(findChild<QSlider *>("ValueSlide1"));
//    _valueslide.push_back(findChild<QSlider *>("ValueSlide2"));
//    _valueslide.push_back(findChild<QSlider *>("ValueSlide3"));
//    _valueslide.push_back(findChild<QSlider *>("ValueSlide4"));
//    _valueslide.push_back(findChild<QSlider *>("ValueSlide5"));

    _valuebox.push_back(findChild<QDoubleSpinBox *>("ValueBox1"));
    _valuebox.push_back(findChild<QDoubleSpinBox *>("ValueBox2"));
    _valuebox.push_back(findChild<QDoubleSpinBox *>("ValueBox3"));
    _valuebox.push_back(findChild<QDoubleSpinBox *>("ValueBox4"));
    _valuebox.push_back(findChild<QDoubleSpinBox *>("ValueBox5"));
    _valuebox.push_back(findChild<QDoubleSpinBox *>("ValueBox6"));
    _valuebox.push_back(findChild<QDoubleSpinBox *>("ValueBox7"));
    _valuebox.push_back(findChild<QDoubleSpinBox *>("ValueBox8"));

    _radiobox.push_back(findChild<QRadioButton *>("RadioButton1"));
    _radiobox.push_back(findChild<QRadioButton *>("RadioButton2"));
    _radiobox.push_back(findChild<QRadioButton *>("RadioButton3"));
    _radiobox.push_back(findChild<QRadioButton *>("RadioButton4"));
    _radiobox.push_back(findChild<QRadioButton *>("RadioButton5"));
    _radiobox.push_back(findChild<QRadioButton *>("RadioButton6"));
    _radiobox.push_back(findChild<QRadioButton *>("RadioButton7"));
    _radiobox.push_back(findChild<QRadioButton *>("RadioButton8"));


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

            // connect slider to spinbox
//            connect(_valueslide[i], &QSlider::valueChanged,
//                   std::bind(&SliderWidget::on_slider_changed, this, i)
//                   );

            // connect spinbox to slider
            connect(_valuebox[i], &QDoubleSpinBox::editingFinished,
                   std::bind(&SliderWidget::on_spinbox_changed, this, i)
                   );

            // POSITION, MIN POS, MAX POS, HOMING POSITION and TRAJECTORY [Ampl,Theta]
            _valuebox[i]->setMaximum(10);
            _valuebox[i]->setMinimum(-10);
            _valuebox[i]->setDecimals(2);
            _valuebox[i]->setSingleStep(0.01);

            if((_slidertype[i]->text().toStdString()=="P")||
               (_slidertype[i]->text().toStdString()=="I")||
               (_slidertype[i]->text().toStdString()=="D"))
            {
                _valuebox[i]->setMaximum(200000);
                _valuebox[i]->setMinimum(0);
                _valuebox[i]->setDecimals(2);
                _valuebox[i]->setSingleStep(0.01);
            }
            else if(_slidertype[i]->text().toStdString()=="[rad/s]")
            {
                _valuebox[i]->setMaximum(100);
                _valuebox[i]->setMinimum(-100);
                _valuebox[i]->setDecimals(2);
                _valuebox[i]->setSingleStep(0.01);
            }
            else if(_slidertype[i]->text().toStdString()=="Max_vel")
            {
                _valuebox[i]->setMaximum(100);
                _valuebox[i]->setMinimum(0);
                _valuebox[i]->setDecimals(2);
                _valuebox[i]->setSingleStep(0.01);
            }
            else if(_slidertype[i]->text().toStdString()=="[Nm]")
            {
                _valuebox[i]->setMaximum(100);
                _valuebox[i]->setMinimum(-100);
                _valuebox[i]->setDecimals(2);
                _valuebox[i]->setSingleStep(0.01);
            }
            else if(_slidertype[i]->text().toStdString()=="Max_Torq")
            {
                _valuebox[i]->setMaximum(100);
                _valuebox[i]->setMinimum(0);
                _valuebox[i]->setDecimals(2);
                _valuebox[i]->setSingleStep(0.01);
            }
            else if((_slidertype[i]->text().toStdString()=="Tau_p")||
                    (_slidertype[i]->text().toStdString()=="Tau_d")||
                    (_slidertype[i]->text().toStdString()=="Tau_fc"))
            {
                _valuebox[i]->setMaximum(10);
                _valuebox[i]->setMinimum(0);
                _valuebox[i]->setDecimals(3);
                _valuebox[i]->setSingleStep(0.001);
            }

            else if (_slidertype[i]->text().toStdString()=="[A]")
            {
                _valuebox[i]->setMaximum(50);
                _valuebox[i]->setMinimum(-50);
                _valuebox[i]->setDecimals(2);
                _valuebox[i]->setSingleStep(0.01);
            }
            else if (_slidertype[i]->text().toStdString()=="[Max A]")
            {
                _slidertype[i]->setText("[A]");
                _valuebox[i]->setMaximum(50);
                _valuebox[i]->setMinimum(0);
                _valuebox[i]->setDecimals(2);
                _valuebox[i]->setSingleStep(0.01);
            }

            else if(_slidertype[i]->text().toStdString()=="Freq [Hz]")
            {
                _valuebox[i]->setMaximum(1000);
                _valuebox[i]->setMinimum(0);
                _valuebox[i]->setDecimals(2);
                _valuebox[i]->setSingleStep(0.5);
            }

            else if(_slidertype[i]->text().toStdString()=="Secs [s]")
            {
                _valuebox[i]->setMaximum(60);
                _valuebox[i]->setMinimum(0);
                _valuebox[i]->setDecimals(3);
                _valuebox[i]->setSingleStep(0.001);
            }
            else if((_slidertype[i]->text().toStdString()=="x1_home [s]")||
                    (_slidertype[i]->text().toStdString()=="x2_home [s]")||
                    (_slidertype[i]->text().toStdString()=="x1_smooth [s]")||
                    (_slidertype[i]->text().toStdString()=="x2_smooth [s]")||
                    (_slidertype[i]->text().toStdString()=="x3_smooth [s]")||
                    (_slidertype[i]->text().toStdString()=="x4_smooth [s]"))
            {
                _valuebox[i]->setMaximum(120);
                _valuebox[i]->setMinimum(0);
                _valuebox[i]->setDecimals(2);
                _valuebox[i]->setSingleStep(0.5);
            }

            if(set_init_value)
            {
                _valuebox[i]->setValue(init_value[i]);
            }

        }
        for(k=i;k<_slidertype.size();k++)
        {
            _slidertype[k]->hide();
            _valuebox[k]->hide();
        }
        int start_value=0;
        if(_slidertype[0]->text().toStdString()=="[OFF/ON]")
        {
            _valuebox[0]->hide();
            start_value=1;
        }
        for(int j=start_value;j<_radiobox.size();j++)
        {
            _radiobox[j]->hide();
        }
    }

}

void SliderWidget::on_slider_changed(int i)
{

//    double value = _valueslide[i]->value();
    
//    // update spinbox
//    _valuebox[i]->setValue(value);

}

void SliderWidget::on_spinbox_changed(int i)
{

      double value = _valuebox[i]->value();
      _valuebox[i]->setValue(value);
//    // update slider
//    _valueslide[i]->blockSignals(true);
//    _valueslide[i]->setValue(value);
//    _valueslide[i]->blockSignals(false);
    
}

void SliderWidget::setRange(std::vector<double> min, std::vector<double> max)
{
//    if(min.size() != _widget_vec.size())
//    {
//        throw std::invalid_argument("min.size() != joint num");
//    }

//    if(max.size() != _widget_vec.size())
//    {
//        throw std::invalid_argument("max.size() != joint num");
//    }

//    double global_max = -1.;

//    for(int i = 0; i < _widget_vec.size(); i++)
//    {
//        _widget_vec[i].min = min[i];
//        _widget_vec[i].max = max[i];

//        global_max = std::max(global_max, std::fabs(min[i]));
//        global_max = std::max(global_max, std::fabs(max[i]));
//    }

//    _max_stiffness_spinbox->setValue(global_max);
//    on_max_stiffness_changed();
}

void SliderWidget::setInitialValue(std::vector<double> x_0)
{
//    _callback_enabled = false;
    
//    if(x_0.size() != _widget_vec.size())
//    {
//        throw std::invalid_argument("x_0.size() != joint num");
//    }
    
//    on_unlockall_pressed();

//    auto compare_abs = [](double a, double b)
//    {
//        return std::fabs(a) < std::fabs(b);
//    };
    
//    double max_val = *(std::max_element(x_0.begin(), x_0.end(), compare_abs));
//    double curr_max = _max_stiffness_spinbox->value();
    
//    if(curr_max < max_val)
//    {
//        _max_stiffness_spinbox->setValue(max_val);
//        on_max_stiffness_changed();
//    }

//    for(int i = 0; i < _widget_vec.size(); i++)
//    {
//        _widget_vec[i].spinbox->setValue(x_0[i]);
//        on_spinbox_changed(i);
//    }
    
//    _callback_enabled = true;
}


SliderWidget::~SliderWidget()
{
    
}

double SliderWidget::get_slider_value(int type)
{
    return _valuebox[type]->value();
}

double SliderWidget::get_radiobtn_value(int type)
{
    double ret=0;

    if(_radiobox[type]->isChecked())
    {
        ret=1;
    }
    return ret;
}
