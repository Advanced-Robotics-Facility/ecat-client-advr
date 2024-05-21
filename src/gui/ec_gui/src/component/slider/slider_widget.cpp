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
    // find wave tab.
    _tab_name_wid = findChild<QTabWidget *>("tabNameWidget");
    _tab_wave = findChild<QTabWidget *>("tabWave");

    auto slider_name_layout = findChild<QVBoxLayout *>("sliderNameLayout");
    auto valuebox_name_layout = findChild<QVBoxLayout *>("valueboxNameLayout");

    for(int i=0;i<pid_string.size();i++){

        QLabel *slider_label = new QLabel(this);
        QDoubleSpinBox *value_box = new QDoubleSpinBox(this);

        slider_label->setText(QString::fromStdString(pid_string[i]));
        slider_label->setMaximumWidth(150);
        slider_label->setMinimumHeight(25);
        slider_label->setMaximumHeight(25);
        slider_name_layout->addWidget(slider_label,0, Qt::AlignTop);

        connect(value_box, &QDoubleSpinBox::editingFinished,
                std::bind(&SliderWidget::on_spinbox_clicked, this, i)
                );

        value_box->setMaximumWidth(150);
        value_box->setMinimumHeight(25);
        value_box->setMaximumHeight(25);
        valuebox_name_layout->addWidget(value_box,0, Qt::AlignTop);

        auto tab_layout = new QHBoxLayout();
        auto page_wid = new QWidget();
        auto wave_wid = new WaveWidget(init_value,value_box,min,max,unit);
        _wave_v.push_back(wave_wid);
        tab_layout->addWidget(wave_wid);
        page_wid->setLayout(tab_layout);
        
        if(i==0){
            _tab_name_wid->setTabText(0,QString::fromStdString(pid_string[i]));
        }
        else{
            _tab_name_wid->insertTab(i,page_wid, QString::fromStdString(pid_string[i]));
        }

    }

    disable_slider();
}


void SliderWidget::on_spinbox_clicked(int i)
{
    _tab_name_wid->setCurrentIndex(i);
}

void SliderWidget::align_spinbox(double value)
{

}

void SliderWidget::align_spinbox()
{
    
}


void SliderWidget::disable_slider()
{
    for(int i=0;i<_wave_v.size();i++){
        _wave_v[i]->disable_slider();
    }
}
void SliderWidget::enable_slider()
{
    if(_slider_enabled->isChecked()){
        for(int i=0;i<_wave_v.size();i++){
            _wave_v[i]->enable_slider();
        }
    }
}

void SliderWidget::hide_slider_enabled()
{
    _slider_enabled->hide();
}

void SliderWidget::enable_slider_enabled()
{
    _slider_enabled->setEnabled(true);
    _tab_name_wid->setEnabled(true);
}
void SliderWidget::disable_slider_enabled()
{
    _slider_enabled->setEnabled(false);
    if(!_slider_enabled->isChecked()){
        _tab_name_wid->setEnabled(false);
    }
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

double SliderWidget::get_spinbox_value(int i)
{
    return _wave_v[i]->get_spinbox_value();
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


void SliderWidget::set_filter(double st)
{
    for(int i=0;i<_wave_v.size();i++){
        _wave_v[i]->set_filter(st);
    }
}

double SliderWidget::compute_wave(int i,double t)
{
    double fx=_wave_v[i]->compute_wave(t);
    return fx;
}


SliderWidget::~SliderWidget()
{
    
}
