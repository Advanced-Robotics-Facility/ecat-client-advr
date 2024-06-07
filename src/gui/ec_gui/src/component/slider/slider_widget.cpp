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
                            const slider_info_t slider_info_s,
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
    name_label->setText(name);
    _slider_name = name.toStdString();

    _slider_enabled = findChild<QCheckBox *>("SliderEnabled");

    auto slider_name_layout = findChild<QVBoxLayout *>("sliderNameLayout");
    auto valuebox_name_layout = findChild<QVBoxLayout *>("valueboxNameLayout");
    auto unit_layout = findChild<QVBoxLayout *>("unitLayout");
    auto wave_layout = findChild<QVBoxLayout *>("waveLayout");

    _tab_name_wid = new QTabWidget();
    
    for(int i=0;i<slider_info_s.slider_name.size();i++){

        QLabel *slider_label = new QLabel(this);
        QDoubleSpinBox *value_box = new QDoubleSpinBox(this);
        QLabel *unit_label = new QLabel(this);

        slider_label->setText(QString::fromStdString(slider_info_s.slider_name[i]));
        slider_label->setMinimumSize(0,25);
        slider_label->setMaximumSize(150,25);
        slider_name_layout->addWidget(slider_label,0, Qt::AlignTop);

        //connect(value_box, &QDoubleSpinBox::editingFinished,
        //        std::bind(&SliderWidget::on_spinbox_clicked, this, i)
        //        );

        value_box->setMinimumSize(0,25);
        value_box->setMaximumSize(150,25);
        valuebox_name_layout->addWidget(value_box,0, Qt::AlignTop);

        unit_label->setText(QString::fromStdString(slider_info_s.slider_unit[i]));
        unit_label->setMinimumSize(50,25);
        unit_label->setMaximumSize(50,25);
        unit_layout->addWidget(unit_label,0, Qt::AlignTop);

        auto tab_layout = new QVBoxLayout();
        auto page_wid = new QWidget();
        auto wave_wid = new WaveWidget(value_box,
                                       slider_info_s.slider_property[i],
                                       QString::fromStdString(slider_info_s.slider_min[i]),
                                       QString::fromStdString(slider_info_s.slider_max[i]),
                                       slider_info_s.slider_decimal[i],
                                       QString::fromStdString(slider_info_s.slider_unit[i]));
        _wave_v.push_back(wave_wid);
        if(slider_info_s.slider_property[i]==SliderProperty::AVAILABLE){
            tab_layout->addWidget(wave_wid);
            page_wid->setLayout(tab_layout);
            _tab_name_wid->insertTab(i,page_wid, QString::fromStdString(slider_info_s.slider_name[i]));
        }
    }
    
    wave_layout->addWidget(_tab_name_wid);
    disable_slider();
}


void SliderWidget::on_spinbox_clicked(int i)
{
    _tab_name_wid->setCurrentIndex(i);
}

void SliderWidget::align_spinbox(int i,double value)
{
    _wave_v[i]->align_spinbox(value);
}

void SliderWidget::align_all_spinbox(double value)
{
    for(int i=0;i<_wave_v.size();i++){
        _wave_v[i]->align_spinbox(value);
    }
}

void SliderWidget::align_spinbox(int i)
{
    _wave_v[i]->align_spinbox();
}

void SliderWidget::align_all_spinbox()
{
    for(int i=0;i<_wave_v.size();i++){
        _wave_v[i]->align_spinbox();
    }
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

void SliderWidget::set_spinbox_value(int i,double actual_spinbox_value)
{
    _wave_v[i]->set_spinbox_value(actual_spinbox_value);
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
