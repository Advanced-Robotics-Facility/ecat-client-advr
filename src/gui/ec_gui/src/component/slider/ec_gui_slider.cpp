#include "ec_gui_slider.h"

EcGuiSlider::EcGuiSlider(QWidget *parent) :
    QWidget(parent)
{
    _sliders_motorlayout  = parent->findChild<QVBoxLayout *>("motorSliders");
    _sliders_valvelayout = parent->findChild<QVBoxLayout *>("valveSliders");
    _sliders_pumplayout = parent->findChild<QVBoxLayout *>("pumpSliders");
}

void EcGuiSlider::create_sliders(SSI device_info)
{
    
    delete_sliders();

    for ( auto &[device_id, device_type, device_pos] : device_info ) {
        if(ec_motors.count(device_type)>0){
            std::string motor_name_s="motor_"+std::to_string(device_id);
            QString motor_name = QString::fromStdString(motor_name_s);
            auto wid_motor=new SliderWidget(motor_name,motor_info,this);
            _slider_map.motor_sw_map[device_id]=wid_motor;
            _sliders_motorlayout->addWidget(wid_motor,0, Qt::AlignTop);
        }
        else if(device_type==iit::ecat::HYQ_KNEE){
            std::string valve_name_s="valve_"+std::to_string(device_id);
            QString valve_name = QString::fromStdString(valve_name_s);
            auto wid_valve=new SliderWidget(valve_name,valve_info,this);
            _slider_map.valve_sw_map[device_id]=wid_valve;
            _sliders_valvelayout->addWidget(wid_valve,0, Qt::AlignTop);

        }
        else if(device_type==iit::ecat::HYQ_HPU){
            std::string pump_name_s="pump_"+std::to_string(device_id);
            QString pump_name = QString::fromStdString(pump_name_s);
            auto wid_pump=new SliderWidget(pump_name,pump_info,this);
            _slider_map.pump_sw_map[device_id]=wid_pump;
            _sliders_pumplayout->addWidget(wid_pump,0, Qt::AlignTop);
        }
    }
}

void EcGuiSlider::delete_sliders()
{
    delete_items(_sliders_motorlayout->layout());
    _slider_map.motor_sw_map.clear();

    delete_items(_sliders_valvelayout->layout());
    _slider_map.valve_sw_map.clear();

    delete_items(_sliders_pumplayout->layout());
    _slider_map.pump_sw_map.clear();    
}

void EcGuiSlider::reset_sliders()
{
    for (auto& [slave_id, slider_wid]:_slider_map.motor_sw_map)
    {
        slider_wid->align_spinbox();
    }
    for (auto& [slave_id, slider_wid]:_slider_map.valve_sw_map){
        slider_wid->align_spinbox(0.0);
    }
    for (auto& [slave_id, slider_wid]:_slider_map.pump_sw_map){
        slider_wid->align_spinbox(0.0);
    }
}
void EcGuiSlider::enable_sliders()
{
    for (auto& [slave_id, slider_wid]:_slider_map.motor_sw_map){
        slider_wid->enable_slider();
    }
    for (auto& [slave_id, slider_wid]:_slider_map.valve_sw_map){
        slider_wid->enable_slider();
    }
    for (auto& [slave_id, slider_wid]:_slider_map.pump_sw_map){
        slider_wid->enable_slider();
    }
}
void EcGuiSlider::disable_sliders()
{
    for (auto& [slave_id, slider_wid]:_slider_map.motor_sw_map){
        slider_wid->disable_slider();
    }

    for (auto& [slave_id, slider_wid]:_slider_map.valve_sw_map){
         slider_wid->disable_slider();
    }
    for (auto& [slave_id, slider_wid]:_slider_map.pump_sw_map){
         slider_wid->disable_slider();
    }
}

void EcGuiSlider::set_sliders_filter(double st)
{
    for (auto& [slave_id, slider_wid]:_slider_map.motor_sw_map){
         slider_wid->set_filter(st);
    }

    for (auto& [slave_id, slider_wid]:_slider_map.valve_sw_map){
         slider_wid->set_filter(st);
    }
    for (auto& [slave_id, slider_wid]:_slider_map.pump_sw_map){
         slider_wid->set_filter(st);
    }
}

void EcGuiSlider::delete_items(QLayout * layout)
{
    if ( layout != NULL )
    {
        QLayoutItem* item;
        while ( ( item = layout->takeAt( 0 ) ) != NULL )
        {
            delete item->widget();
            delete item;
        }
    }
}

EcGuiSlider::slider_map_t EcGuiSlider::get_sliders()
{
    return _slider_map;
}

EcGuiSlider::~EcGuiSlider()
{

}
