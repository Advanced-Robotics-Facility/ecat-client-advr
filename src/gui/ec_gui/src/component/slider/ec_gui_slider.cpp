#include "ec_gui_slider.h"

EcGuiSlider::EcGuiSlider(QWidget *parent) :
    QWidget(parent)
{
    _devicecontrol=parent->findChild<QTabWidget *>("deviceControl");
    _device_list_wid = parent->findChild<QListWidget *>("devicelistWidget");
}

QVBoxLayout* EcGuiSlider::retrieve_slider_layout(const std::string &tab_name,const QStringList &control_mode)
{
    if(_sliders_window_map.count(tab_name)==0){
        int tab_actual_index=_devicecontrol->count();
        _sliders_window_map[tab_name]=new SliderWindow(control_mode,this);
        _devicecontrol->insertTab(tab_actual_index,_sliders_window_map[tab_name],QString::fromStdString(tab_name));
        _devicecontrol->show();
    }
    return _sliders_window_map[tab_name]->get_layout();
}

void EcGuiSlider::create_sliders(SSI device_info)
{
    
    delete_sliders();
    int device_list_index=0;
    for ( auto &[device_id, device_type, device_pos] : device_info ) {
        if(ec_motors.count(device_type)>0){
            std::string motor_name_s="motor_"+std::to_string(device_id);
            QString motor_name = QString::fromStdString(motor_name_s);
            auto wid_motor=new SliderWidget(motor_name,motor_info,this);
            _slider_map.motor_sw_map[device_id]=wid_motor;

            QStringList control_mode= {"Position", "Velocity","Impedance", "Torque","Current", "Idle"};
            auto motor_layout= retrieve_slider_layout("Motors",control_mode);
            motor_layout->addWidget(wid_motor,0, Qt::AlignTop);

            _device_list_wid->addItem(motor_name);
            auto slider_enabled=wid_motor->get_slider_enabled();
            connect(slider_enabled, &QCheckBox::toggled,
                    std::bind(&EcGuiSlider::on_checkbox_clicked, this, slider_enabled,device_list_index)
                    );
            device_list_index++;
        }
        else if(device_type==iit::ecat::HYQ_KNEE){
            std::string valve_name_s="valve_"+std::to_string(device_id);
            QString valve_name = QString::fromStdString(valve_name_s);
            auto wid_valve=new SliderWidget(valve_name,valve_info,this);
            _slider_map.valve_sw_map[device_id]=wid_valve;

            QStringList control_mode= {"Position", "Force","Current", "Idle"};
            auto valve_layout= retrieve_slider_layout("Valves",control_mode);
            valve_layout->addWidget(wid_valve,0, Qt::AlignTop);

            _device_list_wid->addItem(valve_name);
            auto slider_enabled=wid_valve->get_slider_enabled();
            connect(slider_enabled, &QCheckBox::toggled,
                    std::bind(&EcGuiSlider::on_checkbox_clicked, this, slider_enabled,device_list_index)
                    );
            device_list_index++;
        }
        else if(device_type==iit::ecat::HYQ_HPU){
            std::string pump_name_s="pump_"+std::to_string(device_id);
            QString pump_name = QString::fromStdString(pump_name_s);
            auto wid_pump=new SliderWidget(pump_name,pump_info,this);
            _slider_map.pump_sw_map[device_id]=wid_pump;

            QStringList control_mode= {"Pressure", "Idle"};
            auto pump_layout= retrieve_slider_layout("Pumps",control_mode);
            pump_layout->addWidget(wid_pump,0, Qt::AlignTop);

            _device_list_wid->addItem(pump_name);
            auto slider_enabled=wid_pump->get_slider_enabled();
            connect(slider_enabled, &QCheckBox::toggled,
                    std::bind(&EcGuiSlider::on_checkbox_clicked, this, slider_enabled,device_list_index)
                    );
            device_list_index++;
        }
    }
    for(int i=0;i<_device_list_wid->count();i++){
        _device_list_wid->item(i)->setHidden(true);
    }
}

void EcGuiSlider::on_checkbox_clicked(QCheckBox * slider_enabled,int i)
{
    _device_list_wid->item(i)->setHidden(true);
    if(slider_enabled->isChecked()){
        _device_list_wid->item(i)->setHidden(false);
    }
}

void EcGuiSlider::delete_sliders()
{
    for(auto [tab_name,vertical_layout]: _sliders_window_map){
        delete_items(vertical_layout->layout());
    }
    
    _devicecontrol->clear();
    _sliders_window_map.clear();
    _slider_map.motor_sw_map.clear();
    _slider_map.valve_sw_map.clear();
    _slider_map.pump_sw_map.clear();    
    _device_list_wid->clear();
}

void EcGuiSlider::reset_sliders()
{
    for (auto& [slave_id, slider_wid]:_slider_map.motor_sw_map){
        slider_wid->align_spinbox(1); // pos_ref=actual motor position
        slider_wid->align_spinbox(2,0.0); // vel_ref=0.0
        slider_wid->align_spinbox(3,0.0); // tor_ref=0.0
        slider_wid->align_spinbox(11,0.0); // aux=0.0
    }
    for (auto& [slave_id, slider_wid]:_slider_map.valve_sw_map){
        slider_wid->align_all_spinbox(0.0);
    }
    for (auto& [slave_id, slider_wid]:_slider_map.pump_sw_map){
        slider_wid->align_spinbox(0); // pressure_ref=actual pressure
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
