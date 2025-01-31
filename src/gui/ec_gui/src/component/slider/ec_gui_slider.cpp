#include "ec_gui_slider.h"

EcGuiSlider::EcGuiSlider(QWidget *parent) :
    QWidget(parent)
{
    _devicecontrol=parent->findChild<QTabWidget *>("deviceControl");
    _device_list_wid = parent->findChild<QListWidget *>("devicelistWidget");
}

QVBoxLayout* EcGuiSlider::retrieve_slider_layout(const std::string &tab_name,
                                                 const QStringList &control_mode,
                                                 const std::vector<int> control_mode_hex)
{
    if(_sliders_window_map.count(tab_name)==0){
        int tab_actual_index=_devicecontrol->count();
        _sliders_window_map[tab_name]=new SliderWindow(control_mode,control_mode_hex,this);
        auto ctrl_mode_combo=_sliders_window_map[tab_name]->get_control_mode();
        connect(ctrl_mode_combo, SIGNAL(currentIndexChanged(int)),this,SLOT(control_mode_change()));
        _devicecontrol->insertTab(tab_actual_index,_sliders_window_map[tab_name],QString::fromStdString(tab_name));
    }
    
    set_control_mode(tab_name);
    
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
            std::vector<int> control_mode_hex= {0x3B,0x71,0xD4,0xCC,0xDD,0x00};
            auto motor_layout= retrieve_slider_layout("Motors",control_mode,control_mode_hex);
            motor_layout->addWidget(wid_motor,0, Qt::AlignTop);

            _device_list_wid->addItem(motor_name);
            auto slider_enabled=wid_motor->get_slider_enabled();
            connect(slider_enabled, &QCheckBox::toggled,
                    std::bind(&EcGuiSlider::on_checkbox_clicked, this, slider_enabled,device_list_index));
            device_list_index++;
        }
        else if(device_type==iit::ecat::HYQ_KNEE){
            std::string valve_name_s="valve_"+std::to_string(device_id);
            QString valve_name = QString::fromStdString(valve_name_s);
            auto wid_valve=new SliderWidget(valve_name,valve_info,this);
            _slider_map.valve_sw_map[device_id]=wid_valve;

            QStringList control_mode= {"Position", "Force","Current", "Idle"};
            std::vector<int> control_mode_hex= {0x3B,0xD4,0xDD,0x00};
            auto valve_layout= retrieve_slider_layout("Valves",control_mode,control_mode_hex);
            valve_layout->addWidget(wid_valve,0, Qt::AlignTop);

            _device_list_wid->addItem(valve_name);
            auto slider_enabled=wid_valve->get_slider_enabled();
            connect(slider_enabled, &QCheckBox::toggled,
                    std::bind(&EcGuiSlider::on_checkbox_clicked, this, slider_enabled,device_list_index));
            device_list_index++;
        }
        else if(device_type==iit::ecat::HYQ_HPU){
            std::string pump_name_s="pump_"+std::to_string(device_id);
            QString pump_name = QString::fromStdString(pump_name_s);
            auto wid_pump=new SliderWidget(pump_name,pump_info,this);
            _slider_map.pump_sw_map[device_id]=wid_pump;

            QStringList control_mode= {"Pressure", "Idle"};
            std::vector<int> control_mode_hex= {0x3B,0x00};

            auto pump_layout= retrieve_slider_layout("Pumps",control_mode,control_mode_hex);
            pump_layout->addWidget(wid_pump,0, Qt::AlignTop);

            _device_list_wid->addItem(pump_name);
            auto slider_enabled=wid_pump->get_slider_enabled();
            connect(slider_enabled, &QCheckBox::toggled,
                    std::bind(&EcGuiSlider::on_checkbox_clicked, this, slider_enabled,device_list_index));
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

void EcGuiSlider::set_sliders_info(double st,bool stopping_wave)
{
    for (auto& [slave_id, slider_wid]:_slider_map.motor_sw_map){
         slider_wid->set_wave_info(st,stopping_wave);
    }

    for (auto& [slave_id, slider_wid]:_slider_map.valve_sw_map){
         slider_wid->set_wave_info(st,stopping_wave);
    }
    for (auto& [slave_id, slider_wid]:_slider_map.pump_sw_map){
         slider_wid->set_wave_info(st,stopping_wave);
    }
}

void EcGuiSlider::check_sliders()
{
    int curr_tab_index=_devicecontrol->currentIndex();
    QString curr_tab_name= _devicecontrol->tabText(curr_tab_index);

    if(curr_tab_name=="Motors"){
        for (auto& [slave_id, slider_wid]:_slider_map.motor_sw_map){
            slider_wid->check_slider_enabled();
        }
    }
    else if(curr_tab_name=="Valves"){
        for (auto& [slave_id, slider_wid]:_slider_map.valve_sw_map){
            slider_wid->check_slider_enabled();
        }
    }
    else if(curr_tab_name=="Pumps"){
        for (auto& [slave_id, slider_wid]:_slider_map.pump_sw_map){
            slider_wid->check_slider_enabled();
        }
    }
}

void EcGuiSlider::uncheck_sliders()
{
    int curr_tab_index=_devicecontrol->currentIndex();
    QString curr_tab_name= _devicecontrol->tabText(curr_tab_index);

    if(curr_tab_name=="Motors"){
        for (auto& [slave_id, slider_wid]:_slider_map.motor_sw_map){
            slider_wid->uncheck_slider_enabled();
        }
    }
    else if(curr_tab_name=="Valves"){
        for (auto& [slave_id, slider_wid]:_slider_map.valve_sw_map){
            slider_wid->uncheck_slider_enabled();
        }
    }
    else if(curr_tab_name=="Pumps"){
        for (auto& [slave_id, slider_wid]:_slider_map.pump_sw_map){
            slider_wid->uncheck_slider_enabled();
        }
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

void EcGuiSlider::control_mode_change()
{
    int curr_tab_index=_devicecontrol->currentIndex();
    QString curr_tab_name= _devicecontrol->tabText(curr_tab_index);
    std::string tab_name=curr_tab_name.toStdString();
    set_control_mode(tab_name);
}


void EcGuiSlider::set_control_mode(const std::string &tab_name)
{
    if(tab_name=="Motors"){
        int ctrl_mode=_sliders_window_map[tab_name]->read_control_mode();
        for (auto& [slave_id, slider_wid]:_slider_map.motor_sw_map){
            slider_wid->align_spinbox(0,ctrl_mode);
        }
    }
}

int EcGuiSlider::get_control_mode(std::string tab_name)
{
    int ctrl_mode=0x00;
    if(_sliders_window_map.count(tab_name)>0){
        ctrl_mode=_sliders_window_map[tab_name]->read_control_mode();
    }
    return ctrl_mode;
}

void EcGuiSlider::enable_control_mode(const std::string& tab_name)
{
    if(tab_name!=""){
        if(_sliders_window_map.count(tab_name)>0){
            _sliders_window_map[tab_name]->enable_control_mode();
        }
    }
    else{
        for(auto&[tab_name,tab_window]:_sliders_window_map){
            tab_window->enable_control_mode();
        }
    }
}
void EcGuiSlider::disable_control_mode(const std::string& tab_name)
{
    if(tab_name!=""){
        if(_sliders_window_map.count(tab_name)>0){
            _sliders_window_map[tab_name]->disable_control_mode();
        }
    }
    else{
        for(auto&[tab_name,tab_window]:_sliders_window_map){
            tab_window->disable_control_mode();
        }
    }
}

EcGuiSlider::~EcGuiSlider()
{

}
