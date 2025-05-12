#include "ec_gui_slider.h"

static std::vector<std::string> motor_std_limits={"-3.14","3.14","6.28","10.0","30.0"};
static SliderWidget::slider_info_s motor_info={
    MotorPdoTx::name,
    {"[uless]","[rad]","[rad/s]","[Nm]","[arbu]", "[arbu]","[arbu]","[arbu]","[arbu]","[uless]","[uless]","[arbu]"},
    {0,2,2,2,4,4,4,4,4,0,0,2},
    {"0",motor_std_limits[0],std::string("-")+motor_std_limits[2],std::string("-")+motor_std_limits[3],"0","0","0","0","0","0","0","-10000"},
    {"500",motor_std_limits[1],motor_std_limits[2],motor_std_limits[3],"10000","10000","10000","10000","10000","2","65535","10000"},
    {0,1,1,1,2,2,2,2,2,3,3,1}
};
static int gain_start_pos=4;

static SliderWidget::slider_info_s valve_info= {
    ValvePdoTx::name,
    {"[mA]", "[um]", "[N]","[arbu]", "[arbu]","[arbu]","[arbu]","[arbu]","[uless]","[uless]","[uless]","[arbu]"},
    {2,2,2,4,4,4,4,4,0,0,0,2},
    {"-25.0","-2000.0","-100","-100.0","-100","-100","-100","-100","0","0","0","-10000"},
    {"25.0","2000.0","100","100.0","100","100","100","100","1","1","1","10000"},
    {1,1,1,2,2,2,2,2,3,3,3,1}
};
  
static SliderWidget::slider_info_s pump_info= {
    PumpPdoTx::name,
    {"[bar]", "[arbu]", "[arbu]","[arbu]", "[arbu]","[uless]","[uless]","[uless]","[uless]","[arbu]"},
    {2,4,4,4,4,0,0,0,0,2},
    {"-150","-2000.0","-2000.0","-2000.0","-2000.0","0","0","0","0","-10000"},
    {"150","2000.0","2000.0","2000.0","2000.0","65536","65536","65536","65536","10000"},
    {1,2,2,2,2,3,3,3,3,1}
};
  

EcGuiSlider::EcGuiSlider(QWidget *parent) :
    QWidget(parent)
{
    _devicecontrol=parent->findChild<QTabWidget *>("deviceControl");
    _device_list_wid = parent->findChild<QListWidget *>("devicelistWidget");
    if(motor_info.slider_name.size()==static_cast<size_t>(MotorPdoTx::pdo_size)){
        std::vector<std::string> new_fields={"curr_ref","[A]","2",std::string("-")+motor_std_limits[4],motor_std_limits[4],"1"};
        adjust_slave_info(motor_info,4,new_fields);
        gain_start_pos=5;
    }
}

template<typename T>
std::string to_string(T value)
{
    int value_int = static_cast<int>(value*100+0.5);
    T round = static_cast<T>(value_int)/100; 

    std::stringstream string_stream;
    string_stream << std::fixed << std::setprecision(2) << value;

    return string_stream.str();
}

void EcGuiSlider::adjust_slave_info(SliderWidget::slider_info_s &slide_info,
                                    int index,
                                    std::vector<std::string> new_fields)
{   
    if(new_fields.size()!=6){
        return;
    }
    slide_info.slider_name.insert(slide_info.slider_name.begin()+index,new_fields[0]);
    slide_info.slider_unit.insert(slide_info.slider_unit.begin()+index,new_fields[1]);
    slide_info.slider_decimal.insert(slide_info.slider_decimal.begin()+index,std::atoi(new_fields[2].c_str()));
    slide_info.slider_min.insert(slide_info.slider_min.begin()+index,new_fields[3]);
    slide_info.slider_max.insert(slide_info.slider_max.begin()+index,new_fields[4]);
    slide_info.slider_property.insert(slide_info.slider_property.begin()+index,std::atoi(new_fields[5].c_str()));
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
    
    return _sliders_window_map[tab_name]->get_layout();
}

void EcGuiSlider::create_sliders(SSI device_info,device_ctrl_t device_ctrl)
{
    
    delete_sliders();
    _device_ctrl=device_ctrl;
    int device_list_index=0;
    for ( auto &[device_id, device_type, device_pos] : device_info ) {
        if(ec_motors.count(device_type)>0){
            std::string motor_name_s="motor_"+std::to_string(device_id);
            QString motor_name = QString::fromStdString(motor_name_s);
            if(_device_ctrl.device_limits.count(device_id)>0){
                auto limits = _device_ctrl.device_limits[device_id];
                int start_device_info=1;
                if(limits[0]!=FLT_MIN && limits[1]!=FLT_MAX){
                    motor_info.slider_min[start_device_info] = to_string(limits[0]);
                    motor_info.slider_max[start_device_info] = to_string(limits[1]);
                }
                else{
                    motor_info.slider_min[start_device_info] = motor_std_limits[0];
                    motor_info.slider_max[start_device_info] = motor_std_limits[1];
                }
                
                int limits_size = static_cast<int>(limits.size());
                start_device_info++;
                for(int i=2;i<limits_size;i++){
                    if(limits[i]>0.0){
                        motor_info.slider_min[start_device_info] = to_string(-limits[i]);
                        motor_info.slider_max[start_device_info] = to_string(limits[i]);
                    }
                    else{
                        motor_info.slider_min[start_device_info] = std::string("-")+motor_std_limits[i];
                        motor_info.slider_max[start_device_info] = motor_std_limits[i];
                    }
                    start_device_info++;
                }   
            }
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

            QStringList control_mode= {"Pressure","Velocity","PWM","Idle"};
            std::vector<int> control_mode_hex= {0xD4,0x71,0x39,0x00};

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

    set_control_mode("Motors");
    set_control_mode("Valves");
    set_control_mode("Pumps");
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

    _old_ctrl_mode=_ctrl_mode=0x00;
    
    _devicecontrol->clear();
    _sliders_window_map.clear();
    _slider_map.motor_sw_map.clear();
    _slider_map.valve_sw_map.clear();
    _slider_map.pump_sw_map.clear();    
    _device_list_wid->clear();
    _device_ctrl.device_gains.clear();
    _device_ctrl.device_limits.clear();
}

void EcGuiSlider::reset_sliders()
{
    for (auto& [slave_id, slider_wid]:_slider_map.motor_sw_map){
        slider_wid->align_spinbox(1); // pos_ref=actual motor position
        slider_wid->align_spinbox(2,0.0); // vel_ref=0.0
        slider_wid->align_spinbox(3,0.0); // tor_ref=0.0
        slider_wid->align_spinbox(4,0.0); // curr_ref=0.0
        slider_wid->align_spinbox(12,0.0); // aux=0.0
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
        _ctrl_mode=_sliders_window_map[tab_name]->read_control_mode();
        for (auto& [slave_id, slider_wid]:_slider_map.motor_sw_map){
            slider_wid->align_spinbox(0,_ctrl_mode);
            if(_device_ctrl.device_gains.count(slave_id)>0){
                int gain_start_index=gain_start_pos; 
                // save gains of the old ctrl mode.
                if(_device_ctrl.device_gains[slave_id].count(_old_ctrl_mode)>0){
                    for(auto & gain_value: _device_ctrl.device_gains[slave_id][_old_ctrl_mode]){
                        gain_value=slider_wid->get_spinbox_value(gain_start_index);
                        gain_start_index++;
                    } 
                }
                gain_start_index=gain_start_pos;
                // load gains of the actual ctrl mode.
                if(_device_ctrl.device_gains[slave_id].count(_ctrl_mode)>0){
                    for(const auto & gain_value: _device_ctrl.device_gains[slave_id][_ctrl_mode]){
                        slider_wid->align_spinbox(gain_start_index,gain_value);
                        gain_start_index++;
                    }
                }
            }
        }
        _old_ctrl_mode=_ctrl_mode;
    }
}

int EcGuiSlider::get_control_mode(std::string tab_name)
{
    if(_sliders_window_map.count(tab_name)>0){
        _ctrl_mode=_sliders_window_map[tab_name]->read_control_mode();
    }
    return _ctrl_mode;
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
