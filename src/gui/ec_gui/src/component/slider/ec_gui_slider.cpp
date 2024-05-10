#include "ec_gui_slider.h"

EcGuiSlider::EcGuiSlider(QWidget *parent) :
    QWidget(parent)
{
    // find position, velocity, torque and current tab for adding the sliders for every slave.
    _sliders_poslayout  = parent->findChild<QVBoxLayout *>("positionSliders");
    _sliders_vellayout  = parent->findChild<QVBoxLayout *>("velocitySliders");
    _sliders_torqlayout = parent->findChild<QVBoxLayout *>("torqueSliders");
    _sliders_currlayout = parent->findChild<QVBoxLayout *>("currentSliders");
    _sliders_valvelayout = parent->findChild<QVBoxLayout *>("valveSliders");
    _sliders_pumplayout = parent->findChild<QVBoxLayout *>("pumpSliders");
}

void EcGuiSlider::create_sliders(std::map<int ,joint_info_t > joint_info_map,
                                 std::map<int ,int > valve_info_map,
                                 std::map<int ,int > pump_info_map)
{
    
    delete_sliders();
    
    _joint_info_map= joint_info_map;
    
    // Create the sliders for every tabs

    for (auto& [slave_id, joint_info_s]:_joint_info_map)
    {

        QString jname = QString::fromStdString(joint_info_s.joint_name);

        std::vector<double> gains={0,0,0,0,0};
        std::vector<std::string> pid_string={"gain_0","gain_1","gain_2","gain_3","gain_4"};

        auto wid_p = new SliderWidget(jname,joint_info_s.actual_pos,QString::number(joint_info_s.min_pos),QString::number(joint_info_s.max_pos),"[rad]",pid_string,gains,this);
        _slider_map.position_sw_map[slave_id]=wid_p;

        auto wid_v = new SliderWidget(jname,0.0,QString::number(-joint_info_s.max_vel),QString::number(joint_info_s.max_vel),"[rad/s]",pid_string,gains,this);
        _slider_map.velocity_sw_map[slave_id]=wid_v;

        auto wid_p_t = new SliderWidget(jname,joint_info_s.actual_pos,QString::number(joint_info_s.min_pos),QString::number(joint_info_s.max_pos),"[rad]",pid_string,gains,this);
        _slider_map.position_t_sw_map[slave_id]=wid_p_t;

        auto wid_t = new SliderWidget("",0.0,QString::number(-joint_info_s.max_torq/100),QString::number(joint_info_s.max_torq/100),"[Nm]",{},{},this);
        _slider_map.torque_sw_map[slave_id]=wid_t;
        wid_t->hide_slider_enabled();


        auto wid_cc = new SliderWidget(jname,0.0,QString::number(-2.0),QString::number(2.0),"[A]",pid_string,gains,this);
        _slider_map.current_sw_map[slave_id]=wid_cc;

        _sliders_poslayout->addWidget(wid_p,0, Qt::AlignTop);
        _sliders_vellayout->addWidget(wid_v,0, Qt::AlignTop);
        _sliders_torqlayout->addWidget(wid_p_t,0, Qt::AlignTop);
        _sliders_torqlayout->addWidget(wid_t,0, Qt::AlignTop);
        _sliders_currlayout->addWidget(wid_cc,0, Qt::AlignTop);
    }

    for (auto& [slave_id, max_current]:valve_info_map){

        std::string valve_name_s="valve_"+std::to_string(slave_id);
        QString valve_name = QString::fromStdString(valve_name_s);

        auto wid_valve = new SliderWidget(valve_name,0.0,QString::number(-max_current),QString::number(max_current),"[mA]",{},{},this);
        wid_valve->remove_calibration();
        _slider_map.valve_sw_map[slave_id]=wid_valve;

        _sliders_valvelayout->addWidget(wid_valve,0, Qt::AlignTop);
    }
    
    for (auto& [slave_id, max_pressure]:pump_info_map){

        std::string pump_name_s="pump_"+std::to_string(slave_id);
        QString pump_name = QString::fromStdString(pump_name_s);

        std::vector<double> pdo_value={0,0,0,0,0,0,0,0};
        std::vector<std::string> pdo_string= {"singlePumpHighLt","singlePumpLowLt", "HPUDemandMode",
                                              "vesc1Mode","vesc2Mode","fan1Spd","fan2Spd","sysStateCmd"};

        auto wid_pump = new SliderWidget(pump_name,0.0,QString::number(0.0),QString::number(max_pressure),"[bar]",pdo_string,pdo_value,this);
        //wid_pump->remove_calibration();
        _slider_map.pump_sw_map[slave_id]=wid_pump;

        _sliders_pumplayout->addWidget(wid_pump,0, Qt::AlignTop);
    }
    
}

void EcGuiSlider::delete_sliders()
{
    _slider_map.actual_sw_map_selected.clear();
    

    delete_items(_sliders_poslayout->layout());
    _slider_map.position_sw_map.clear();
    

    delete_items(_sliders_vellayout->layout());
    _slider_map.velocity_sw_map.clear();
    
    delete_items(_sliders_torqlayout->layout());
    _slider_map.position_t_sw_map.clear();
    _slider_map.torque_sw_map.clear();

    delete_items(_sliders_currlayout->layout());
    _slider_map.current_sw_map.clear();

    delete_items(_sliders_valvelayout->layout());
    _slider_map.valve_sw_map.clear();

    delete_items(_sliders_pumplayout->layout());
    _slider_map.pump_sw_map.clear();

    _joint_info_map.clear();
    
}

void EcGuiSlider::reset_sliders()
{
    for (auto& [slave_id, slider_wid]:_slider_map.actual_sw_map_selected)
    {
        slider_wid->align_spinbox();
        _slider_map.position_t_sw_map[slave_id]->align_spinbox();
        _slider_map.velocity_sw_map[slave_id]->align_spinbox(0.0);
        _slider_map.torque_sw_map[slave_id]->align_spinbox(0.0);
        _slider_map.current_sw_map[slave_id]->align_spinbox(0.0);
    }
    for (auto& [slave_id, slider_wid]:_slider_map.valve_sw_map){
        _slider_map.valve_sw_map[slave_id]->align_spinbox(0.0);
    }
    for (auto& [slave_id, slider_wid]:_slider_map.pump_sw_map){
            _slider_map.pump_sw_map[slave_id]->align_spinbox(0.0);
    }
}
void EcGuiSlider::enable_sliders()
{
    for (auto& [slave_id, slider_wid]:_slider_map.actual_sw_map_selected){
        _slider_map.position_sw_map[slave_id]->enable_slider();
        _slider_map.position_t_sw_map[slave_id]->enable_slider();
        _slider_map.velocity_sw_map[slave_id]->enable_slider();
        _slider_map.torque_sw_map[slave_id]->enable_slider();
        _slider_map.current_sw_map[slave_id]->enable_slider();
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
    for (auto& [slave_id, slider_wid]:_slider_map.actual_sw_map_selected){
        _slider_map.position_sw_map[slave_id]->disable_slider();
        _slider_map.position_t_sw_map[slave_id]->disable_slider();
        _slider_map.velocity_sw_map[slave_id]->disable_slider();
        _slider_map.torque_sw_map[slave_id]->disable_slider();
        _slider_map.current_sw_map[slave_id]->disable_slider();
    }

    for (auto& [slave_id, slider_wid]:_slider_map.valve_sw_map){
         slider_wid->disable_slider();
    }
    for (auto& [slave_id, slider_wid]:_slider_map.pump_sw_map){
         slider_wid->disable_slider();
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

void EcGuiSlider::set_actual_sliders(std::map<int, SliderWidget*> actual_sw_map_selected)
{
    _slider_map.actual_sw_map_selected.clear();
    _slider_map.actual_sw_map_selected = actual_sw_map_selected;
}

EcGuiSlider::~EcGuiSlider()
{

}
