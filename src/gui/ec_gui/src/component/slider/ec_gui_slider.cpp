#include "ec_gui_slider.h"

EcGuiSlider::EcGuiSlider(QWidget *parent) :
    QWidget(parent)
{
    _sliders_motorlayout  = parent->findChild<QVBoxLayout *>("motorSliders");
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
    std::vector<std::string> pdo_string=MotorPdoTx::name;
    pdo_string.erase(pdo_string.begin());

    std::vector<std::string> pdo_unit=MotorPdoTx::unit;
    pdo_unit.erase(pdo_unit.begin());

    for (auto& [slave_id, joint_info_s]:_joint_info_map)
    {
        QString jname = QString::fromStdString(joint_info_s.joint_name);
        auto wid_motor = new SliderWidget(jname,QString::number(joint_info_s.min_pos),QString::number(joint_info_s.max_pos),pdo_unit,pdo_string,this);
        _slider_map.motor_sw_map[slave_id]=wid_motor;

        _sliders_motorlayout->addWidget(wid_motor,0, Qt::AlignTop);
    }

    for (auto& [slave_id, max_current]:valve_info_map){

        std::string valve_name_s="valve_"+std::to_string(slave_id);
        QString valve_name = QString::fromStdString(valve_name_s);

        auto wid_valve = new SliderWidget(valve_name,QString::number(-max_current),QString::number(max_current),ValvePdoTx::unit,ValvePdoTx::name,this);
        _slider_map.valve_sw_map[slave_id]=wid_valve;

        _sliders_valvelayout->addWidget(wid_valve,0, Qt::AlignTop);
    }
    
    for (auto& [slave_id, max_pressure]:pump_info_map){

        std::string pump_name_s="pump_"+std::to_string(slave_id);
        QString pump_name = QString::fromStdString(pump_name_s);

        auto wid_pump = new SliderWidget(pump_name,QString::number(0.0),QString::number(max_pressure),PumpPdoTx::unit,PumpPdoTx::name,this);
        _slider_map.pump_sw_map[slave_id]=wid_pump;

        _sliders_pumplayout->addWidget(wid_pump,0, Qt::AlignTop);
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

    _joint_info_map.clear();
    
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
