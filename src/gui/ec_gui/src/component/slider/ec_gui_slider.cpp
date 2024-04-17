#include "ec_gui_slider.h"

EcGuiSlider::EcGuiSlider(QWidget *parent) :
    QWidget(parent)
{
    // find position, velocity, torque and current tab for adding the sliders for every slave.
    _sliders_poslayout  = parent->findChild<QVBoxLayout *>("positionSliders");
    _sliders_vellayout  = parent->findChild<QVBoxLayout *>("velocitySliders");
    _sliders_torqlayout = parent->findChild<QVBoxLayout *>("torqueSliders");
    _sliders_currlayout = parent->findChild<QVBoxLayout *>("currentSliders");
}

void EcGuiSlider::create_sliders(std::map<int ,joint_info_t > joint_info_map)
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

        auto wid_calib= new SliderWidgetCalib("",gains,pid_string);

        auto wid_v = new SliderWidget(jname,0.0,QString::number(-joint_info_s.max_vel),QString::number(joint_info_s.max_vel),"[rad/s]",pid_string,gains,this);
        _slider_map.velocity_sw_map[slave_id]=wid_v;

        std::vector<double> gains_pt={0,0};
        std::vector<std::string> pid_string_pt={"gain_0","gain_1"};
        auto wid_p_t = new SliderWidget(jname,joint_info_s.actual_pos,QString::number(joint_info_s.min_pos),QString::number(joint_info_s.max_pos),"[rad]",pid_string_pt,gains_pt,this);
        _slider_map.position_t_sw_map[slave_id]=wid_p_t;

        std::vector<double> gains_t={0,0,0};
        std::vector<std::string> pid_string_t={"gain_2","gain_3","gain_4"};
        auto wid_t = new SliderWidget("",0.0,QString::number(-joint_info_s.max_torq/100),QString::number(joint_info_s.max_torq/100),"[Nm]",pid_string_t,gains_t,this);
        _slider_map.torque_sw_map[slave_id]=wid_t;


        auto wid_cc = new SliderWidget(jname,0.0,QString::number(-2.0),QString::number(2.0),"[A]",pid_string,gains,this);
        _slider_map.current_sw_map[slave_id]=wid_cc;

        _sliders_poslayout->addWidget(wid_p,0, Qt::AlignTop);
        _sliders_vellayout->addWidget(wid_v,0, Qt::AlignTop);
        _sliders_torqlayout->addWidget(wid_p_t,0, Qt::AlignTop);
        _sliders_torqlayout->addWidget(wid_t,0, Qt::AlignTop);
        _sliders_currlayout->addWidget(wid_cc,0, Qt::AlignTop);
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
    
    _joint_info_map.clear();
    
}

void EcGuiSlider::reset_sliders()
{
    for (auto& [slave_id, slider_wid]:_slider_map.position_sw_map)
    {
        slider_wid->align_spinbox();
        _slider_map.position_t_sw_map[slave_id]->align_spinbox();
        _slider_map.velocity_sw_map[slave_id]->align_spinbox(0.0);
        _slider_map.torque_sw_map[slave_id]->align_spinbox(0.0);
        _slider_map.current_sw_map[slave_id]->align_spinbox(0.0);
    }
}
void EcGuiSlider::enable_sliders()
{
    for (auto& [slave_id, slider_wid]:_slider_map.actual_sw_map_selected)
    {
        if(slider_wid->is_joint_enabled())
        {
            _slider_map.position_sw_map[slave_id]->enable_slider();
            _slider_map.position_t_sw_map[slave_id]->enable_slider();
            _slider_map.velocity_sw_map[slave_id]->enable_slider();
            _slider_map.torque_sw_map[slave_id]->enable_slider();
            _slider_map.current_sw_map[slave_id]->enable_slider();
        }
    }
}
void EcGuiSlider::disable_sliders()
{
    for (auto& [slave_id, slider_wid]:_slider_map.position_sw_map)
    {
        slider_wid->disable_slider();
        _slider_map.position_t_sw_map[slave_id]->disable_slider();
        _slider_map.velocity_sw_map[slave_id]->disable_slider();
        _slider_map.torque_sw_map[slave_id]->disable_slider();
        _slider_map.current_sw_map[slave_id]->disable_slider();
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
