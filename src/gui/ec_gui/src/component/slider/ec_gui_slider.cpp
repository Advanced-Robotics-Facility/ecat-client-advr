﻿#include "ec_gui_slider.h"

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

        std::vector<double> gains_pos={220.0,0.0,10.0};
        std::vector<std::string> pid_string={"P","I","D"};

        auto wid_p = new SliderWidget(jname,joint_info_s.actual_pos,QString::number(joint_info_s.min_pos),QString::number(joint_info_s.max_pos),"[rad]",0x3B,pid_string,gains_pos,this);
        wid_p->setMaximumHeight(300);
        wid_p->disable_slider();
        
        _slider_map.position_sw_map[slave_id]=wid_p;

        std::vector<double> gains_vel={20.0,0.0,0.0};

        auto wid_v = new SliderWidget(jname,0.0,QString::number(-joint_info_s.max_vel),QString::number(joint_info_s.max_vel),"[rad/s]",0x71,pid_string,gains_vel,this);
        wid_v->setMaximumHeight(300);
        wid_v->disable_slider();
        
        _slider_map.velocity_sw_map[slave_id]=wid_v;

        gains_pos.clear();
        gains_pos={500.0,0.0,10.0};
        auto wid_p_t = new SliderWidget(jname,joint_info_s.actual_pos,QString::number(joint_info_s.min_pos),QString::number(joint_info_s.max_pos),"[rad]",0xD4,pid_string,gains_pos,this);
        wid_p_t->setMaximumHeight(300);
        wid_p_t->disable_slider();
        
        
        auto wid_p_t_calib=wid_p_t->get_wid_calibration();
        wid_p_t_calib->hide_slider_calib(1);
        
        _slider_map.position_t_sw_map[slave_id]=wid_p_t;

        std::vector<double> gains_tor={1.0, 0.7, 0.007};
        pid_string.clear();
        pid_string={"Tau_p","Tau_fc","Tau_d"};

        auto wid_t = new SliderWidget("",0.0,QString::number(-joint_info_s.max_torq/100),QString::number(joint_info_s.max_torq/100),"[Nm]",0xD4,pid_string,gains_tor,this);
        wid_t->setMaximumHeight(300);
        wid_t->disable_slider();
        wid_t->hide_joint_enabled();
//         auto wid_t_calib=wid_t->get_wid_calibration();
//         for(int i=0 ; i < 3 ; i++)
//         {
//             wid_t_calib->disable_slider_calib(i);
//         }

        _slider_map.torque_sw_map[slave_id]=wid_t;

        pid_string={"P","I","D"};
        std::vector<double> gains_curr={0.0,0.0,0.0};

        auto wid_cc = new SliderWidget(jname,0.0,QString::number(-2.0),QString::number(2.0),"[A]",0xCC,pid_string,gains_curr,this);
        wid_cc->setMaximumHeight(300);
        wid_cc->disable_slider();
        
        _slider_map.velocity_sw_map[slave_id]=wid_v;

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
