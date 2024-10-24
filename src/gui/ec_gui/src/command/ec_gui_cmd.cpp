﻿#include "ec_gui_cmd.h"

using namespace std::chrono;
EcGuiCmd::EcGuiCmd(EcGuiSlider::Ptr ec_gui_slider,
                   QWidget *parent) :
    QWidget(parent),
    _ec_gui_slider(ec_gui_slider)
{

    /*  EtherCAT Master commands */
    _fieldtype_combobox = parent->findChild<QComboBox *>("SelectFieldComboBox");

    /* connection of read command function */
    connect(_fieldtype_combobox, SIGNAL(currentIndexChanged(int)),this,
        SLOT(readCommand())
    );

    /*  create mode type to start he motors */
    _mode_type_combobox = parent->findChild<QComboBox *>("ModeType");

    /* connection of read mode type function */
    connect(_mode_type_combobox, SIGNAL(currentIndexChanged(int)),this,
        SLOT(readModeType())
    );

    // find devices
    _devicecontrol=parent->findChild<QTabWidget *>("deviceControl");

    // find position, velocity and torque tab.
    _tabcontrol = parent->findChild<QTabWidget *>("tabControl");

    /* Getting command manager (Apply) */
    _cmd_manager = parent->findChild<QDialogButtonBox *>("CmdManager");
    _applybtn = _cmd_manager->button(QDialogButtonBox::Apply);

    connect(_applybtn, &QPushButton::released,
           this, &EcGuiCmd::onApplyCmdReleased);

    /* Getting command manager (Apply) */
    auto dis_enable_slaves = parent->findChild<QDialogButtonBox *>("DisEnableSlaves");

   _notallbtn = dis_enable_slaves->button(QDialogButtonBox::NoToAll);

    connect(_notallbtn, &QPushButton::released,
            this, &EcGuiCmd::onNotAllCmdReleased);

    _allbtn = dis_enable_slaves->button(QDialogButtonBox::YesToAll);

    connect(_allbtn, &QPushButton::released,
            this, &EcGuiCmd::onAllCmdReleased);
    
        /* Getting command manager (Apply) */
    auto dis_enable_brakes = parent->findChild<QDialogButtonBox *>("DisEnableBrakes");

   _notallbtn_brake = dis_enable_brakes->button(QDialogButtonBox::NoToAll);

    connect(_notallbtn_brake, &QPushButton::released,
            this, &EcGuiCmd::onNotAllBrakeReleased);

    _allbtn_brake = dis_enable_brakes->button(QDialogButtonBox::YesToAll);

    connect(_allbtn_brake, &QPushButton::released,
            this, &EcGuiCmd::onAllBrakeReleased);
    

    _motor_start_req=_send_ref=false;
    _ctrl_cmd=0;
    
    readCommand();
}

void EcGuiCmd::restart_ec_gui_cmd(EcIface::Ptr client)
{
    _client.reset();
    _client=client;
    
    _slider_map = _ec_gui_slider->get_sliders();
    
    _fieldtype_combobox->setCurrentIndex(0);
    _mode_type_combobox->setCurrentIndex(0);
    readCommand();
}

std::string EcGuiCmd::getFieldType() const
{
    return _fieldtype_combobox->currentText().toStdString();
}

void EcGuiCmd::readCommand()
{
    if(getFieldType() == "Start motors")
    {
        _devicecontrol->setTabEnabled(0,true);
        _devicecontrol->setTabEnabled(1,false);
        _devicecontrol->setTabEnabled(2,false);
        _devicecontrol->setCurrentIndex(0);

        _ctrl_cmd_type=ClientCmdType::START;
        _tabcontrol->setEnabled(true);
        _mode_type_combobox->setEnabled(true);
        readModeType();
        _notallbtn->setEnabled(!_motor_start_req); // motors already started.
        _allbtn->setEnabled(!_motor_start_req); // motors already started.
    }
    else if(getFieldType() == "Stop motors")
    {

        _devicecontrol->setTabEnabled(0,true);
        _devicecontrol->setTabEnabled(1,false);
        _devicecontrol->setTabEnabled(2,false);
        _devicecontrol->setCurrentIndex(0);

        _ctrl_cmd_type=ClientCmdType::STOP;
        _mode_type_combobox->setEnabled(false);
        _tabcontrol->setEnabled(false);
        _notallbtn->setEnabled(false);
        _allbtn->setEnabled(false);
    }
    else if(getFieldType() == "Start valves"){
        _devicecontrol->setTabEnabled(0,false);
        _devicecontrol->setTabEnabled(1,true);
        _devicecontrol->setTabEnabled(2,false);
        _devicecontrol->setCurrentIndex(1);
    }
    else if(getFieldType() == "Stop valves"){
        _devicecontrol->setTabEnabled(0,false);
        _devicecontrol->setTabEnabled(1,true);
        _devicecontrol->setTabEnabled(2,false);
        _devicecontrol->setCurrentIndex(1);
    }
    else
    {
        throw std::runtime_error("Error: Found not valid EtherCAT Master commnad");
    }
}

std::string EcGuiCmd::getModeType() const
{
    return _mode_type_combobox->currentText().toStdString();
}

void EcGuiCmd::enable_disable_pid()
{
    for (auto& [slave_id, slider_wid]:_actual_sw_map_selected)
    {
        auto joint_calib_selected=slider_wid->get_wid_calibration();
        for(int calib_index=0; calib_index < joint_calib_selected->get_slider_numb(); calib_index++)
        {   
            if(getModeType() == "Idle")
            {
                joint_calib_selected->disable_slider_calib(calib_index);
                if(_ctrl_cmd == 0xD4)
                {
                    _slider_map.torque_sw_map[slave_id]->get_wid_calibration()->disable_slider_calib(calib_index);
                }
            }
            else
            {
                joint_calib_selected->enable_slider_calib(calib_index);
                if(_ctrl_cmd == 0xD4)
                {
                    _slider_map.torque_sw_map[slave_id]->get_wid_calibration()->enable_slider_calib(calib_index);
                }
            }
        }
    }
}
void EcGuiCmd::readModeType()
{
    if(getModeType() != "Idle")
    {
        _actual_sw_map_selected.clear();
        if(getModeType() == "Position")
        {
            _tabcontrol->setTabEnabled(0,true);
            _tabcontrol->setTabEnabled(1,false);
            _tabcontrol->setTabEnabled(2,false);
            _tabcontrol->setTabEnabled(3,false);
            _tabcontrol->setCurrentIndex(0);
            
            _ctrl_cmd=0x3B;
            _actual_sw_map_selected=_slider_map.position_sw_map;
        }
        else if(getModeType() == "Velocity")
        {
            _tabcontrol->setTabEnabled(0,false);
            _tabcontrol->setTabEnabled(1,true);
            _tabcontrol->setTabEnabled(2,false);
            _tabcontrol->setTabEnabled(3,false);
            _tabcontrol->setCurrentIndex(1);
            
            _ctrl_cmd=0x71;
            _actual_sw_map_selected=_slider_map.velocity_sw_map;
        }
        else if(getModeType() == "Impedance")
        {
            _tabcontrol->setTabEnabled(0,false);
            _tabcontrol->setTabEnabled(1,false);
            _tabcontrol->setTabEnabled(2,true);
            _tabcontrol->setTabEnabled(3,false);
            _tabcontrol->setCurrentIndex(2);
            
            _ctrl_cmd=0xD4;
            _actual_sw_map_selected=_slider_map.position_t_sw_map;
        }
        else if(getModeType() == "Current")
        {
            _tabcontrol->setTabEnabled(0,false);
            _tabcontrol->setTabEnabled(1,false);
            _tabcontrol->setTabEnabled(2,false);
            _tabcontrol->setTabEnabled(3,true);
            _tabcontrol->setCurrentIndex(3);
            
            _ctrl_cmd=0xCC;
            _actual_sw_map_selected=_slider_map.current_sw_map;
        }
        else
        {
            throw std::runtime_error("Error: Found not valid starting mode");
        }
    }
    
    _ec_gui_slider->set_actual_sliders(_actual_sw_map_selected);
    enable_disable_pid();  
}

void EcGuiCmd::onNotAllCmdReleased()
{
    /* Uncheck all checkboxes of Joint WID */
    for (auto& [slave_id, slider_wid]:_actual_sw_map_selected)
    {
        slider_wid->uncheck_joint_enabled();
    }

}

void EcGuiCmd::onAllCmdReleased()
{
    /* Check all checkboxes of Slider WID */
    for (auto& [slave_id, slider_wid]:_actual_sw_map_selected)
    {
        slider_wid->check_joint_enabled();
    }

}

void EcGuiCmd::onNotAllBrakeReleased()
{
    /* Uncheck all checkboxes of Joint WID */
    for (auto& [slave_id, slider_wid]:_actual_sw_map_selected)
    {
        slider_wid->uncheck_joint_braked();
    }

}

void EcGuiCmd::onAllBrakeReleased()
{
    /* Check all checkboxes of Slider WID */
    for (auto& [slave_id, slider_wid]:_actual_sw_map_selected)
    {
        slider_wid->check_joint_braked();
    }

}

void EcGuiCmd::launch_cmd_message(QString message)
{
   QMessageBox msgBox;
   msgBox.setText(message);
   msgBox.exec();
}

void EcGuiCmd::fill_start_stop_cmd()
{
    _motors_start.clear();
    _brake_cmds.clear();
    _motors_selected = false;
    for (auto& [slave_id, slider_wid]:_actual_sw_map_selected)
    {
        if(slider_wid->is_joint_enabled())
        {
            _motors_selected |= true;
            if(_ctrl_cmd_type==ClientCmdType::STOP)
            {
                if(slider_wid->is_joint_braked())
                {
                    _brake_cmds.push_back(std::make_tuple(slave_id,to_underlying(PdoAuxCmdType::BRAKE_ENGAGE)));
                }
            }
            else
            {
                if(getModeType() != "Idle")
                {
                    _gains.clear();
                    auto joint_calib_selected=slider_wid->get_wid_calibration();
                    for(int calib_index=0; calib_index < joint_calib_selected->get_slider_numb(); calib_index++)
                    {
                        _gains.push_back(joint_calib_selected->get_slider_value(calib_index));
                    }
                    if(_ctrl_cmd==0xD4)
                    {
                        _gains.erase(_gains.begin()+1);
                        joint_calib_selected=_slider_map.torque_sw_map[slave_id]->get_wid_calibration();
                        for(int calib_index=0; calib_index < joint_calib_selected->get_slider_numb(); calib_index++)
                        {
                            _gains.push_back(joint_calib_selected->get_slider_value(calib_index));
                        }
                    }
                    else
                    {
                        _gains.push_back(0);
                        _gains.push_back(0);
                    }
                    _motors_start.push_back(std::make_tuple(slave_id,_ctrl_cmd,_gains));
                }
                
                if(slider_wid->is_joint_braked())
                {
                    _brake_cmds.push_back(std::make_tuple(slave_id,to_underlying(PdoAuxCmdType::BRAKE_RELEASE)));
                }
            }
        }
    }
}

bool EcGuiCmd::braking_cmd_req()
{
    //********** USE SDO /***********
    bool braking_cmd_ack=false;
    RD_SDO rd_sdo{};
    for(int i=0;i<_brake_cmds.size();i++)
    {
        int esc_id; 
        int brake_req;
        int brake_req_sdo=0;
        WR_SDO wr_sdo{};
                
        std::tie(esc_id,brake_req) = _brake_cmds[i];

        if(brake_req == to_underlying(PdoAuxCmdType::BRAKE_RELEASE))
        {
            brake_req_sdo=0x00BD;
        }
        else if(brake_req == to_underlying(PdoAuxCmdType::BRAKE_ENGAGE))
        {
            brake_req_sdo=0x00DB;
        }
        
        wr_sdo.push_back(std::make_tuple("ctrl_status_cmd",std::to_string(brake_req)));
        braking_cmd_ack &= _client->set_wr_sdo(esc_id,rd_sdo,wr_sdo);
    }
    
    //********** USE PDO /***********
    //braking_cmd_ack=_client->pdo_aux_cmd(_brake_cmds);
    
    return braking_cmd_ack;
}

void EcGuiCmd::onApplyCmdReleased()
{
    QString cmd_message;
    if((_motor_start_req)&&(_ctrl_cmd_type==ClientCmdType::START))
    {
        cmd_message="Motor(s) already started, please launch STOP EtherCAT command";
    }
    else if((!_motor_start_req)&&(_ctrl_cmd_type==ClientCmdType::STOP))
    {
        cmd_message="No motor was started, please launch START EtherCAT command";
    }
    else
    {
        // @NOTE to be tested.
        _ec_gui_slider->reset_sliders();
        
        _motor_start_req=false;
        _send_ref=false;
        
        fill_start_stop_cmd();
        
        if(!_motors_selected)
        {
            cmd_message="No Motor selected, please select at least one";
        }
        else
        {
            //********** START MOTORS **********//
            if(!_motors_start.empty())
            {
                _motor_start_req=_client->start_motors(_motors_start);
                if(!_motor_start_req)
                {
                    cmd_message="Cannot perform the start command on the motor(s) requested";
                    launch_cmd_message(cmd_message);
                    return;
                }
            }
            
            //********** RELEASE OR ENGAGE BRAKES WITH CHECKS **********//
            if(!_brake_cmds.empty())
            {
                bool braking_cmd_fdb = braking_cmd_req(); // release or engage the brakes
                if(braking_cmd_fdb)
                {
                    std::this_thread::sleep_for(1000ms);
                    if(!_client->pdo_aux_cmd_sts(_brake_cmds)) // release or engage the brakes status
                    {
                        cmd_message="Wrong status of the brakes requested";
                        launch_cmd_message(cmd_message);
                        return;
                    }
                }
                else
                {
                    cmd_message="Cannot perform the release or engage brake command on the motor(s) requested";
                    launch_cmd_message(cmd_message);
                    return;
                }
            }

            cmd_message="All motor(s) requested have performed the command successfully";
            
            if(!_motor_start_req)
            {
                //********** STOP MOTORS **********//
                if(!_client->stop_motors())
                {
                    cmd_message.clear();
                    cmd_message="Cannot perform the stop command on the motor(s) requested";
                }
            }
            else
            {
                _mode_type_combobox->setEnabled(false);
                _send_ref=true;
                _notallbtn->setEnabled(false);
                _allbtn->setEnabled(false);

                for (auto& [slave_id, slider_wid]:_actual_sw_map_selected)
                {
                    slider_wid->disable_joint_enabled();
                }
            }
        }
    }
    
    launch_cmd_message(cmd_message);
}

bool EcGuiCmd::get_cmd_sts(float &ctrl_cmd)
{
    ctrl_cmd = _ctrl_cmd;
    return _send_ref;
}

EcGuiCmd::~EcGuiCmd()
{

}
