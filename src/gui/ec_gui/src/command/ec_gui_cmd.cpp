#include "ec_gui_cmd.h"

using namespace std::chrono;
EcGuiCmd::EcGuiCmd(EcIface::Ptr client,
                   std::map<int, SliderWidget*> position_sw_map,
                   std::map<int, SliderWidget*> velocity_sw_map,
                   std::map<int, SliderWidget*> position_t_sw_map,
                   std::map<int, SliderWidget*> torque_sw_map,
                   QWidget *parent) :
    QWidget(parent),
    _client(client),
    _position_sw_map(position_sw_map),
    _velocity_sw_map(velocity_sw_map),
    _position_t_sw_map(position_t_sw_map),
    _torque_sw_map(torque_sw_map)
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


    _motor_start_req=_send_ref=false;
    _value=0;
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
        _ctrl_cmd_type=ClientCmdType::START;
        _tabcontrol->setEnabled(true);
        _mode_type_combobox->setEnabled(true);
        readModeType();
        _notallbtn->setEnabled(!_motor_start_req); // motors already started.
        _allbtn->setEnabled(!_motor_start_req); // motors already started.
    }
    else if(getFieldType() == "Stop motors")
    {
        _ctrl_cmd_type=ClientCmdType::STOP;
        _mode_type_combobox->setEnabled(false);
        _tabcontrol->setEnabled(false);
        _notallbtn->setEnabled(false);
        _allbtn->setEnabled(false);
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
    for (auto& [slave_id, slider_wid]:_sw_map_selected)
    {
        auto joint_calib_selected=slider_wid->get_wid_calibration();
        for(int calib_index=0; calib_index < joint_calib_selected->get_slider_numb(); calib_index++)
        {   
            if(getModeType() == "Idle")
            {
                joint_calib_selected->disable_slider_calib(calib_index);
                if(_value == 0xD4)
                {
                    _torque_sw_map[slave_id]->get_wid_calibration()->disable_slider_calib(calib_index);
                }
            }
            else
            {
                joint_calib_selected->enable_slider_calib(calib_index);
                if(_value == 0xD4)
                {
                    _torque_sw_map[slave_id]->get_wid_calibration()->enable_slider_calib(calib_index);
                }
            }
        }
    }
}
void EcGuiCmd::readModeType()
{
    if(getModeType() == "Position")
    {
        _tabcontrol->setTabEnabled(0,true);
        _tabcontrol->setTabEnabled(1,false);
        _tabcontrol->setTabEnabled(2,false);

        _tabcontrol->setCurrentIndex(0);
        _value=0x3B;
        _sw_map_selected.clear();
        _sw_map_selected=_position_sw_map;
    }
    else if(getModeType() == "Velocity")
    {
        _tabcontrol->setTabEnabled(0,false);
        _tabcontrol->setTabEnabled(1,true);
        _tabcontrol->setTabEnabled(2,false);

        _tabcontrol->setCurrentIndex(1);
        _value=0x71;
        _sw_map_selected.clear();
        _sw_map_selected=_velocity_sw_map;
    }
    else if(getModeType() == "Impedance")
    {
        _tabcontrol->setTabEnabled(0,false);
        _tabcontrol->setTabEnabled(1,false);
        _tabcontrol->setTabEnabled(2,true);
        _tabcontrol->setCurrentIndex(2);
        _value=0xD4;
        _sw_map_selected.clear();
        _sw_map_selected=_position_t_sw_map;
    }
    else if(getModeType() == "Idle")
    {
    }
    else
    {
        throw std::runtime_error("Error: Found not valid starting mode");
    }
    
    enable_disable_pid();  
}

void EcGuiCmd::onNotAllCmdReleased()
{
    /* Uncheck all checkboxes of Joint WID */
    for (auto& [slave_id, slider_wid]:_sw_map_selected)
    {
        slider_wid->uncheck_joint_enabled();
    }

}

void EcGuiCmd::onAllCmdReleased()
{
    /* Check all checkboxes of Slider WID */
    for (auto& [slave_id, slider_wid]:_sw_map_selected)
    {
        slider_wid->check_joint_enabled();
    }

}

void EcGuiCmd::launch_cmd_message(QString message)
{
   QMessageBox msgBox;
   msgBox.setText(message);
   msgBox.exec();
}

void EcGuiCmd::onLED_ON_OFF_Released()
{
    PAC led_cmds = {};
    QPushButton* led_on_off_btn = qobject_cast<QPushButton*>(sender());
    for (auto& [slave_id, slider_wid]:_sw_map_selected)
    {
        auto actual_led_on_off_btn = slider_wid->get_led_on_off_btn();
        bool led_on_off_cmd_result=false;
        if(actual_led_on_off_btn == led_on_off_btn)
        {
            QString led_text,led_style;
            if(led_on_off_btn->text()=="LED OFF")
            {
                led_cmds.push_back(std::make_tuple(slave_id,to_underlying(PdoAuxCmdType::LED_ON)));     
                if(_client->pdo_aux_cmd(led_cmds))
                {
                    std::this_thread::sleep_for(100ms);
                    if(_client->pdo_aux_cmd_sts(led_cmds))
                    {
                        led_on_off_cmd_result=true;
                        led_text="LED ON";
                        led_style="background: green; color: #00FF00";
                    }
                }
            }
            else
            {
                led_cmds.push_back(std::make_tuple(slave_id,to_underlying(PdoAuxCmdType::LED_OFF)));
                if(_client->pdo_aux_cmd(led_cmds))
                {
                    std::this_thread::sleep_for(100ms);
                    if(_client->pdo_aux_cmd_sts(led_cmds))
                    {
                        led_on_off_cmd_result=true;
                        led_text="LED OFF";
                        led_style="background: red; color: #00FF00";
                    }
                }
            }
            
            if(!led_on_off_cmd_result)
            {
                QMessageBox msgBox;
                msgBox.setText("Cannot perform LED switching on/off command on the slaves requested");
                msgBox.exec();
            }
            else
            {
                
                auto pos_led_on_off_btn= _position_sw_map[slave_id]->get_led_on_off_btn();
                pos_led_on_off_btn->setText(led_text);
                pos_led_on_off_btn->setStyleSheet(led_style);
                auto vel_led_on_off_btn= _velocity_sw_map[slave_id]->get_led_on_off_btn();
                vel_led_on_off_btn->setText(led_text);
                vel_led_on_off_btn->setStyleSheet(led_style);
                auto pos_t_led_on_off_btn= _position_t_sw_map[slave_id]->get_led_on_off_btn();
                pos_t_led_on_off_btn->setText(led_text);
                pos_t_led_on_off_btn->setStyleSheet(led_style);
            }
            
            break;
        }
    }

}

void EcGuiCmd::fill_start_stop_cmd()
{
    _motors_start.clear();
    _brake_cmds.clear();
    _motors_selected = false;
    for (auto& [slave_id, slider_wid]:_sw_map_selected)
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
                    if(_value==0xD4)
                    {
                        _gains.erase(_gains.begin()+1);
                        joint_calib_selected=_torque_sw_map[slave_id]->get_wid_calibration();
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
                    _motors_start.push_back(std::make_tuple(slave_id,_value,_gains));
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
    // @NOTE to be tested.
    for (auto& [slave_id, slider_wid]:_position_sw_map)
    {
        slider_wid->align_spinbox();
        _position_t_sw_map[slave_id]->align_spinbox();
        _velocity_sw_map[slave_id]->align_spinbox(0.0);
        _torque_sw_map[slave_id]->align_spinbox(0.0);
    }
    
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
        _motor_start_req=false;
        _send_ref=false;
        
        fill_start_stop_cmd();
        
        if(!_motors_selected)
        {
            cmd_message="No Slave selected, please select at least one slave";
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

                for (auto& [slave_id, slider_wid]:_sw_map_selected)
                {
                    slider_wid->disable_joint_enabled();
                }
            }
        }
    }
    
    launch_cmd_message(cmd_message);
}

EcGuiCmd::~EcGuiCmd()
{

}
