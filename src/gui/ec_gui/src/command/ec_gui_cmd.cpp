#include "ec_gui_cmd.h"

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
    
    _device_start_req=_send_ref=false;
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
    if(getFieldType() == "Start devices")
    {
        _ctrl_cmd_type=ClientCmdType::START;
        _tabcontrol->setEnabled(true);
        _mode_type_combobox->setEnabled(true);
        readModeType();
        if(!_device_start_req){
            
            for (auto& [slave_id, slider_wid]:_actual_sw_map_selected){
                    slider_wid->enable_slider_enabled();
            }
                    
            for (auto& [slave_id, slider_wid]:_slider_map.valve_sw_map){
                    slider_wid->enable_slider_enabled();
            }

            for (auto& [slave_id, slider_wid]:_slider_map.pump_sw_map){
                slider_wid->enable_slider_enabled();
            }
        
        }

        _notallbtn->setEnabled(!_device_start_req); // devices already started.
        _allbtn->setEnabled(!_device_start_req); // devices already started.
    
    }
    else if(getFieldType() == "Stop devices")
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
    for (auto& [slave_id, slider_wid]:_actual_sw_map_selected)
    {
        auto joint_calib_selected=slider_wid->get_wid_calibration();
        for(int calib_index=0; calib_index < joint_calib_selected->get_slider_numb(); calib_index++)
        {   
            if(getModeType() == "Idle")
            {
                joint_calib_selected->disable_slider_calib(calib_index);
            }
            else
            {
                joint_calib_selected->enable_slider_calib(calib_index);
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
        slider_wid->uncheck_slider_enabled();
    }

}

void EcGuiCmd::onAllCmdReleased()
{
    /* Check all checkboxes of Slider WID */
    for (auto& [slave_id, slider_wid]:_actual_sw_map_selected)
    {
        slider_wid->check_slider_enabled();
    }

}

void EcGuiCmd::launch_cmd_message(QString message)
{
   QMessageBox msgBox;
   msgBox.setText(message);
   msgBox.exec();
}

void EcGuiCmd::fill_start_stop_motor()
{
    _motors_start.clear();
    _brake_cmds.clear();
    _motors_selected = false;
    for (auto& [slave_id, slider_wid]:_actual_sw_map_selected){
        if(slider_wid->is_slider_enabled()){
            _motors_selected |= true;
            if(_ctrl_cmd_type==ClientCmdType::STOP){
                if(false){
                    _brake_cmds.push_back(std::make_tuple(slave_id,to_underlying(PdoAuxCmdType::BRAKE_ENGAGE)));
                }
            }
            else{
                if(getModeType() != "Idle"){
                    _gains.clear();
                    auto joint_calib_selected=slider_wid->get_wid_calibration();
                    for(int calib_index=0; calib_index < joint_calib_selected->get_slider_numb(); calib_index++){
                        _gains.push_back(joint_calib_selected->get_slider_value(calib_index));
                    }
                    _motors_start.push_back(std::make_tuple(slave_id,_ctrl_cmd,_gains));
                }
                
                if(false){
                    _brake_cmds.push_back(std::make_tuple(slave_id,to_underlying(PdoAuxCmdType::BRAKE_RELEASE)));
                }
            }
        }
    }
}

void EcGuiCmd::fill_start_stop_valve()
{
    _start_stop_valve.clear();
    _valves_selected = false;
    for (auto& [slave_id, slider_wid]:_slider_map.valve_sw_map){
        if(slider_wid->is_slider_enabled()){
            _valves_selected |= true;
            if(_ctrl_cmd_type==ClientCmdType::STOP){
                _start_stop_valve[slave_id]={std::make_tuple("ctrl_status_cmd","90")};
            }
            else{
                _start_stop_valve[slave_id]={std::make_tuple("ctrl_status_cmd","165")};
            }
        }
    }
}

void EcGuiCmd::fill_start_stop_pump()
{
    _pumps_selected = false;
    for (auto& [slave_id, slider_wid]:_slider_map.pump_sw_map){
        if(slider_wid->is_slider_enabled()){
            _pumps_selected |= true;
        }
    }
}
    

bool EcGuiCmd::braking_cmd_req()
{
    //********** USE SDO /***********
    bool braking_cmd_ack=false;
    RD_SDO rd_sdo{};
    for(int i=0;i<_brake_cmds.size();i++){
        int esc_id; 
        int brake_req;
        int brake_req_sdo=0;
        WR_SDO wr_sdo{};
                
        std::tie(esc_id,brake_req) = _brake_cmds[i];

        if(brake_req == to_underlying(PdoAuxCmdType::BRAKE_RELEASE)){
            brake_req_sdo=0x00BD;
        }
        else if(brake_req == to_underlying(PdoAuxCmdType::BRAKE_ENGAGE)){
            brake_req_sdo=0x00DB;
        }
        
        wr_sdo.push_back(std::make_tuple("ctrl_status_cmd",std::to_string(brake_req)));
        braking_cmd_ack &= _client->set_wr_sdo(esc_id,rd_sdo,wr_sdo);
    }
    
    //********** USE PDO /***********
    //braking_cmd_ack=_client->pdo_aux_cmd(_brake_cmds);
    
    return braking_cmd_ack;
}

void EcGuiCmd::onApplyCmdMotors()
{
    if(_motors_selected){
    //********** START MOTORS **********//
        if(!_motors_start.empty()){
            _device_start_req =_client->start_motors(_motors_start);
#ifdef TEST_GUI 
            _device_start_req=true;
#endif 
            if(!_device_start_req){
                _cmd_message="Cannot perform the start command on the motor(s) requested";
                launch_cmd_message(_cmd_message);
                return;
            }
        }
        
        //********** RELEASE OR ENGAGE BRAKES WITH CHECKS **********//
        if(!_brake_cmds.empty()){
            bool braking_cmd_fdb = braking_cmd_req(); // release or engage the brakes
            if(braking_cmd_fdb){
                std::this_thread::sleep_for(1000ms);
                if(!_client->pdo_aux_cmd_sts(_brake_cmds)){ // release or engage the brakes status
                    _cmd_message="Wrong status of the brakes requested";
                    launch_cmd_message(_cmd_message);
                    return;
                }
            }
            else{
                _cmd_message="Cannot perform the release or engage brake command on the motor(s) requested";
                launch_cmd_message(_cmd_message);
                return;
            }
        }

        _cmd_message="All motor(s) requested have performed the command successfully";
        
        if(!_device_start_req){
            //********** STOP MOTORS **********//
            if(!_client->stop_motors()){
                _cmd_message.clear();
                _cmd_message="Cannot perform the stop command on the motor(s) requested";
            }
        }

        launch_cmd_message(_cmd_message);
    }
}

void EcGuiCmd::onApplyCmdValves()
{
    if(_valves_selected){
    //********** START/STOP VALVES **********//
        if(!_start_stop_valve.empty()){
            bool valve_cmd_req = true;
            for (auto& [slave_id, wr_sdo]:_start_stop_valve){
                valve_cmd_req &= _client->set_wr_sdo(slave_id,{},wr_sdo);
            }
#ifdef TEST_GUI 
            valve_cmd_req=true;
#endif 
            if(!valve_cmd_req){
                _cmd_message="Cannot perform the stop command on the valves(s) requested"; 
                if(_ctrl_cmd_type==ClientCmdType::START){
                    _cmd_message="Cannot perform the start command on the valves(s) requested";
                }
            }
            else{
                _cmd_message="All valve(s) requested have performed the command successfully";
                if(_ctrl_cmd_type==ClientCmdType::START){
                    _device_start_req |= true;
                }
            }
            launch_cmd_message(_cmd_message);
        }
    }
}

void EcGuiCmd::onApplyCmdPumps()
{
    if(_pumps_selected){
    //********** START/STOP PUMPS **********//
         if(_ctrl_cmd_type==ClientCmdType::START){
            _device_start_req |= true;
         }
         
        _cmd_message="All pumps(s) requested have performed the command successfully";
        launch_cmd_message(_cmd_message);
    }
}

void EcGuiCmd::onApplyCmdReleased()
{
    _cmd_message.clear();
    if((_device_start_req)&&(_ctrl_cmd_type==ClientCmdType::START)){
        _cmd_message="Device(s) already started, please launch STOP EtherCAT command";
        launch_cmd_message(_cmd_message);
    }
    else if((!_device_start_req)&&(_ctrl_cmd_type==ClientCmdType::STOP)){
        _cmd_message="No device(s)  was started, please launch START EtherCAT command";         
        launch_cmd_message(_cmd_message);
    }
    else{
        // @NOTE to be tested.
        _ec_gui_slider->reset_sliders();
        
        _device_start_req=false;
        _send_ref=false;
        
        fill_start_stop_motor();
        fill_start_stop_valve();
        fill_start_stop_pump();
        
        if(!_motors_selected && 
           !_valves_selected &&
           !_pumps_selected){
            _cmd_message="No device selected, please select at least one";
            launch_cmd_message(_cmd_message);
        }
        else{
            
            onApplyCmdMotors();
            onApplyCmdValves();
            onApplyCmdPumps();

            if(_device_start_req){
                _mode_type_combobox->setEnabled(false);
                _send_ref=true;
                _notallbtn->setEnabled(false);
                _allbtn->setEnabled(false);

                for (auto& [slave_id, slider_wid]:_actual_sw_map_selected){
                    slider_wid->disable_slider_enabled();
                }

                for (auto& [slave_id, slider_wid]:_slider_map.valve_sw_map){
                    slider_wid->disable_slider_enabled();
                }

                for (auto& [slave_id, slider_wid]:_slider_map.pump_sw_map){
                    slider_wid->disable_slider_enabled();
                }
            }
        }
    }
}

bool EcGuiCmd::get_cmd_sts(float &ctrl_cmd)
{
    ctrl_cmd = _ctrl_cmd;
    return _send_ref;
}

EcGuiCmd::~EcGuiCmd()
{

}
