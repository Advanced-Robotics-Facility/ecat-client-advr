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
    
    _device_start_req=_device_controlled=false;
    
    readCommand();
}

void EcGuiCmd::restart_ec_gui_cmd(EcIface::Ptr client)
{
    _client=client;
    
    _slider_map = _ec_gui_slider->get_sliders();
    
    _fieldtype_combobox->setCurrentIndex(0);
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
        if(!_device_controlled){
            _ec_gui_slider->enable_control_mode("");
            for (auto& [slave_id, slider_wid]:_slider_map.motor_sw_map){
                slider_wid->enable_slider_enabled();
            }
                    
            for (auto& [slave_id, slider_wid]:_slider_map.valve_sw_map){
                slider_wid->enable_slider_enabled();
            }

            for (auto& [slave_id, slider_wid]:_slider_map.pump_sw_map){
                slider_wid->enable_slider_enabled();
            }
        
        }

        _notallbtn->setEnabled(!_device_controlled); // devices already started.
        _allbtn->setEnabled(!_device_controlled); // devices already started.
    
    }
    else if(getFieldType() == "Stop devices"){
        _ctrl_cmd_type=ClientCmdType::STOP;
        _ec_gui_slider->disable_control_mode("");
        _notallbtn->setEnabled(false);
        _allbtn->setEnabled(false);
    }
    else{
        throw std::runtime_error("Error: Found not valid EtherCAT Master commnad");
    }
}

void EcGuiCmd::onNotAllCmdReleased()
{
    _ec_gui_slider->uncheck_sliders();
}

void EcGuiCmd::onAllCmdReleased()
{
    _ec_gui_slider->check_sliders();
}

void EcGuiCmd::launch_cmd_message(QString message)
{
   QMessageBox msgBox;
   msgBox.setText(message);
   msgBox.exec();
}

void EcGuiCmd::fill_start_stop_motor()
{
    _brake_cmds.clear();
    _motors_selected = false;
    for (auto& [slave_id, slider_wid]:_slider_map.motor_sw_map){
        if(slider_wid->is_slider_checked()){
            _motors_selected |= true;
            if(_ctrl_cmd_type==ClientCmdType::STOP){
                if(false){
                    _brake_cmds.push_back(std::make_tuple(slave_id,to_underlying(PdoAuxCmdType::BRAKE_ENGAGE)));
                }
            }
            else{
                int ctrl_mode=_ec_gui_slider->get_control_mode("Motors");
                if(ctrl_mode!= 0x00){
                    std::vector<float> gains;
                    gains.push_back(slider_wid->get_spinbox_value(4));
                    gains.push_back(slider_wid->get_spinbox_value(5));
                    gains.push_back(slider_wid->get_spinbox_value(6));
                    gains.push_back(slider_wid->get_spinbox_value(7));
                    gains.push_back(slider_wid->get_spinbox_value(8));
                    _start_devices.push_back(std::make_tuple(slave_id,ctrl_mode,gains));
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
    _valves_selected = false;
    for (auto& [slave_id, slider_wid]:_slider_map.valve_sw_map){
        if(slider_wid->is_slider_checked()){
            _valves_selected |= true;
            if(_ctrl_cmd_type==ClientCmdType::START){
                int ctrl_mode=_ec_gui_slider->get_control_mode("Valves");
                if(ctrl_mode!= 0x00){
                    std::vector<float> gains;
                    gains.push_back(slider_wid->get_spinbox_value(3));
                    gains.push_back(slider_wid->get_spinbox_value(4));
                    gains.push_back(slider_wid->get_spinbox_value(5));
                    gains.push_back(slider_wid->get_spinbox_value(6));
                    gains.push_back(slider_wid->get_spinbox_value(7));
                    _start_devices.push_back(std::make_tuple(slave_id,ctrl_mode,gains));
                }
            }
        }
    }
}

void EcGuiCmd::fill_start_stop_pump()
{
    _pumps_selected = false;
    for (auto& [slave_id, slider_wid]:_slider_map.pump_sw_map){
        if(slider_wid->is_slider_checked()){
            _pumps_selected |= true;
            if(_ctrl_cmd_type==ClientCmdType::START){
                int ctrl_mode=_ec_gui_slider->get_control_mode("Pumps");
                if(ctrl_mode!= 0x00){
                    std::vector<float> gains;
                    gains.push_back(slider_wid->get_spinbox_value(2));
                    gains.push_back(slider_wid->get_spinbox_value(3));
                    gains.push_back(slider_wid->get_spinbox_value(4));
                    gains.push_back(slider_wid->get_spinbox_value(5));
                    gains.push_back(slider_wid->get_spinbox_value(6));
                    _start_devices.push_back(std::make_tuple(slave_id,ctrl_mode,gains));
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
    for(const auto &brake_cmd:_brake_cmds){
        int esc_id; 
        int brake_req;
        int brake_req_sdo=0;
        WR_SDO wr_sdo{};
                
        std::tie(esc_id,brake_req) = brake_cmd;

        if(brake_req == to_underlying(PdoAuxCmdType::BRAKE_RELEASE)){
            brake_req_sdo=0x00BD;
        }
        else if(brake_req == to_underlying(PdoAuxCmdType::BRAKE_ENGAGE)){
            brake_req_sdo=0x00DB;
        }
        
        wr_sdo.push_back(std::make_tuple("ctrl_status_cmd",std::to_string(brake_req_sdo)));
        braking_cmd_ack &= _client->set_wr_sdo(esc_id,rd_sdo,wr_sdo);
    }
    
    //********** USE PDO /***********
    //braking_cmd_ack=_client->pdo_aux_cmd(_brake_cmds);
    
    return braking_cmd_ack;
}

void EcGuiCmd::onApplyCmd()
{
    if(!_start_devices.empty()){
        _device_start_req =_client->start_devices(_start_devices);
#ifdef TEST_GUI
        std::this_thread::sleep_for(100ms);
        _device_start_req=_client->get_client_status().run_loop;
#endif 
        if(!_device_start_req){
            _cmd_message="Cannot perform the start command on the devices(s) requested";
            launch_cmd_message(_cmd_message);
            return;
        }
    }

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

    _cmd_message="All devices(s) requested have performed the command successfully";
    
    if(!_device_start_req){
        bool device_stop_req=_client->stop_devices();
#ifdef TEST_GUI 
        std::this_thread::sleep_for(100ms);
        device_stop_req=_client->get_client_status().run_loop;
#endif 
        if(!device_stop_req){
            _cmd_message.clear();
            _cmd_message="Cannot perform the stop command on the device(s) requested";

        }
        else{
            _device_controlled=false;
        }
    }
    launch_cmd_message(_cmd_message);
}

void EcGuiCmd::onApplyCmdReleased()
{
    
    QMessageBox msgBox;
    if(_client == nullptr){
        msgBox.critical(this,msgBox.windowTitle(),
                        tr("EtherCAT client not setup"
                           ",please scan device button.\n"));
        return;
    }
    else{
        if(!_client->get_client_status().run_loop){
            msgBox.critical(this,msgBox.windowTitle(),
                tr("EtherCAT Client loop is not running state"
                    ",please press scan device button.\n"));
            return;
        }
    }

    _cmd_message.clear();
    if((_device_controlled)&&(_ctrl_cmd_type==ClientCmdType::START)){
        _cmd_message="Device(s) already started, please launch STOP EtherCAT command";
        launch_cmd_message(_cmd_message);
    }
    else if((!_device_controlled)&&(_ctrl_cmd_type==ClientCmdType::STOP)){
        _cmd_message="No device(s)  was started, please launch START EtherCAT command";         
        launch_cmd_message(_cmd_message);
    }
    else{
        // @NOTE to be tested.
        _ec_gui_slider->reset_sliders();
        
        _start_devices.clear();
        _device_start_req=false;
        
        fill_start_stop_motor();
        fill_start_stop_valve();
        fill_start_stop_pump();
        
        if(!_motors_selected && !_valves_selected &&!_pumps_selected){
            _cmd_message="No device selected, please select at least one";
            launch_cmd_message(_cmd_message);
        }
        else{
            
            onApplyCmd();

            if(_device_start_req){
                _device_controlled=true;

                _ec_gui_slider->disable_control_mode("");
                _notallbtn->setEnabled(false);
                _allbtn->setEnabled(false);

                for (auto& [slave_id, slider_wid]:_slider_map.motor_sw_map){
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

bool EcGuiCmd::get_command_sts()
{
    return _device_controlled;
}

void EcGuiCmd::set_command_sts(bool device_controlled)
{
    _device_controlled=device_controlled;
}

EcGuiCmd::~EcGuiCmd()
{
    if(_client){
        _client->stop_devices();
    }
}
