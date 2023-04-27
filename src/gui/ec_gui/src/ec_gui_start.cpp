#include "ec_gui_start.h"
#include "ec_pdo_read.h"

#include <iostream>
#include <csignal>
#include <atomic>

#include <QLabel>
#include <QPixmap>
#include <QFile>

#include <chrono>
#define _HYST_THRESHOLD 5 // 5s 

using namespace std::chrono;

void ec_gui_start_widget_qrc_init()
{
    Q_INIT_RESOURCE(ec_gui_start_resources);
}

namespace
{

QWidget * LoadUiFile(QWidget * parent)
{
    ec_gui_start_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/ec_gui_start.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);

    file.close();

    return formWidget;
}

}

EcGuiStart::EcGuiStart(std::map<int ,joint_info_t > joint_info_map,EcUtils::EC_CONFIG ec_config,std::shared_ptr<Client> client,QWidget *parent) :
    _joint_info_map(joint_info_map),
    _ec_config(ec_config),
    _client(client),
    QWidget(parent)
{

    /* Load ui */
    auto wid = LoadUiFile(this);

    /*  EtherCAT Master commands */
    _fieldtype_combobox = findChild<QComboBox *>("SelectFieldComboBox");

    /* connection of read command function */
    connect(_fieldtype_combobox, SIGNAL(currentIndexChanged(int)),this,
        SLOT(readCommand())
    );

    /*  create mode type to start he motors */
    _mode_type_combobox = findChild<QComboBox *>("ModeType");

    /* connection of read mode type function */
    connect(_mode_type_combobox, SIGNAL(currentIndexChanged(int)),this,
        SLOT(readModeType())
    );


    /*  UDP frequency */

    _udp_freq_combobox = findChild<QComboBox *>("UDPFreq");
    _udp_freq_combobox->setCurrentIndex(1); // set Default 2.5ms.

    // change FONT
    QFont font;
    font.setPointSize(font.pointSize() + 5);
    _udp_freq_combobox->setFont(font);

    /* connection of UDP frequency function */
    connect(_udp_freq_combobox, SIGNAL(currentIndexChanged(int)),this,
        SLOT(OnUDPFreqChanged())
    );

    _battery_level = findChild<QLCDNumber *>("BatteryLevel");
    _battery_level->setDigitCount(6);
    _battery_level->display(888888);
    _battery_level->setStyleSheet("background: black; color: #00FF00");
    
    _timer_change_color = new QTimer(this);
    _flashing=false;
    _first_detection=true;
    _count_warning = _count_not_warning = 0;
    connect(_timer_change_color, SIGNAL(timeout()), this, SLOT(warnig_level_batt()));
    

    // find position, velocity and torque tab for adding the sliders for every slave.

    _tabcontrol = findChild<QTabWidget *>("tabControl");

    _sliders_poslayout  = findChild<QVBoxLayout *>("positionSliders");
    _sliders_vellayout  = findChild<QVBoxLayout *>("velocitySliders");
    _sliders_torqlayout = findChild<QVBoxLayout *>("torqueSliders");

    /* Getting command manager (Apply) */
    _cmd_manager = findChild<QDialogButtonBox *>("CmdManager");
    _applybtn = _cmd_manager->button(QDialogButtonBox::Apply);

    connect(_applybtn, &QPushButton::released,
           this, &EcGuiStart::onApplyCmdReleased);

    // Get Send and Stop button

    _send_stop_btn = findChild<QPushButton *>("SendStopBtn");

    connect(_send_stop_btn, &QPushButton::released,
            this, &EcGuiStart::onSendStopBtnReleased);

    /* Getting command manager (Apply) */
    auto dis_enable_slaves = findChild<QDialogButtonBox *>("DisEnableSlaves");

   _notallbtn = dis_enable_slaves->button(QDialogButtonBox::NoToAll);

    connect(_notallbtn, &QPushButton::released,
            this, &EcGuiStart::onNotAllCmdReleased);

    _allbtn = dis_enable_slaves->button(QDialogButtonBox::YesToAll);

    connect(_allbtn, &QPushButton::released,
            this, &EcGuiStart::onAllCmdReleased);


     _tree_wid = findChild<QTreeWidget *>("MotorData");

     
    _slave_id_led = _ec_config.slave_id_led;
    
    // Create the sliders for every tabs

     for (auto& [slave_id, joint_info_s]:_joint_info_map)
     {

         QString jname = QString::fromStdString(joint_info_s.joint_name);

         std::vector<double> gains_pos={220.0,0.0,10.0};
         std::vector<std::string> pid_string={"P","I","D"};

         auto wid_p = new SliderWidget(jname,joint_info_s.actual_pos,QString::number(joint_info_s.min_pos),QString::number(joint_info_s.max_pos),"[rad]",0x3B,pid_string,gains_pos,this);
         wid_p->setMaximumHeight(300);
         wid_p->disable_slider();
         bool is_slave_led=false;
         
         for(int led_index=0; led_index < _slave_id_led.size() && !is_slave_led ;led_index++)
         {
            if(slave_id==_slave_id_led[led_index])
            {
                is_slave_led=true;
            }
         }
         
         if(!is_slave_led)
         {
            wid_p->hide_led_on_off_btn();
         }
         else
         {
            auto led_on_off_btn=wid_p->get_led_on_off_btn();
            led_on_off_btn->setStyleSheet("background: red; color: #00FF00");
            connect(led_on_off_btn, &QPushButton::released,this, &EcGuiStart::onLED_ON_OFF_Released); 
         }

         _position_sw_map[slave_id]=wid_p;

         std::vector<double> gains_vel={20.0,0.0,0.0};

         auto wid_v = new SliderWidget(jname,0.0,QString::number(-joint_info_s.max_vel),QString::number(joint_info_s.max_vel),"[rad/s]",0x71,pid_string,gains_vel,this);
         wid_v->setMaximumHeight(300);
         wid_v->disable_slider();
         
         if(!is_slave_led)
         {
             wid_v->hide_led_on_off_btn();
         }
         else
         {
            auto led_on_off_btn=wid_v->get_led_on_off_btn();
            led_on_off_btn->setStyleSheet("background: red; color: #00FF00");
            connect(led_on_off_btn, &QPushButton::released,this, &EcGuiStart::onLED_ON_OFF_Released); 
         }
         
         _velocity_sw_map[slave_id]=wid_v;

         gains_pos.clear();
         gains_pos={500.0,0.0,10.0};
         auto wid_p_t = new SliderWidget(jname,joint_info_s.actual_pos,QString::number(joint_info_s.min_pos),QString::number(joint_info_s.max_pos),"[rad]",0xD4,pid_string,gains_pos,this);
         wid_p_t->setMaximumHeight(300);
         wid_p_t->disable_slider();
         
         if(!is_slave_led)
         {
             wid_p_t->hide_led_on_off_btn();
         }
         else
         {
            auto led_on_off_btn=wid_p_t->get_led_on_off_btn();
            led_on_off_btn->setStyleSheet("background: red; color: #00FF00");
            connect(led_on_off_btn, &QPushButton::released,this, &EcGuiStart::onLED_ON_OFF_Released);
         }
             
         
         
         auto wid_p_t_calib=wid_p_t->get_wid_calibration();
         wid_p_t_calib->hide_slider_calib(1);
         
         _position_t_sw_map[slave_id]=wid_p_t;

         std::vector<double> gains_tor={1.0, 0.7, 0.007};
         pid_string.clear();
         pid_string={"Tau_p","Tau_fc","Tau_d"};

         auto wid_t = new SliderWidget("",0.0,QString::number(-joint_info_s.max_torq/100),QString::number(joint_info_s.max_torq/100),"[Nm]",0xD4,pid_string,gains_tor,this);
         wid_t->setMaximumHeight(300);
        
         if(!is_slave_led)
         {
            wid_t->setContentsMargins(2*jname.size(),0,0,0); 
         }
         else
         {
            wid_t->setContentsMargins(2*jname.size(),0,85,0);
         }
         
         wid_t->disable_slider();
         wid_t->hide_joint_enabled();
         wid_t->hide_led_on_off_btn();
//         auto wid_t_calib=wid_t->get_wid_calibration();
//         for(int i=0 ; i < 3 ; i++)
//         {
//             wid_t_calib->disable_slider_calib(i);
//         }

         _torque_sw_map[slave_id]=wid_t;

         _sliders_poslayout->addWidget(wid_p,0, Qt::AlignTop);
         _sliders_vellayout->addWidget(wid_v,0, Qt::AlignTop);
         _sliders_torqlayout->addWidget(wid_p_t,0, Qt::AlignTop);
         _sliders_torqlayout->addWidget(wid_t,0, Qt::AlignTop);
     }

    _motor_start_req=_send_ref=_first_send_udp_comm=false;
    _value=0;
    readCommand();

    // create a timer for UDP communication (send)
    _UDPTimer_send = new QTimer(this);

    // setup signal and slot
    connect(_UDPTimer_send, SIGNAL(timeout()),
        this, SLOT(UDP_Communication_send()));


    // create a timer for UDP communication (read)
    _UDPTimer_receive = new QTimer(this);

    // setup signal and slot
    connect(_UDPTimer_receive, SIGNAL(timeout()),
        this, SLOT(UDP_Communication_receive()));

    _ec_pdo_read = std::make_shared<EcPDORead>(_client,_tree_wid);

    OnUDPFreqChanged();

    auto l = new QVBoxLayout;
    l->addWidget(wid);
    setLayout(l);
}

std::string EcGuiStart::getFieldType() const
{
    return _fieldtype_combobox->currentText().toStdString();
}

std::string EcGuiStart::getModeType() const
{
    return _mode_type_combobox->currentText().toStdString();
}

double EcGuiStart::getUDPFreq() const
{
    return _udp_freq_combobox->currentText().toDouble();
}

void EcGuiStart::readCommand()
{
    _tabcontrol->setEnabled(false);
    _mode_type_combobox->setEnabled(false);

    _ctrl_cmd_type=Client::ClientCmdType::STOP;

    if(getFieldType() == "Start motors")
    {
        _tabcontrol->setEnabled(true);
        _mode_type_combobox->setEnabled(true);
        _ctrl_cmd_type=Client::ClientCmdType::START;
        readModeType();
        if(!_motor_start_req)
        {
            _notallbtn->setEnabled(true);
            _allbtn->setEnabled(true);
        }
    }
    else if(getFieldType() == "Stop motors")
    {
        _ctrl_cmd_type=Client::ClientCmdType::STOP;

        _notallbtn->setEnabled(false);
        _allbtn->setEnabled(false);
    }
    else
    {
        throw std::runtime_error("Error: Found not valid EtherCAT Master commnad");
    }
}

void EcGuiStart::readModeType()
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

void EcGuiStart::onNotAllCmdReleased()
{
    /* Uncheck all checkboxes of Joint WID */
    for (auto& [slave_id, slider_wid]:_sw_map_selected)
    {
        slider_wid->uncheck_joint_enabled();
    }

}

void EcGuiStart::onAllCmdReleased()
{
    /* Check all checkboxes of Slider WID */
    for (auto& [slave_id, slider_wid]:_sw_map_selected)
    {
        slider_wid->check_joint_enabled();
    }

}

void EcGuiStart::onLED_ON_OFF_Released()
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


void EcGuiStart::onApplyCmdReleased()
{
    
    // @NOTE to be tested.
    for (auto& [slave_id, slider_wid]:_position_sw_map)
    {
        slider_wid->align_spinbox();
        _position_t_sw_map[slave_id]->align_spinbox();
        _velocity_sw_map[slave_id]->align_spinbox(0.0);
        _torque_sw_map[slave_id]->align_spinbox(0.0);
    }
    
    if((_motor_start_req)&&(_ctrl_cmd_type==Client::ClientCmdType::START))
    {
        QMessageBox msgBox;
        msgBox.setText("All or some slaves are already started/released brake"
                       ", please launch STOP EtherCAT command");
        msgBox.exec();
    }
    else if((!_motor_start_req)&&(_ctrl_cmd_type==Client::ClientCmdType::STOP))
    {
        QMessageBox msgBox;
        msgBox.setText("No slaves was started releasing the brake"
                       ", please launch START EtherCAT command");
        msgBox.exec();
    }
    else
    {
        if(_ctrl_cmd_type==Client::ClientCmdType::STOP)
        {
            _UDPTimer_send->stop();
            _send_stop_btn->setText("Send");
            _first_send_udp_comm=false;
            
            _brake_cmds.clear();

            for (auto& [slave_id, slider_wid]:_sw_map_selected)
            {
                slider_wid->enable_joint_enabled();

                _position_sw_map[slave_id]->disable_slider();
                _position_t_sw_map[slave_id]->disable_slider();
                _velocity_sw_map[slave_id]->disable_slider();
                _torque_sw_map[slave_id]->disable_slider();

                _brake_cmds.push_back(std::make_tuple(slave_id,to_underlying(PdoAuxCmdType::BRAKE_ENGAGE)));
            }

            if(_client->pdo_aux_cmd(_brake_cmds))
            {
                std::this_thread::sleep_for(1000ms);
                if(_client->pdo_aux_cmd_sts(_brake_cmds))
                {
                    if(_client->stop_motors())
                    {
                        QMessageBox msgBox;
                        msgBox.setText("All slaves requested have performed the command successfully");
                        msgBox.exec();
                    }
                    else
                    {
                        QMessageBox msgBox;
                        msgBox.setText("Cannot perform the stop command on the slaves requested");
                        msgBox.exec();
                    }
                }
                else
                {
                    QMessageBox msgBox;
                    msgBox.setText("Cannot perform the engage brake command on the slaves requested");
                    msgBox.exec();
                }
            }


            _udp_freq_combobox->setEnabled(true);
            _motor_start_req=false;
            _send_ref=false;
        }
        else
        {
            _motors_start.clear();
            _brake_cmds.clear();

            for (auto& [slave_id, slider_wid]:_sw_map_selected)
            {
                if(slider_wid->is_joint_enabled())
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
                    _brake_cmds.push_back(std::make_tuple(slave_id,to_underlying(PdoAuxCmdType::BRAKE_RELEASE)));
                }
            }

            bool release_breke_req=false;

            if(!_motors_start.empty())
            {
                _motor_start_req=_client->start_motors(_motors_start);
                if(!_motor_start_req)
                {
                    QMessageBox msgBox;
                    msgBox.setText("Cannot perform the start command on the slaves requested");
                    msgBox.exec();

                    return;
                }
                else
                {
                    release_breke_req=_client->pdo_aux_cmd(_brake_cmds);  // USE PDO COMMAND
                }
            }
            else
            {
                if(!_brake_cmds.empty())
                {
                    RD_SDO rd_sdo{};
                    for(int i=0;i<_brake_cmds.size();i++)
                    {
                        int esc_id,brake_req;
                        std::tie(esc_id,brake_req) = _brake_cmds[i];
                        WR_SDO wr_sdo{};
                        wr_sdo.push_back(std::make_tuple("ctrl_status_cmd",std::to_string(brake_req)));
                        release_breke_req &= _client->set_wr_sdo(esc_id,rd_sdo,wr_sdo); // USE SDO COMMAND
                    }
                }
                else
                {
                    QMessageBox msgBox;
                    msgBox.setText("No Slave selected, please select at least one slave");
                    msgBox.exec();

                    return;
                }
            }

            if(release_breke_req)
            {
                std::this_thread::sleep_for(1000ms);
                if(_client->pdo_aux_cmd_sts(_brake_cmds))
                {
                    _mode_type_combobox->setEnabled(false);
                    _send_ref=true;
                    _motor_start_req = true; // motors started in IDLE

                    _notallbtn->setEnabled(false);
                    _allbtn->setEnabled(false);

                    for (auto& [slave_id, slider_wid]:_sw_map_selected)
                    {
                        slider_wid->disable_joint_enabled();
                    }


                    QMessageBox msgBox;
                    msgBox.setText("All slaves requested have performed the command successfully");
                    msgBox.exec();
                }
                else
                {
                    QMessageBox msgBox;
                    msgBox.setText("Wrong status of the brakes requested");
                    msgBox.exec();
                }
            }
            else
            {
                QMessageBox msgBox;
                msgBox.setText("Cannot perform the release brake command on the slaves requested");
                msgBox.exec();
            }
        }
    }

}

void EcGuiStart::OnUDPFreqChanged()
{
    double udp_freq=getUDPFreq();
    double udp_s_req= 1/udp_freq;
    
    _hysteresis_battery_level=  _HYST_THRESHOLD * udp_freq; 
    
    if(_hysteresis_battery_level <= 1)
    {
        _hysteresis_battery_level= 2; // twice of communication time (i.e 5s-->10s or 10s-->20s)
    }

    _udp_ms_req=(int) 1000*udp_s_req;
    

/**** RX STOP and START *****/

    _UDPTimer_receive->stop();

    _ec_pdo_read->restart_receive_timer();

    _UDPTimer_receive->start(_udp_ms_req);
    auto period=std::chrono::milliseconds(_udp_ms_req);

    _client->set_period(period);


/**** RX STOP and START *****/

}


void EcGuiStart::onSendStopBtnReleased()
{
    // @NOTE to be tested.
    for (auto& [slave_id, slider_wid]:_position_sw_map)
    {
        slider_wid->align_spinbox();
        _position_t_sw_map[slave_id]->align_spinbox();
        _velocity_sw_map[slave_id]->align_spinbox(0.0);
        _torque_sw_map[slave_id]->align_spinbox(0.0);
    }
    
    if((_send_stop_btn->text()=="Send")&&(_send_ref))
    {
        _first_send_udp_comm=true;

        _UDPTimer_send->start(_udp_ms_req);
        _udp_freq_combobox->setEnabled(false);
        _send_stop_btn->setText("Stop");

        for (auto& [slave_id, slider_wid]:_sw_map_selected)
        {
            if(slider_wid->is_joint_enabled())
            {
                _position_sw_map[slave_id]->enable_slider();
                _position_t_sw_map[slave_id]->enable_slider();
                _velocity_sw_map[slave_id]->enable_slider();
                _torque_sw_map[slave_id]->enable_slider();
            }
        }

    }
    else
    {
        _UDPTimer_send->stop();
        for (auto& [slave_id, slider_wid]:_position_sw_map)
        {
//             slider_wid->align_spinbox();
//             _position_t_sw_map[slave_id]->align_spinbox();
//             _velocity_sw_map[slave_id]->align_spinbox(0.0);
//             _torque_sw_map[slave_id]->align_spinbox(0.0);

            slider_wid->disable_slider();
            _position_t_sw_map[slave_id]->disable_slider();
            _velocity_sw_map[slave_id]->disable_slider();
            _torque_sw_map[slave_id]->disable_slider();

        }
        _udp_freq_combobox->setEnabled(true);
        _send_stop_btn->setText("Send");

        if(!_send_ref)
        {
            QMessageBox msgBox;
            msgBox.setText("Cannot send references without starting the motors"
                           ", please launch START EtherCAT command ");
            msgBox.exec();
        }
        else
        {
            _first_send_udp_comm=true;
            UDP_Communication_send();  //STOP the motors using the actual position of the motor and zero feedforward torque (in impedance) or zero velocity.
        }
        _first_send_udp_comm=false;
    }
}

double  EcGuiStart::filtering(SecondOrderFilter<double>::Ptr filter,double actual_value)
{
    if(_first_send_udp_comm)
    {
        filter->reset(actual_value);
        double ts=((double) _udp_ms_req)/1000;
        filter->setTimeStep(ts);
    }

    // Second Order Filtering

    double value_filtered=filter->process(actual_value);

    return value_filtered;
}


void EcGuiStart::UDP_Communication_send()
{
    _motors_ref.clear();
    _motor_ref_flags = MotorRefFlags::FLAG_MULTI_REF;

    for (auto& [slave_id, slider_wid]:_sw_map_selected)
    {
        _gains.clear();
        auto gains_calib_selected=_sw_map_selected[slave_id]->get_wid_calibration();

        for(int calib_index=0; calib_index < gains_calib_selected->get_slider_numb(); calib_index++)
        {
            double gain_filtered=filtering(gains_calib_selected->get_slider_filter(calib_index),gains_calib_selected->get_slider_value(calib_index));
            //_gains.push_back(gains_calib_selected->get_slider_value(calib_index));
            _gains.push_back(gain_filtered);
        }
        if(_value==0xD4)
        {
            _gains.erase(_gains.begin()+1);
            auto gains_t_calib= _torque_sw_map[slave_id]->get_wid_calibration();
            for(int calib_index=0; calib_index < gains_t_calib->get_slider_numb(); calib_index++)
            {
                double gain_t_filtered=filtering(gains_t_calib->get_slider_filter(calib_index),gains_t_calib->get_slider_value(calib_index));
                //_gains.push_back(gains_t_calib->get_slider_value(calib_index));
                _gains.push_back(gain_t_filtered);
            }
        }
        else
        {
            _gains.push_back(0);
            _gains.push_back(0);
        }

        double pos_ref= filtering(_position_sw_map[slave_id]->get_filer(),_position_sw_map[slave_id]->get_spinbox_value());
        if(_value==0xD4)
        {
            pos_ref= filtering(_position_t_sw_map[slave_id]->get_filer(),_position_t_sw_map[slave_id]->get_spinbox_value());
        }

        double vel_ref= filtering(_velocity_sw_map[slave_id]->get_filer(),_velocity_sw_map[slave_id]->get_spinbox_value());
        double tor_ref= filtering(_torque_sw_map[slave_id]->get_filer(),_torque_sw_map[slave_id]->get_spinbox_value());
                                   
        MR references{slave_id, _value, pos_ref, vel_ref, tor_ref, _gains[0], _gains[1],_gains[2], _gains[3], _gains[4],1,0,0};
        //            ID      CTRL_MODE, POS_REF, VEL_RF, TOR_REF,  GAIN_1,    GAIN_2,   GAIN_3,   GAIN_4,    GAIN_5, OP, IDX,AUX  OP->1 means NO_OP
        _motors_ref.push_back(references);


    }
    if(!_motors_ref.empty())
    {
       _client->set_motors_references(_motor_ref_flags, _motors_ref);
    }

    _first_send_udp_comm=false;

}

void EcGuiStart::UDP_Communication_receive()
{
    if(_client->is_client_alive())
    {
        /************************************* READ PDOs  ********************************************/
        _ec_pdo_read->read_motor_status();
        _ec_pdo_read->read_ft6_status();
        _ec_pdo_read->read_pow_status();
        /************************************* READ PDOs  ********************************************/
        
        auto motors_status_map= _client->get_motors_status();
        if(!motors_status_map.empty())
        {
            for ( const auto &[esc_id, motor_status] : motors_status_map)
            {
                /************************************* ALIGN POSITION SLIDERS with the motor position ********************************************/
                double motor_pos=std::get<1>(motor_status);
                if(_position_sw_map.count(esc_id)>0)
                {
                    _position_sw_map[esc_id]->set_actual_slider_value(motor_pos);
                    _position_t_sw_map[esc_id]->set_actual_slider_value(motor_pos);
                }
                /************************************* ALIGN POSITION SLIDERS with the motor position ********************************************/
            }
        }
        
        /************************************* CHECK BATTERY LEVEL ********************************************/
        auto pow_status_map= _client->get_pow_status();
        if(!pow_status_map.empty())
        {
            for ( const auto &[esc_id, pow_status] : pow_status_map)
            {
                float v_batt = pow_status[0];
                v_batt = std::round(v_batt * 100) / 100;
                _battery_level->display(v_batt);
                
                if( v_batt < 50.0)
                {
                    _count_not_warning = 0;
                    if(_count_warning  >= _hysteresis_battery_level)
                    {
                        if(_first_detection)
                        {
                            _first_detection=false;
                            _timer_change_color->start(500);
                        }
                    }
                    else
                    {
                        _count_warning = _count_warning +1;
                    }
                }
                else
                {
                    _count_warning = 0;
                    if(_count_not_warning  >= _hysteresis_battery_level)
                    {
                        if(!_first_detection)
                        {
                            _first_detection =true;
                            _timer_change_color->stop();
                            _battery_level->setStyleSheet("background: black; color: #00FF00");
                        }
                    }
                    else
                    {
                        _count_not_warning = _count_not_warning + 1;
                    }
                }
            }
        }
        /************************************* CHECK BATTERY LEVEL ********************************************/
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("Server is not alive!");
        msgBox.exec();
        QApplication::closeAllWindows();
    }
}

void EcGuiStart::warnig_level_batt()
{
    if(_flashing)
    {
        _battery_level->setStyleSheet("background: red; color: #00FF00");
        _flashing = false;
    }
    else
    {
        _battery_level->setStyleSheet("background: black; color: #00FF00");
        _flashing = true;
    }
}

EcGuiStart::~EcGuiStart()
{
    if(_client->is_client_alive())
    {
        _client->stop_client();
    }
}
