#include "ec_gui_start.h"

#include <iostream>
#include <csignal>
#include <atomic>

#include <QLabel>
#include <QPixmap>
#include <QFile>

#include <chrono>
#define _HYST_THRESHOLD 5 // 5s

#define LO_PWR_DC_MC 0x12
#define CENT_AC 0x15
#define FT6 0x20
#define POW_F28M36_BOARD 0x32
#define IMU_ANY 0x40

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

EcGuiStart::EcGuiStart(EcUtils::EC_CONFIG ec_config,EcIface::Ptr client,QWidget *parent) :
    QMainWindow(parent),
    _ec_config(ec_config),
    _client(client)
{

    /* Load ui */
    auto wid = LoadUiFile(this);
    
    _command_dw = findChild<QDockWidget *>("Command");
    connect(_command_dw, SIGNAL(topLevelChanged(bool)), this, SLOT(DwTopLevelChanged(bool)));
    
    _pdo_dw = findChild<QDockWidget *>("DataObject");
    connect(_pdo_dw, SIGNAL(topLevelChanged(bool)), this, SLOT(DwTopLevelChanged(bool)));
    
    _graphics_dw = findChild<QDockWidget *>("Graphics");
    connect(_graphics_dw, SIGNAL(topLevelChanged(bool)), this, SLOT(DwTopLevelChanged(bool)));
    
    _net_tree_wid = findChild<QTreeWidget *>("NetworkSetup");
    _net_tree_wid->resizeColumnToContents(0);
    _net_tree_wid->expandAll();
    
    _scan_device = findChild<QPushButton *>("ScanDevice");
    connect(_scan_device, &QPushButton::released,
            this, &EcGuiStart::onScanDeviceReleased);
    
    /*frequency */

    _freq_combobox = findChild<QComboBox *>("Freq");
    _freq_combobox->setCurrentIndex(1); // set Default 2.5ms.

    // change FONT
    QFont font;
    font.setPointSize(font.pointSize() + 5);
    _freq_combobox->setFont(font);

    /* connection of frequency function */
    connect(_freq_combobox, SIGNAL(currentIndexChanged(int)),this,
        SLOT(OnFreqChanged())
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
    
    _receive_action = findChild<QAction *>("actionReceive");
    connect(_receive_action, SIGNAL(triggered()), this, SLOT(stat_receive()));
    
    _stop_receive_action = findChild<QAction *>("actionStopReceive");
    connect(_stop_receive_action, SIGNAL(triggered()), this, SLOT(stop_receive()));
    
    _record_action = findChild<QAction *>("actionRecord");
    connect(_record_action, SIGNAL(triggered()), this, SLOT(stat_record()));
    
    _stop_record_action = findChild<QAction *>("actionStopRecord");
    connect(_stop_record_action, SIGNAL(triggered()), this, SLOT(stop_record()));
    
    _receive_started = _record_started = false;
    
     
    _slave_id_led = _ec_config.slave_id_led;
    
    _ec_gui_slider = std::make_shared<EcGuiSlider>(this);
    
    _ec_gui_pdo = std::make_shared<EcGuiPdo>(_ec_gui_slider,
                                             _client,
                                             this);
    
    _ec_gui_sdo = std::make_shared<EcGuiSdo>(_client,
                                             this);

    _ec_gui_cmd = std::make_shared<EcGuiCmd>(_ec_gui_slider,
                                             _client,
                                             this);

    _send_ref=_first_send=false;
        
    // Get Send and Stop button
    _send_stop_btn = findChild<QPushButton *>("SendStopBtn");

    connect(_send_stop_btn, &QPushButton::released,
            this, &EcGuiStart::onSendStopBtnReleased);
    
    // create a timer for sending PDO
    _send_timer = new QTimer(this);

    // setup signal and slot
    connect(_send_timer, SIGNAL(timeout()),
        this, SLOT(send()));


    // create a timer for receiving PDO
    _receive_timer = new QTimer(this);

    // setup signal and slot
    connect(_receive_timer, SIGNAL(timeout()),
        this, SLOT(receive()));


    OnFreqChanged();
    
    
}

void EcGuiStart::restart_gui()
{
    add_device();
    _ec_gui_slider->create_sliders(_joint_info_map);
    
    _ec_gui_cmd->restart_ec_gui_cmd();
    
    _ec_gui_pdo->restart_ec_gui_pdo();
    _ec_gui_pdo->set_internal_map(_internal_motor_status_map,
                                  _internal_ft6_status_map,
                                  _internal_pow_status_map,
                                  _internal_imu_status_map);
    
    _ec_gui_sdo->set_internal_sdo_map(_internal_sdo_map);
    _ec_gui_sdo->restart_ec_gui_sdo(_sdo_map);
}


void EcGuiStart::try_gui()
{
// ********************* TEST ****************************////

    _device_info.push_back(std::make_tuple(201,POW_F28M36_BOARD,0));   // create power board
    
    _internal_pow_status_map[201]={48.0,48.0,2.0,25.0,25.0,25.0};
    
    RR_SDO rr_sdo_info_motor = {
        { "motor_pos", 0.0},
        { "Min_pos",  -1.0},
        { "Max_pos",   1.0},
        { "motor_vel", 0.0},
        { "Max_vel",   2.0},
        { "torque",    0.0},
        { "Max_tor",   5.0}
    };
    
    for(int i=1; i<21; i++)
    {
        _device_info.push_back(std::make_tuple(i,CENT_AC,i));   // create motor board
        EcGuiSlider::joint_info_t joint_info_s;

        joint_info_s.joint_name    ="motor_id_"+std::to_string(i);
        joint_info_s.actual_pos    =0.0;
        joint_info_s.min_pos       =-1.0;
        joint_info_s.max_pos       =1.0;
        joint_info_s.actual_vel    =0.0;
        joint_info_s.max_vel       =2.0;
        joint_info_s.actual_torq   =0.0;
        joint_info_s.max_torq      =5.0;

        _joint_info_map[i]=joint_info_s;
        _internal_motor_status_map[i] = std::make_tuple(10,10,0,0,100,25,25,0,0,0,0,0);
        _internal_sdo_map[i] = rr_sdo_info_motor;
    }
    _device_info.push_back(std::make_tuple(100,FT6,21));   // create ft6 board
    _internal_ft6_status_map[100]={10.0,5.0,2.0,100.0,125.0,130.0};
    _device_info.push_back(std::make_tuple(101,FT6,22));   // create ft6 board
    _internal_ft6_status_map[101]={10.0,5.0,2.0,100.0,125.0,130.0};
    _device_info.push_back(std::make_tuple(102,IMU_ANY,23));   // create imu board
    _internal_imu_status_map[102]={5.0,15.0,20.0,5.0,2.0,3.0,0,0,0,1};
    _device_info.push_back(std::make_tuple(103,IMU_ANY,24));   // create imu board
    _internal_imu_status_map[103]={5.0,15.0,20.0,5.0,2.0,3.0,0,0,0,1};

// ********************* TEST ****************************////
    
}

void EcGuiStart::error_on_scannig()
{
    QMessageBox msgBox;
    msgBox.setText("Cannot find EtherCAT devices on network"
                ", please control the EtherCAT Master status or sever status");
    msgBox.exec();
}

void EcGuiStart::add_device()
{
    auto ec_device_item =_net_tree_wid->topLevelItem(0)->child(2)->child(0);
    for ( auto &[esc_id, type, pos] : _device_info )
    {
        QTreeWidgetItem * type_item = new QTreeWidgetItem();
        std::string type_str = "   type: "+std::to_string(type);
        type_item->setText(0,QString::fromStdString(type_str));
        type_item->setFont(0,QFont("Sans Serif", 10));
        
        QTreeWidgetItem * pos_item = new QTreeWidgetItem();
        std::string pos_str = "   pos: "+std::to_string(pos);
        pos_item->setText(0,QString::fromStdString(pos_str));
        pos_item->setFont(0,QFont("Sans Serif", 10));
        
        QTreeWidgetItem * esc_item = new QTreeWidgetItem();
        std::string esc_id_name = "esc_id_"+std::to_string(esc_id);
        esc_item->setText(0,QString::fromStdString(esc_id_name));
        esc_item->setFont(0,QFont("Sans Serif", 12));
        esc_item->setIcon(0,QIcon(":/icon/esc_icon.png"));
        
        esc_item->addChild(type_item);
        esc_item->addChild(pos_item);
        ec_device_item->addChild(esc_item);
        
    }
}

void EcGuiStart::clear_device()
{
    auto ec_device_item =_net_tree_wid->topLevelItem(0)->child(2)->child(0);
    while(ec_device_item->childCount()>0)
    {
        auto child = ec_device_item->child(0);
        ec_device_item->removeChild(child);
    }
    
    _device_info.clear();
    _joint_info_map.clear();
    
    _internal_motor_status_map.clear();
    _internal_ft6_status_map.clear();
    _internal_pow_status_map.clear();
    _internal_imu_status_map.clear();
    
    _sdo_map.clear();
    _internal_sdo_map.clear();
    
    stop_receive();
    stop_record();
}

void EcGuiStart::scan_device()
{
    if(_client->retrieve_slaves_info(_device_info))
    {
        if(_device_info.empty())
        {
            error_on_scannig();
        }
        else
        {

            // *************** END AUTODETECTION *************** //

            // GET Mechanical Limits
            _joint_info_map.clear();
            std::map<int,RR_SDO> motor_info_map;
            RD_SDO rd_sdo = { "motor_pos","Min_pos","Max_pos","motor_vel","Max_vel","torque","Max_tor"};
            WR_SDO wr_sdo = {};
            int motors_counter=0;
            
            for ( auto &[esc_id, type, pos] : _device_info )
            {
                RR_SDO rr_sdo_info;
                if(type==CENT_AC || type==LO_PWR_DC_MC)
                {
                    motors_counter++;
                    if(_client->retrieve_rr_sdo(esc_id,rd_sdo,wr_sdo,rr_sdo_info))
                    {    
                        if(!rr_sdo_info.empty())
                        {
                            motor_info_map[esc_id]=rr_sdo_info;
                        }
                    }
                }
                /********************* RETRIEVE ALL SDO */////////////
                rr_sdo_info.clear();
                _client->retrieve_all_sdo(esc_id,rr_sdo_info);
                _sdo_map[esc_id] = rr_sdo_info;
            }

            if((motor_info_map.size()!=motors_counter)|| (motor_info_map.empty()))
            {
                QMessageBox msgBox;
                msgBox.setText("Cannot find the SDO information requested, mechanical limits and actual position, velocity and torque for all motors"
                            ", please control the EtherCAT Slave setup and restart the GUI");
                msgBox.exec();
            }
            else
            {
                for ( auto &[slave_id, type, pos] : _device_info )
                {
                    if(motor_info_map.count(slave_id)>0)
                    {
                        std::map<std::string,float> slaves_sdo_data=motor_info_map[slave_id];

                        EcGuiSlider::joint_info_t joint_info_s;

                        joint_info_s.joint_name    ="motor_id_"+std::to_string(slave_id);
                        joint_info_s.actual_pos    =slaves_sdo_data.at("motor_pos");
                        joint_info_s.min_pos       =slaves_sdo_data.at("Min_pos");
                        joint_info_s.max_pos       =slaves_sdo_data.at("Max_pos");
                        if(joint_info_s.max_pos<joint_info_s.min_pos)
                        {
                        double aux_value=joint_info_s.min_pos;
                        joint_info_s.min_pos=joint_info_s.max_pos;
                        joint_info_s.max_pos=aux_value;
                        }
                        joint_info_s.actual_vel    =slaves_sdo_data.at("motor_vel");
                        joint_info_s.max_vel       =slaves_sdo_data.at("Max_vel");
                        joint_info_s.actual_torq   =slaves_sdo_data.at("torque");
                        joint_info_s.max_torq      =slaves_sdo_data.at("Max_tor");


                        _joint_info_map[slave_id]=joint_info_s;
                        // parse the message taking the information requested. Save it into the joint_info_map.
                    }
                }
            }
        }
    }
    else
    {
        error_on_scannig();
    }
}

void EcGuiStart::onScanDeviceReleased()
{
    if(!_device_info.empty())
    {
        QMessageBox::StandardButton reply;
        QMessageBox msgBox;
        reply = msgBox.warning(this,msgBox.windowTitle(),
                            tr("EtherCAT device(s) already scanned.\n"
                            "Do you want to rescan?"),
                                QMessageBox::Yes|QMessageBox::No);
        
        if(reply == QMessageBox::Yes)
        {
            if(!_ec_gui_cmd->get_cmd_sts(_ctrl_cmd))
            {
                clear_device();
                scan_device();
            }
            else
            {
                msgBox.critical(this,msgBox.windowTitle(),
                                tr("Cannot rescan EtherCAT device(s) already started or controlled.\n"));
                return;
            }
        }
        else
        {
            return;
        }
    }
    else
    {
        scan_device();
    }
    
    if(_device_info.empty())
    {
#ifdef TEST 
        try_gui();
        restart_gui();
#endif
    }
    else
    {
        restart_gui();
    }
        
}

void EcGuiStart::DwTopLevelChanged(bool isFloating)
{
    auto dw = qobject_cast<QDockWidget*>(sender());
	if(isFloating)
	{	dw->setWindowFlags(Qt::Window);
		dw->show();
	}
}

double EcGuiStart::getFreq() const
{
    return _freq_combobox->currentText().toDouble();
}

void EcGuiStart::OnFreqChanged()
{
    double freq=getFreq();
    double time_s= 1/freq;
    
    _hysteresis_battery_level=  _HYST_THRESHOLD * freq; 
    
    if(_hysteresis_battery_level <= 1)
    {
        _hysteresis_battery_level= 2; // twice of communication time (i.e 5s-->10s or 10s-->20s)
    }

    _time_ms=(int) 1000*time_s;
    

/**** RX STOP and START *****/

    if(_receive_started)
    {
        _receive_timer->stop();

        _ec_gui_pdo->restart_receive_timer();

        _receive_timer->start(_time_ms);

        _client->set_loop_time(_time_ms);
    }


/**** RX STOP and START *****/

}


void EcGuiStart::onSendStopBtnReleased()
{
    // @NOTE to be tested.
    _ec_gui_slider->reset_sliders();

    _send_ref = _ec_gui_cmd->get_cmd_sts(_ctrl_cmd);
    _ec_gui_pdo->set_ctrl_mode(_ctrl_cmd);
    
    if((_send_stop_btn->text()=="Send")&&(_send_ref))
    {
        _send_timer->start(_time_ms);
        _freq_combobox->setEnabled(false);
        _send_stop_btn->setText("Stop");
        _first_send=true;

        _ec_gui_slider->enable_sliders();

    }
    else
    {
        _send_timer->stop();
        _freq_combobox->setEnabled(true);        
        _ec_gui_slider->disable_sliders();

        if(_send_stop_btn->text()=="Send")
        {
            QMessageBox msgBox;
            msgBox.setText("Cannot send references without starting the motors"
                           ", please launch START EtherCAT command ");
            msgBox.exec();
        }
        else
        {
            _send_stop_btn->setText("Send");
            _first_send=true;
            _ec_gui_pdo->set_filter(_first_send,_time_ms);
            send();  //STOP the motors using the actual position of the motor and zero feedforward torque (in impedance) or zero velocity.
        }
        _first_send=false;
    }
    
    _ec_gui_pdo->set_filter(_first_send,_time_ms);
}



void EcGuiStart::send()
{
    if(_ec_gui_cmd->get_cmd_sts(_ctrl_cmd)) // stop motors command
    {
        _ec_gui_pdo->write();
    }
    else
    {
        onSendStopBtnReleased(); // stop sending references
    }
}

void EcGuiStart::stat_record()
{
    if(!_record_started)
    {
        if(_device_info.empty())
        {
            QMessageBox msgBox;
            msgBox.warning(this,msgBox.windowTitle(),
                           tr("Cannot recod PDO.\n"
                           "Please press scan device button"),
                           QMessageBox::Ok);
        }
        else
        {
            _record_started = true;
            _client->start_logging();
        }
    }
}

void EcGuiStart::stop_record()
{
    if(_record_started)
    {
        _record_started = false;
        _client->stop_logging();
    }
}

void EcGuiStart::stat_receive()
{
    if(!_receive_started)
    {
        if(_device_info.empty())
        {
            QMessageBox msgBox;
            msgBox.warning(this,msgBox.windowTitle(),
                        tr("Cannot receive PDO.\n"
                        "Please press scan device button"),
                            QMessageBox::Ok);
        }
        else
        {
            _receive_started = true;
        
            _ec_gui_pdo->restart_receive_timer();

            _receive_timer->start(_time_ms);
        }
    }
}

void EcGuiStart::stop_receive()
{
    if(_receive_started)
    {
        _receive_started = false;
        
        _receive_timer->stop();
    }
}

void EcGuiStart::receive()
{
    if(_client->is_client_alive())
    {
        /************************************* READ PDOs  ********************************************/
        _ec_gui_pdo->read();
        /************************************* READ PDOs  ********************************************/
        
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
