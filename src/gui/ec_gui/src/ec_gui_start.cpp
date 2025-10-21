#include "ec_gui_start.h"
#include "utils/ec_utils.h"

#include <iostream>
#include <csignal>
#include <atomic>

#include <QLabel>
#include <QPixmap>
#include <QFile>

#include <chrono>

#define _HYST_THRESHOLD 5 // 5s

static const std::string expert_user_password="facility";

using namespace std::chrono;
using namespace std::chrono_literals;

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

EcGuiStart::EcGuiStart(QWidget *parent) :
    QMainWindow(parent)
{
    /* Load ui */
    auto wid = LoadUiFile(this);
    
    _net_tree_wid = findChild<QTreeWidget *>("NetworkSetup");
    _net_tree_wid->resizeColumnToContents(0);
    _net_tree_wid->expandAll();
    _master_process_type= new QComboBox();
    _master_process_type->addItems({"Default Master", "Robot Master", "Ecat Master"});
    connect(_master_process_type, SIGNAL(currentIndexChanged(int)),this,SLOT(onEcMasterProcessChanged(int)));
    auto process_item =_net_tree_wid->topLevelItem(0)->child(2);
    _net_tree_wid->setItemWidget(process_item,0, _master_process_type);

    
    _ec_sys_start = findChild<QPushButton *>("StartEthercatSystem");
    connect(_ec_sys_start, &QPushButton::released,this, &EcGuiStart::onStartEtherCATSystem);
    
    _ec_sys_stop = findChild<QPushButton *>("StopEthercatSystem");
    connect(_ec_sys_stop, &QPushButton::released,this, &EcGuiStart::onStopEtherCATSystem);
    
    auto scan_device = findChild<QPushButton *>("ScanDevice");
    connect(scan_device, &QPushButton::released,this, &EcGuiStart::onScanDeviceReleased);

    _firmware_update_btn=findChild<QPushButton *>("FirmwareUpdate");
    _firmware_update_btn->setEnabled(false);
    connect(_firmware_update_btn, &QPushButton::released,this, &EcGuiStart::onFirmwareUpdateReleased);
    
    _firmware_update_wizard = new EcGuiFirmware(this);
    connect(_firmware_update_wizard, SIGNAL(finished(int)), this, SLOT(onFirmwarewizardClosed(int)));

    auto firmware_update_btns=_firmware_update_wizard->get_firmware_update_btns();
    connect(firmware_update_btns[0], &QPushButton::released,this, &EcGuiStart::onFirmwareUpdateCopyFiles);
    connect(firmware_update_btns[1], &QPushButton::released,this, &EcGuiStart::onFirmwareUpdateOpenConfig);
    connect(firmware_update_btns[2], &QPushButton::released,this, &EcGuiStart::onFirmwareUpdateStart);

    _expert_user = findChild<QLineEdit *>("ExpertUserPass");
    connect(_expert_user, &QLineEdit::returnPressed,std::bind(&EcGuiStart::ExpertUserPassChanged, this));
    
    _ec_gui_net = std::make_shared<EcGuiNet>(this);
    _ec_gui_wrapper = std::make_shared<EcGuiWrapper>(this);
    connect(_ec_gui_wrapper.get(),&EcGuiWrapper::clientStsChanged,this,&EcGuiStart::stopping_ec_sys);

    auto gui_status_bar = findChild<QStatusBar *>("Guistatusbar");
    _ec_system_status=new QLabel();  
    disable_ec_system();
    gui_status_bar->showMessage("EtherCAT GUI v0.0.1"); 
    gui_status_bar->setStyleSheet("font: 12pt;");
    gui_status_bar->addPermanentWidget(_ec_system_status); 
}

bool EcGuiStart::create_ec_iface()
{
    try{
        if(_ec_wrapper_info.client){
            _ec_wrapper_info.client->stop_client();
            _ec_wrapper_info.client.reset();
        }
        std::this_thread::sleep_for(100ms);
    }catch ( std::exception &e ){
        QMessageBox msgBox;
        msgBox.critical(this,msgBox.windowTitle(),tr(e.what()));
        return false;
    }
    
    auto ec_net_info = _ec_gui_net->get_net_setup();

    EcUtils::EC_CONFIG ec_cfg;
    ec_cfg.protocol=ec_net_info.protocol;
    if(ec_net_info.protocol=="ipc"){    
        ec_cfg.protocol="iddp";
    }
    ec_cfg.host_name=ec_net_info.host_name;
    ec_cfg.host_port=ec_net_info.host_port;
    ec_cfg.period_ms = 4; //4ms fix period 
    ec_cfg.logging = false; 
    
#ifdef TEST_GUI 
    EcUtils::Ptr ec_utils = std::make_shared<EcUtils>();
#else
    EcUtils::Ptr ec_utils = std::make_shared<EcUtils>(ec_cfg);
#endif

    try{
        _ec_wrapper_info.client = ec_utils->make_ec_iface();
        _ec_wrapper_info.client->start_client(ec_cfg.period_ms);
    }
    catch ( std::exception &e ){
        QMessageBox msgBox;
        msgBox.critical(this,msgBox.windowTitle(),tr(e.what()));
        return false;
    }

    return true;
}

void EcGuiStart::ExpertUserPassChanged()
{
    QMessageBox msgBox;
    QString message="export user password incorrect!";
    if(_expert_user->text().toStdString()==expert_user_password){
        message="export user password correct!";
        _ec_gui_wrapper->set_expert_user();
        _firmware_update_btn->setEnabled(true);
        _expert_user->setEnabled(false);
    }
    msgBox.setText(message);
    msgBox.exec();
}

void EcGuiStart::onEcMasterProcessChanged(int index)
{
    QString repl_config="";
    switch (index)
    {
    case 0:
        repl_config="";
        break;
    case 1:
        repl_config="ROBOT_MASTER_CONFIG";
        break;
    case 2:
        repl_config="ECAT_MASTER_CONFIG";
        break;
    
    default:
        repl_config="";
        break;
    }
    _ec_gui_net->set_repl_config(repl_config);
}

void EcGuiStart::disable_ec_system()
{
    _ec_system_status->setText("EtherCAT system not running");
    _ec_system_status->setStyleSheet("background-color : gray;color : black;border :3px solid blue;font: 16pt;");
    _ec_sys_started=false;
    _ec_gui_net->set_protocol_enabled(true);
    _ec_gui_net->set_net_enabled(true);
    _master_process_type->setEnabled(true);
}

void EcGuiStart::enable_ec_system()
{
    _ec_system_status->setText("EtherCAT system running");
    _ec_system_status->setStyleSheet("background-color : green;color : black;border :3px solid blue;font: 16pt;");
    _ec_sys_started=true;
    _ec_gui_net->set_protocol_enabled(false);
    _ec_gui_net->set_net_enabled(false);
    _master_process_type->setEnabled(false);
}

void EcGuiStart::onStartEtherCATSystem()
{
    QMessageBox msgBox;
    if(!_ec_sys_started){

        if(!_ec_gui_net->start_network()){
            return;
        }
        /******************************STAR EtherCAT Master and Server ************************************************/
        // clear client 
        // problem on udp bind netstat -antup | grep 54320 = client port
        // sshpass
        if(_ec_wrapper_info.client){
            _ec_wrapper_info.client->stop_client();
            _ec_wrapper_info.client.reset();
            clear_gui();
        }

        _ec_gui_net->setObjectName("ec_gui_net");
    
        _ec_gui_net->set_protocol_enabled(false);
        enable_ec_system();
        
        msgBox.setText("EtherCAT Master system started");
        
    }
    else{
        msgBox.setText("EtherCAT Master system already started");
    }
    msgBox.exec();
}

void EcGuiStart::stopping_ec_sys()
{
    disable_ec_system();
    _ec_gui_net->stop_network();

    return;
}

void EcGuiStart::onStopEtherCATSystem()
{
    QString msg="";
    if(_ec_sys_started){
        /******************************STOP EtherCAT Master and Server ************************************************/
        if(_ec_gui_net->stop_network()){
            /******************************CLEAN UP THE GUI ************************************************/
            clear_gui();
            msg="EtherCAT Master system stopped";
        }
    }
    else{
        msg="EtherCAT Master system already stopped";
    }
    if(msg!=""){
        QMessageBox msgBox;
        msgBox.setText(msg);
        msgBox.exec();
    }
}

void EcGuiStart::onFirmwareUpdateReleased()
{
    if(_ec_sys_started){

        QMessageBox::StandardButton reply;
        QMessageBox msgBox;
        reply = msgBox.warning(this,msgBox.windowTitle(),tr("Firmware wizard cannot be opened if the EtherCAT System is running state!\n"
                               "Do you want to stop it?"),
                                QMessageBox::Yes|QMessageBox::No);
        if(reply == QMessageBox::Yes) {
            if(!_ec_gui_net->stop_network()){
                return;
            }
        }
        else{
            return;
        }
    }
    
    _ec_sys_start->setEnabled(false);
    _ec_sys_stop->setEnabled(false);
    clear_gui();
    _firmware_update_wizard->run_wizard();
}

void EcGuiStart::onFirmwareUpdateCopyFiles()
{
    _ec_gui_net->copy_files_network(_firmware_update_wizard->get_files_list());
}

void EcGuiStart::onFirmwareUpdateOpenConfig()
{
    _ec_gui_net->open_firmware_config();
}

void EcGuiStart::onFirmwareUpdateStart()
{
    _ec_gui_net->start_firmware_update();
}

void EcGuiStart::onFirmwarewizardClosed(int ret)
{
    _ec_sys_start->setEnabled(true);
    _ec_sys_stop->setEnabled(true);
    _ec_gui_net->stop_firmware_update();
}

void EcGuiStart::restart_gui()
{
    add_device();
    enable_ec_system();
    _ec_gui_wrapper->restart_gui_wrapper(_ec_wrapper_info);
}

void EcGuiStart::error_on_scannig()
{
    disable_ec_system();
    QMessageBox msgBox;
    msgBox.setText("Cannot find EtherCAT devices on network"
                   ", please control the EtherCAT Master status or sever status");
    msgBox.exec();
}

void EcGuiStart::add_device()
{
    auto ec_device_item =_net_tree_wid->topLevelItem(0)->child(2)->child(0);
    for ( auto &[esc_id, type, pos] : _ec_wrapper_info.device_info ){
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
    while(ec_device_item->childCount()>0){
        auto child = ec_device_item->child(0);
        ec_device_item->removeChild(child);
    }
    
    _ec_wrapper_info.device_info.clear();
    _ec_wrapper_info.sdo_map.clear();
}

void EcGuiStart::clear_gui()
{
    disable_ec_system();

    clear_device();
    _ec_gui_wrapper->clear_gui_wrapper();

    EcGuiWrapper::ec_wrapper_info_t _ec_wrapper_info_reset;
    _ec_wrapper_info=_ec_wrapper_info_reset;
}

void EcGuiStart::read_sdo_info(const int32_t device_id,
                               const std::vector<std::string> sdo_name,
                               std::vector<float> &sdo_info)
{
    int i=0;
    for(auto &sdo:sdo_name){
        if(_ec_wrapper_info.sdo_map.count(device_id)>0){
            if(_ec_wrapper_info.sdo_map[device_id].count(sdo)){
                std::stringstream out;
                out << std::fixed << std::setprecision(5) << _ec_wrapper_info.sdo_map[device_id][sdo];
                out >> sdo_info[i];
            }
        }
        i++;
    }
}

void EcGuiStart::setup_motor_device(int32_t device_id,int32_t device_type)
{
    if(ec_motors[device_type]=="Synap_Motor"||
       ec_motors[device_type]=="Nov_Motor"){
        std::vector<std::string> sdo_gains={"Position_loop_Kp","Position_loop_Ki","Position_loop_Kd"};
        _ec_wrapper_info.device_ctrl.device_gains[device_id][0x3B]={0.0,0.0,0.0,0.0,0.0};
        read_sdo_info(device_id,sdo_gains,_ec_wrapper_info.device_ctrl.device_gains[device_id][0x3B]);

        sdo_gains={"Velocity_loop_Kp","Velocity_loop_Ki","Velocity_loop_Kd"};
        if(ec_motors[device_type]=="Synap_Motor"){
            sdo_gains={"Controller_Kp","Controller_Ki","Controller_Kd"};
        }
        _ec_wrapper_info.device_ctrl.device_gains[device_id][0x71]={0.0,0.0,0.0,0.0,0.0};
        read_sdo_info(device_id,sdo_gains,_ec_wrapper_info.device_ctrl.device_gains[device_id][0x71]);
        
        _ec_wrapper_info.device_ctrl.device_gains[device_id][0xD4]={0.0,0.0,0.0,0.0,0.0};
        if(ec_motors[device_type]=="Synap_Motor"){
            sdo_gains={"Torque_Controller_Kp","Torque_Controller_Ki","Torque_Controller_Kd"};
            read_sdo_info(device_id,sdo_gains,_ec_wrapper_info.device_ctrl.device_gains[device_id][0xD4]);
        }

        sdo_gains={"Torque_loop_Kp","Torque_loop_Ki"};
        if(ec_motors[device_type]=="Synap_Motor"){
            sdo_gains={"Damping_ratio","Settling_time"};
        }
        _ec_wrapper_info.device_ctrl.device_gains[device_id][0xCC]={0.0,0.0,0.0,0.0,0.0};
        read_sdo_info(device_id,sdo_gains,_ec_wrapper_info.device_ctrl.device_gains[device_id][0xCC]);

        _ec_wrapper_info.device_ctrl.device_gains[device_id][0xDD]={0.0,0.0,0.0,0.0,0.0};
        if(ec_motors[device_type]=="Nov_Motor"){
            sdo_gains={"Current_quadrature_loop_Kp","Current_quadrature_loop_Ki"};
            read_sdo_info(device_id,sdo_gains,_ec_wrapper_info.device_ctrl.device_gains[device_id][0xDD]);
        }
        _ec_wrapper_info.device_ctrl.device_gains[device_id][0x00]={0.0,0.0,0.0,0.0,0.0};
    }else{
        _ec_wrapper_info.device_ctrl.device_gains[device_id][0x3B]={200,0,10,0,0};
        _ec_wrapper_info.device_ctrl.device_gains[device_id][0x71]={20,0,0,0,0};
        _ec_wrapper_info.device_ctrl.device_gains[device_id][0xD4]={500.0,10.0,1.0,0.007,0.7};
        _ec_wrapper_info.device_ctrl.device_gains[device_id][0xCC]={0.18,0.01,0.0,0.0,0.0};
        _ec_wrapper_info.device_ctrl.device_gains[device_id][0xDD]={0.18,0.01,0.0,0.0,0.0};
        _ec_wrapper_info.device_ctrl.device_gains[device_id][0x00]={0.0,0.0,0.0,0.0,0.0};
    }

    std::vector<std::string> sdo_limits={"Min_pos","Max_pos","Max_vel","Max_tor","Max_ref"};
    _ec_wrapper_info.device_ctrl.device_limits[device_id]={FLT_MIN,FLT_MAX,0.0,0.0,0.0};
    read_sdo_info(device_id,sdo_limits,_ec_wrapper_info.device_ctrl.device_limits[device_id]);        
}

void EcGuiStart::scan_device()
{
    if(_ec_wrapper_info.client->retrieve_slaves_info(_ec_wrapper_info.device_info)){
        if(_ec_wrapper_info.device_info.empty()){
            error_on_scannig();
        }
        else{
            for ( auto &[device_id, device_type, device_pos] : _ec_wrapper_info.device_info ) {
                RR_SDOS rr_sdo;
                if(_ec_wrapper_info.client->retrieve_all_sdo(device_id,rr_sdo)){
                    _ec_wrapper_info.sdo_map[device_id]=rr_sdo;
                }
                if(ec_motors.count(device_type)>0){
#ifdef TEST_GUI 
                    if(_ec_wrapper_info.sdo_map.count(device_id)==0){
                        RR_SDOS motor_sdo={{"Min_pos","-3.0"},
                                           {"Max_pos","2.0"},
                                           {"Max_vel","5.0"},
                                           {"Max_tor","50.0"},
                                           {"Max_ref","5.0"}};
                        _ec_wrapper_info.sdo_map[device_id]=motor_sdo;
                    }
#endif
                    setup_motor_device(device_id,device_type);
                }
            }
        }
    }
    else{
        error_on_scannig();
    }
}

void EcGuiStart::onScanDeviceReleased()
{
    if(!_ec_wrapper_info.device_info.empty()){
        QMessageBox::StandardButton reply;
        QMessageBox msgBox;
        reply = msgBox.warning(this,msgBox.windowTitle(),tr("EtherCAT device(s) already scanned.\n"
                               "Do you want to rescan?"),
                                QMessageBox::Yes|QMessageBox::No);
        if(reply == QMessageBox::Yes) {
            if(!_ec_gui_wrapper->get_wrapper_send_sts() && 
               !_ec_gui_wrapper->get_wrapper_cmd_sts()){
                clear_gui();
                if(create_ec_iface()){
                    scan_device();
                }
            }
            else{
                msgBox.critical(this,msgBox.windowTitle(),
                                tr("Cannot rescan EtherCAT device(s) already started or controlled.\n"));
                return;
            }
        }
        else{
            return;
        }
    }
    else{
        if(create_ec_iface()){
            scan_device();
        }
    }
    
    if(!_ec_wrapper_info.device_info.empty()){
        restart_gui();
    } 
}

EcGuiStart::~EcGuiStart()
{
}
