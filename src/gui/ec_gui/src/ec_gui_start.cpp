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
    
    auto measurement_setup_dw= findChild<QDockWidget *>("MeasurementSetup"); 
    measurement_setup_dw->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable);
    
    _net_tree_wid = findChild<QTreeWidget *>("NetworkSetup");
    _net_tree_wid->resizeColumnToContents(0);
    _net_tree_wid->expandAll();
    
    auto ec_sys_start = findChild<QPushButton *>("StartEthercatSystem");
    connect(ec_sys_start, &QPushButton::released,this, &EcGuiStart::onStartEtherCATSystem);
    
    auto ec_sys_stop = findChild<QPushButton *>("StopEthercatSystem");
    connect(ec_sys_stop, &QPushButton::released,this, &EcGuiStart::onStopEtherCATSystem);
    
    auto scan_device = findChild<QPushButton *>("ScanDevice");
    connect(scan_device, &QPushButton::released,this, &EcGuiStart::onScanDeviceReleased);
    
    _ec_gui_net = std::make_shared<EcGuiNet>(this);
    _ec_gui_wrapper = std::make_shared<EcGuiWrapper>(this);
    
    _etherCAT_sys_started=false;
}

void EcGuiStart::create_ec_iface()
{
    try{
        if(_ec_wrapper_info.client){
            _ec_wrapper_info.client->stop_client();
        }
        _ec_wrapper_info.client.reset();
        std::this_thread::sleep_for(100ms);
    }catch ( std::exception &e ){
        QMessageBox msgBox;
        msgBox.critical(this,msgBox.windowTitle(),tr(e.what()));
    }
    
    auto ec_net_info = _ec_gui_net->get_net_setup();

    EcUtils::EC_CONFIG ec_cfg;    
    ec_cfg.protocol=ec_net_info.protocol;
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
    }
}

void EcGuiStart::onStartEtherCATSystem()
{
    QMessageBox msgBox;
    if(!_etherCAT_sys_started){
        /******************************STAR EtherCAT Master and Server ************************************************/
        if(!_ec_gui_net->start_network()){
            return;
        }
        _ec_gui_net->setObjectName("ec_gui_net");
    
        _etherCAT_sys_started=true;
        
        msgBox.setText("EtherCAT Master system started");
        
    }
    else{
        msgBox.setText("EtherCAT Master system already started");
    }
    msgBox.exec();
}

void EcGuiStart::stopping_client()
{
    if(_ec_wrapper_info.client){
        _ec_wrapper_info.client->stop_client();
        _ec_wrapper_info.client.reset();
    }
}

bool EcGuiStart::stopping_ec_sys()
{
    bool etherCAT_sys_stopped=false;
    if(_etherCAT_sys_started){
        //stopping_client();
        /******************************STOP EtherCAT Master and Server ************************************************/
        _ec_gui_net->stop_network();
        /******************************CLEAN UP THE GUI ************************************************/
        clear_device();
        EcGuiWrapper::ec_wrapper_info_t _ec_wrapper_info_reset;
        _ec_wrapper_info=_ec_wrapper_info_reset;
        
        restart_gui();

        _etherCAT_sys_started=false;
        etherCAT_sys_stopped=true;
    }
    
    return etherCAT_sys_stopped;
}

void EcGuiStart::onStopEtherCATSystem()
{
    QMessageBox msgBox;
    if(stopping_ec_sys()){
        msgBox.setText("EtherCAT Master system stopped");
    }
    else{
        msgBox.setText("EtherCAT Master system already stopped");
    }
    msgBox.exec();
}

void EcGuiStart::restart_gui()
{
    add_device();
    _ec_gui_wrapper->restart_gui_wrapper(_ec_wrapper_info);
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
    clear_device();
    _ec_gui_wrapper->clear_gui_wrapper();
}

void EcGuiStart::read_sdo_info(const int32_t device_id,
                               const std::vector<std::string> sdo_name,
                               std::vector<float> &sdo_info)
{
    int i=0;
    for(auto &sdo:sdo_name){
        if(_ec_wrapper_info.sdo_map[device_id].count(sdo)){
            sdo_info[i] = std::stof(sdo);
        }
    }
}

void EcGuiStart::setup_motor_device(int32_t device_id,int32_t device_type)
{
    std::vector<std::string> sdo_gains;
    std::vector<std::string> sdo_limits;
    if(ec_motors[device_type]=="Synapticon_Motor"){

        sdo_gains={"Position_loop_Kp","Position_loop_Ki","Position_loop_Kd",
                    "Velocity_loop_Kp","Velocity_loop_Ki"};
        
        _ec_wrapper_info.device_ctrl.device_gains[device_id][0x3B]={0.0,0.0,0.0,0.0,0.0};
        read_sdo_info(device_id,sdo_gains,_ec_wrapper_info.device_ctrl.device_gains[device_id][0x3B]);

        sdo_gains.clear();
        sdo_gains={"Controller_Kp","Controller_Ki","Controller_Kd"};
        _ec_wrapper_info.device_ctrl.device_gains[device_id][0x71]={0.0,0.0,0.0,0.0,0.0};
        read_sdo_info(device_id,sdo_gains,_ec_wrapper_info.device_ctrl.device_gains[device_id][0x71]);

        sdo_gains.clear();
        sdo_gains={"Torque_Controller_Kp","Torque_Controller_Ki","Torque_Controller_Kd"};
        _ec_wrapper_info.device_ctrl.device_gains[device_id][0xD4]={0.0,0.0,0.0,0.0,0.0};
        read_sdo_info(device_id,sdo_gains,_ec_wrapper_info.device_ctrl.device_gains[device_id][0xD4]);

        sdo_gains.clear();
        sdo_gains={"Damping_ratio","Settling_time"};
        _ec_wrapper_info.device_ctrl.device_gains[device_id][0xCC]={0.0,0.0,0.0,0.0,0.0};
        read_sdo_info(device_id,sdo_gains,_ec_wrapper_info.device_ctrl.device_gains[device_id][0xCC]);

        _ec_wrapper_info.device_ctrl.device_gains[device_id][0xDD]={0.0,0.0,0.0,0.0,0.0};
        _ec_wrapper_info.device_ctrl.device_gains[device_id][0x00]={0.0,0.0,0.0,0.0,0.0};
    }else{
        _ec_wrapper_info.device_ctrl.device_gains[device_id][0x3B]={200,0,10,0,0};
        _ec_wrapper_info.device_ctrl.device_gains[device_id][0x71]={20,0,0,0,0};
        _ec_wrapper_info.device_ctrl.device_gains[device_id][0xD4]={500.0,10.0,1.0,0.7,0.007};
        _ec_wrapper_info.device_ctrl.device_gains[device_id][0xCC]={0.18,0.01,0.0,0.0,0.0};
        _ec_wrapper_info.device_ctrl.device_gains[device_id][0xDD]={0.18,0.01,0.0,0.0,0.0};
        _ec_wrapper_info.device_ctrl.device_gains[device_id][0x00]={0.0,0.0,0.0,0.0,0.0};

        sdo_limits={"Min_pos","Max_pos","Max_vel","Max_tor","Max_cur"};
        _ec_wrapper_info.device_ctrl.device_limits[device_id]={FLT_MIN,FLT_MAX,0.0,0.0,0.0};
        read_sdo_info(device_id,sdo_limits,_ec_wrapper_info.device_ctrl.device_limits[device_id]);        
    }
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
                create_ec_iface();
                scan_device();
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
        create_ec_iface();
        scan_device();
    }
    

    if(!_ec_wrapper_info.device_info.empty()){
        restart_gui();
    } 
}

EcGuiStart::~EcGuiStart()
{

}
