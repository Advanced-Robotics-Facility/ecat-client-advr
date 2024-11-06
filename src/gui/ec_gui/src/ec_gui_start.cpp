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
    
    
    auto ec_sys_start = findChild<QPushButton *>("StartEthercatSystem");
    connect(ec_sys_start, &QPushButton::released,
            this, &EcGuiStart::onStartEtherCATSystem);
    
    auto ec_sys_stop = findChild<QPushButton *>("StopEthercatSystem");
    connect(ec_sys_stop, &QPushButton::released,
            this, &EcGuiStart::onStopEtherCATSystem);
    
    auto scan_device = findChild<QPushButton *>("ScanDevice");
    connect(scan_device, &QPushButton::released,
            this, &EcGuiStart::onScanDeviceReleased);
    
    
    _ec_gui_net = std::make_shared<EcGuiNet>(this);
    _ec_gui_wrapper = std::make_shared<EcGuiWrapper>(this);
    

    _etherCAT_sys_started=false;
}




void EcGuiStart::create_ec_iface()
{
    if(_ec_gui_net->check_network()){
        if(_ec_wrapper_info.client){
            _ec_wrapper_info.client->stop_client();
        }
        
        auto ec_net_info = _ec_gui_net->get_net_setup();
    
        EcUtils::EC_CONFIG ec_cfg;    
        ec_cfg.protocol=ec_net_info.protocol;
        ec_cfg.host_name=ec_net_info.host_name;
        ec_cfg.host_port=ec_net_info.host_port;
        ec_cfg.period_ms = _ec_gui_wrapper->get_period_ms();
        ec_cfg.logging = false; 
        
#ifdef TEST_GUI 
        EcUtils::Ptr ec_utils = std::make_shared<EcUtils>();
#else
        EcUtils::Ptr ec_utils = std::make_shared<EcUtils>(ec_cfg);
#endif

        
        _ec_wrapper_info.client.reset();
        try{
            _ec_wrapper_info.client = ec_utils->make_ec_iface();
            _ec_wrapper_info.client->start_client(ec_cfg.period_ms);
        }
        catch ( std::exception &e ){
            QMessageBox msgBox;
            msgBox.critical(this,msgBox.windowTitle(),tr(e.what()));
        }
    }
    else
    {
        QMessageBox msgBox;
        msgBox.critical(this,msgBox.windowTitle(),tr("Cannot find the server in running mode, please start it and retry!"));
        return;
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
        _ec_gui_net->start();
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
        if(_ec_gui_net->isRunning()){
            _ec_gui_net->terminate();
            _ec_gui_net->wait();
        }
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
    if(_ec_wrapper_info.client != nullptr){ // the restart-gui function can be used for cleaning the entire gui without reading the pdo.
        _ec_gui_wrapper->start_receive(); // auto-start of receiving
    }
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

void EcGuiStart::scan_device()
{
    if(_ec_wrapper_info.client->retrieve_slaves_info(_ec_wrapper_info.device_info)){
        if(_ec_wrapper_info.device_info.empty()){
            error_on_scannig();
        }
        else{
            for ( auto &[device_id, device_type, device_pos] : _ec_wrapper_info.device_info ) {
                RR_SDOS rr_sdo;
                _ec_wrapper_info.client->retrieve_all_sdo(device_id,rr_sdo);
                _ec_wrapper_info.sdo_map[device_id]=rr_sdo;
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
            if(!_ec_gui_wrapper->get_wrapper_send_sts()){
                clear_device();
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
   if(!stopping_ec_sys()){
       stopping_client();
   }
}
