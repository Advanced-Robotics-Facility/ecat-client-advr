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
            if(_ec_wrapper_info.client->is_client_alive()){
                _ec_wrapper_info.client->stop_client();
            }
        }
    
        auto ec_net_info = _ec_gui_net->get_net_setup();
        
        EcUtils::EC_CONFIG ec_cfg;
        
        ec_cfg.protocol=ec_net_info.protocol;
        ec_cfg.host_name=ec_net_info.host_name;
        ec_cfg.host_port=ec_net_info.host_port;
        ec_cfg.period_ms = _ec_gui_wrapper->get_period_ms();
        ec_cfg.logging = false;
        
        EcUtils::Ptr ec_utils = std::make_shared<EcUtils>(ec_cfg);
        
        _ec_wrapper_info.client.reset();
        try{
            _ec_wrapper_info.client = ec_utils->make_ec_iface();
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
    if(!_etherCAT_sys_started)
    {
        /******************************STAR EtherCAT Master and Server ************************************************/
        if(!_ec_gui_net->start_network())
        {
            return;
        }
        /******************************STAR CLIENT ************************************************/
        sleep(1);
        create_ec_iface();
        _etherCAT_sys_started=true;
        
        msgBox.setText("EtherCAT Master system started");
        
    }
    else
    {
        msgBox.setText("EtherCAT Master system already started");
    }
    msgBox.exec();
}

void EcGuiStart::stopping_client()
{
    if(_ec_wrapper_info.client)
    {
        if(_ec_wrapper_info.client->is_client_alive())
        {
            _ec_wrapper_info.client->stop_client();
        }
        _ec_wrapper_info.client.reset();
    }
}
bool EcGuiStart::stopping_ec_sys()
{
    bool etherCAT_sys_stopped=false;
    if(_etherCAT_sys_started)
    {
        stopping_client();
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
    if(stopping_ec_sys())
    {
        msgBox.setText("EtherCAT Master system stopped");
    }
    else
    {
        msgBox.setText("EtherCAT Master system already stopped");
    }
    msgBox.exec();
}

void EcGuiStart::restart_gui()
{
    add_device();
    _ec_gui_wrapper->restart_gui_wrapper(_ec_wrapper_info);
}


void EcGuiStart::try_gui()
{
// ********************* TEST ****************************////

    _ec_wrapper_info.device_info.push_back(std::make_tuple(201,iit::ecat::POW_F28M36_BOARD,0));   // create power board
    
    _ec_wrapper_info.internal_pow_status_map[201]={48.0,48.0,2.0,25.0,25.0,25.0};
    
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
        _ec_wrapper_info.device_info.push_back(std::make_tuple(i,iit::ecat::CENT_AC,i));   // create motor board
        EcGuiSlider::joint_info_t joint_info_s;

        joint_info_s.joint_name    ="motor_id_"+std::to_string(i);
        joint_info_s.actual_pos    =0.0;
        joint_info_s.min_pos       =-1.0;
        joint_info_s.max_pos       =1.0;
        joint_info_s.actual_vel    =0.0;
        joint_info_s.max_vel       =2.0;
        joint_info_s.actual_torq   =0.0;
        joint_info_s.max_torq      =5.0;

        _ec_wrapper_info.joint_info_map[i]=joint_info_s;
        _ec_wrapper_info.internal_motor_status_map[i] = std::make_tuple(10,10,0,0,100,25,25,0,0,0,0,0);
        _ec_wrapper_info.internal_sdo_map[i] = rr_sdo_info_motor;
    }
    _ec_wrapper_info.device_info.push_back(std::make_tuple(100,iit::ecat::FT6,21));   // create ft6 board
    _ec_wrapper_info.internal_ft6_status_map[100]={10.0,5.0,2.0,100.0,125.0,130.0};
    _ec_wrapper_info.device_info.push_back(std::make_tuple(101,iit::ecat::FT6,22));   // create ft6 board
    _ec_wrapper_info.internal_ft6_status_map[101]={10.0,5.0,2.0,100.0,125.0,130.0};
    _ec_wrapper_info.device_info.push_back(std::make_tuple(102,iit::ecat::IMU_ANY,23));   // create imu board
    _ec_wrapper_info.internal_imu_status_map[102]={5.0,15.0,20.0,5.0,2.0,3.0,0,0,0,1};
    _ec_wrapper_info.device_info.push_back(std::make_tuple(103,iit::ecat::IMU_ANY,24));   // create imu board
    _ec_wrapper_info.internal_imu_status_map[103]={5.0,15.0,20.0,5.0,2.0,3.0,0,0,0,1};

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
    for ( auto &[esc_id, type, pos] : _ec_wrapper_info.device_info )
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
    
    _ec_wrapper_info.device_info.clear();
    _ec_wrapper_info.joint_info_map.clear();
    
    _ec_wrapper_info.internal_motor_status_map.clear();
    _ec_wrapper_info.internal_ft6_status_map.clear();
    _ec_wrapper_info.internal_pow_status_map.clear();
    _ec_wrapper_info.internal_imu_status_map.clear();
    
    _ec_wrapper_info.sdo_map.clear();
    _ec_wrapper_info.internal_sdo_map.clear();
}

void EcGuiStart::scan_device()
{
    if(_ec_wrapper_info.client->retrieve_slaves_info(_ec_wrapper_info.device_info))
    {
        if(_ec_wrapper_info.device_info.empty())
        {
            error_on_scannig();
        }
        else
        {

            // *************** END AUTODETECTION *************** //

            // GET Mechanical Limits
            _ec_wrapper_info.joint_info_map.clear();
            std::map<int,RR_SDO> motor_info_map;
            RD_SDO rd_sdo = { "motor_pos","Min_pos","Max_pos","motor_vel","Max_vel","torque","Max_tor"};
            WR_SDO wr_sdo = {};
            int motors_counter=0;
            
            for ( auto &[esc_id, type, pos] : _ec_wrapper_info.device_info )
            {
                RR_SDO rr_sdo_info;
                if(ec_motors.count(type))
                {
                    motors_counter++;
                    if(_ec_wrapper_info.client->retrieve_rr_sdo(esc_id,rd_sdo,wr_sdo,rr_sdo_info))
                    {    
                        if(!rr_sdo_info.empty())
                        {
                            motor_info_map[esc_id]=rr_sdo_info;
                        }
                    }
                }
                /********************* RETRIEVE ALL SDO */////////////
                rr_sdo_info.clear();
                _ec_wrapper_info.client->retrieve_all_sdo(esc_id,rr_sdo_info);
                _ec_wrapper_info.sdo_map[esc_id] = rr_sdo_info;
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
                for ( auto &[slave_id, type, pos] : _ec_wrapper_info.device_info )
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


                        _ec_wrapper_info.joint_info_map[slave_id]=joint_info_s;
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
    if(!_ec_wrapper_info.device_info.empty())
    {
        QMessageBox::StandardButton reply;
        QMessageBox msgBox;
        reply = msgBox.warning(this,msgBox.windowTitle(),tr("EtherCAT device(s) already scanned.\n"
                               "Do you want to rescan?"),
                                QMessageBox::Yes|QMessageBox::No);
        if(reply == QMessageBox::Yes)
        {
            if(!_ec_gui_wrapper->get_wrapper_send_sts())
            {
                clear_device();
                create_ec_iface();
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
        create_ec_iface();
        scan_device();
    }
    

    if(_ec_wrapper_info.device_info.empty())
    {
#ifdef TEST_GUI 
        try_gui();
        restart_gui();
#endif
    }
    else
    {
        restart_gui();
    } 
}

EcGuiStart::~EcGuiStart()
{
   if(!stopping_ec_sys())
   {
       stopping_client();
   }
}
