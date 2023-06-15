#include "ec_gui_start.h"
#include "ec_utils.h"

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
#define HOSTNAME_COL 1
#define HOSTIP_COL 2
#define HOSTPORT_COL 3

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
    _net_tree_wid->installEventFilter(this);
    connect(_net_tree_wid, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)),this, SLOT(OnMouseDoubleClicked(QTreeWidgetItem*, int)));
    
    _net_item = nullptr;
    _net_column=-1;
    
    _server_hostname=_net_tree_wid->topLevelItem(0)->child(1)->text(1);
    
    if(_net_tree_wid->topLevelItem(0)->child(1)->text(2)=="localhost")
    {
        _server_ip="127.0.0.1";
    }
    else
    {
        _server_ip=_net_tree_wid->topLevelItem(0)->child(1)->text(2);
    }
    _server_port=_net_tree_wid->topLevelItem(0)->child(1)->text(3);
    
    connect(_net_tree_wid, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)),this, SLOT(OnMouseClicked(QTreeWidgetItem*, int)));

    
    auto ec_sys_start = findChild<QPushButton *>("StartEthercatSystem");
    connect(ec_sys_start, &QPushButton::released,
            this, &EcGuiStart::onStartEtherCATSystem);
    
    auto ec_sys_stop = findChild<QPushButton *>("StopEthercatSystem");
    connect(ec_sys_stop, &QPushButton::released,
            this, &EcGuiStart::onStopEtherCATSystem);
    
    auto scan_device = findChild<QPushButton *>("ScanDevice");
    connect(scan_device, &QPushButton::released,
            this, &EcGuiStart::onScanDeviceReleased);
    
    _ec_master_process = new QProcess(this);
    _ec_master_process->setReadChannel(QProcess::StandardOutput);
    _ec_master_process->setProcessChannelMode(QProcess::MergedChannels);
    _ec_master_process->setCurrentReadChannel(QProcess::StandardOutput);
    
    connect(_ec_master_process, &QProcess::readyReadStandardOutput,
            this, &EcGuiStart::on_ec_process_readyReadStandardOutput);
    
    _server_process = new QProcess(this);
    _server_process->setReadChannel(QProcess::StandardOutput);
    _server_process->setProcessChannelMode(QProcess::MergedChannels);
    _server_process->setCurrentReadChannel(QProcess::StandardOutput);
    
    connect(_server_process, &QProcess::readyReadStandardOutput,
            this, &EcGuiStart::on_server_process_readyReadStandardOutput);
    
    /*protocl */

    _protocol_combobox = findChild<QComboBox *>("Protocol");
    _protocol_combobox->setCurrentIndex(1); // set Default UDP.
    _server_protocol = _protocol_combobox->currentText();
    
    /* connection of frequency function */
    connect(_protocol_combobox, SIGNAL(currentIndexChanged(int)),this,
        SLOT(OnProtocolChanged())
    );
    
    _ec_gui_wrapper = std::make_shared<EcGuiWrapper>(this);
    
    _ec_master_terminal=std::make_shared<EcGuiTerminal>();
    _ec_master_terminal->setWindowTitle("EtherCAT Master terminal");
    _ec_master_terminal->setAttribute( Qt::WA_QuitOnClose, false );
    
    _server_terminal=std::make_shared<EcGuiTerminal>();
    _server_terminal->setWindowTitle("Server terminal");
    _server_terminal->setAttribute( Qt::WA_QuitOnClose, false );
    
    _etherCAT_sys_started=false;
}

void EcGuiStart::on_ec_process_readyReadStandardOutput()
{
   _ec_master_stoud.clear();
   while(_ec_master_process->canReadLine()){
       _ec_master_stoud = _ec_master_process->readLine();
       _ec_master_terminal->setText(_ec_master_stoud);
  }
}

void EcGuiStart::on_server_process_readyReadStandardOutput()
{
   _server_stdout.clear();
   while(_server_process->canReadLine()){
       _server_stdout = _server_process->readLine();
       _server_terminal->setText(_server_stdout);
  }
}

void EcGuiStart::OnMouseDoubleClicked(QTreeWidgetItem* item, int column)
{
    if(column == 4 && item->text(0)=="Server")
    {
        _server_terminal->show();
        _server_terminal->setWindowState(Qt::WindowActive);
    }
    else if(column == 4 && item->text(0)=="EtherCAT Master")
    {
        _ec_master_terminal->show();
        _ec_master_terminal->setWindowState(Qt::WindowActive);
    }
}

void EcGuiStart::OnProtocolChanged()
{
    _server_protocol = _protocol_combobox->currentText();
}

void EcGuiStart::create_ec_iface()
{
    if(_ec_wrapper_info.client)
    {
        if(_ec_wrapper_info.client->is_client_alive())
        {
            _ec_wrapper_info.client->stop_client();
        }
    }
    
    EcUtils::EC_CONFIG ec_cfg;
    
    ec_cfg.protocol=_server_protocol.toStdString();
    ec_cfg.host_name=_server_ip.toStdString();
    ec_cfg.host_port=_server_port.toUInt();
    ec_cfg.period_ms = _ec_gui_wrapper->get_period_ms();
    ec_cfg.logging = false;
    
    EcUtils::Ptr ec_utils = std::make_shared<EcUtils>(ec_cfg);
    
    _ec_wrapper_info.client.reset();
    try{
        _ec_wrapper_info.client = ec_utils->make_ec_iface();
    }
    catch ( std::exception &e )
    {
        QMessageBox msgBox;
        msgBox.critical(this,msgBox.windowTitle(),tr(e.what()));
    }
}

void EcGuiStart::set_ec_network()
{
    _server_hostname=_net_tree_wid->topLevelItem(0)->child(1)->text(1);
    _net_tree_wid->topLevelItem(0)->child(2)->setText(HOSTNAME_COL,_server_hostname);
    
    _net_tree_wid->topLevelItem(0)->child(2)->setText(HOSTIP_COL,_net_tree_wid->topLevelItem(0)->child(1)->text(2));
    if(_net_tree_wid->topLevelItem(0)->child(1)->text(2)=="localhost")
    {
        _server_ip="127.0.0.1";
    }
    else
    {
        _server_ip=_net_tree_wid->topLevelItem(0)->child(1)->text(2);
    }
    
    _server_port=_net_tree_wid->topLevelItem(0)->child(1)->text(3);
    _net_tree_wid->topLevelItem(0)->child(0)->setText(HOSTPORT_COL,_server_port);
    
    _net_tree_wid->closePersistentEditor(_net_item,_net_column); // close old editor
    _net_item = nullptr;
    _net_column=-1;
}

bool EcGuiStart::eventFilter( QObject* o, QEvent* e )
{
    if( o == _net_tree_wid && e->type() == QEvent::KeyRelease)
    {
        QKeyEvent *qkey = static_cast<QKeyEvent*>(e);
        if(qkey->key() == Qt::Key_Return)
        {
            set_ec_network();
        }
    }
    return false;
}

void EcGuiStart::OnMouseClicked(QTreeWidgetItem* item, int column)
{
    if(_net_item != nullptr)
    {
        _net_tree_wid->closePersistentEditor(_net_item,_net_column); // close old editor
    }

    _net_item = item;
    _net_column=column;

    if((item->text(0)=="Server") &&
       ((column == HOSTNAME_COL) || 
        (column == HOSTIP_COL)   ||
        (column == HOSTPORT_COL)))    
    {
        _net_tree_wid->openPersistentEditor(item,column);
    }
    else
    {
        _net_tree_wid->closePersistentEditor(item,column);
    } 
}

QString EcGuiStart::find_running_process(QProcess * process,QString bin_name,QString& stdout)
{
    QStringList cmd;
    QString bin_file;
    
    cmd = _ssh_command;
    cmd.append("'pgrep'"); // remember comment out: .bashrc all line of # If not running interactively, don't do anything
    cmd.append(bin_name);
    
    stdout.clear();
    process->start("sshpass", cmd);
    if(process->waitForFinished())
    {
       bin_file = stdout;
    }

    return bin_file;
}


QString EcGuiStart::find_process(QProcess * process,QString bin_name,QString& stdout)
{
    QStringList cmd;
    QString bin_file;
    
    cmd = _ssh_command;
    cmd.append("'which'"); // remember comment out: .bashrc all line of # If not running interactively, don't do anything
    cmd.append(bin_name);
    
    stdout.clear();
    process->start("sshpass", cmd);
    if(process->waitForFinished())
    {
       bin_file = stdout;
    }

    return bin_file;
}

void EcGuiStart::kill_process(QProcess *process,QString bin_name,QString& stdout)
{
    QString pid=find_running_process(process,bin_name,stdout);
    if(pid!="")
    {
        QStringList cmd;
        
        cmd=_ssh_command;
        cmd.append("'killall'");
        cmd.append(bin_name);
        
        process->start("sshpass", cmd);
        process->waitForFinished();
    }
}

void EcGuiStart::start_process(QProcess *process,QString bin_file_path)
{
    QStringList cmd;
    
    cmd=_ssh_command;    
    cmd.append(bin_file_path);
    process->start("sshpass", cmd);
}

void EcGuiStart::create_ssh_cmd()
{
    _ssh_command.clear();
    _ssh_command.append("-p");
    _ssh_command.append("user");
    _ssh_command.append("ssh");
    _ssh_command.append("user@127.0.01");
}

void EcGuiStart::onStartEtherCATSystem()
{
    
    if(!_etherCAT_sys_started)
    {
        create_ssh_cmd();
    
        QString bin_file_name = "'repl'";
        kill_process(_ec_master_process,bin_file_name,_ec_master_stoud);
        auto bin_file_path = find_process(_ec_master_process,bin_file_name,_ec_master_stoud);
        if(!bin_file_path.isEmpty())
        {
            start_process(_ec_master_process,bin_file_path);
        }
        
        bin_file_name = "'udp_server'";
        kill_process(_server_process,bin_file_name,_server_stdout);
        
        bin_file_path.clear();
        bin_file_path = find_process(_server_process,bin_file_name,_server_stdout);

        if(!bin_file_path.isEmpty())
        {
            start_process(_server_process,bin_file_path);
        }
        
        sleep(1);
        create_ec_iface();
        
        _etherCAT_sys_started=true;
    }
}

void EcGuiStart::onStopEtherCATSystem()
{
    if(_etherCAT_sys_started)
    {
        create_ssh_cmd();
        
        if(_ec_wrapper_info.client)
        {
            if(_ec_wrapper_info.client->is_client_alive())
            {
                _ec_wrapper_info.client->stop_client();
            }
            _ec_wrapper_info.client.reset();
        }
        
        QString bin_file_name = "'udp_server'";
        _server_process->close();
        kill_process(_server_process,bin_file_name,_server_stdout);
        
        _ec_master_process->close();
        bin_file_name = "'repl'";
        kill_process(_ec_master_process,bin_file_name,_ec_master_stoud);
        
        clear_device();
        EcGuiWrapper::ec_wrapper_info_t _ec_wrapper_info_reset;
        _ec_wrapper_info=_ec_wrapper_info_reset;
        
        restart_gui();
        
        _etherCAT_sys_started=false;
    }
}

void EcGuiStart::restart_gui()
{
    add_device();
    _ec_gui_wrapper->restart_gui_wrapper(_ec_wrapper_info);
}


void EcGuiStart::try_gui()
{
// ********************* TEST ****************************////

    _ec_wrapper_info.device_info.push_back(std::make_tuple(201,POW_F28M36_BOARD,0));   // create power board
    
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
        _ec_wrapper_info.device_info.push_back(std::make_tuple(i,CENT_AC,i));   // create motor board
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
    _ec_wrapper_info.device_info.push_back(std::make_tuple(100,FT6,21));   // create ft6 board
    _ec_wrapper_info.internal_ft6_status_map[100]={10.0,5.0,2.0,100.0,125.0,130.0};
    _ec_wrapper_info.device_info.push_back(std::make_tuple(101,FT6,22));   // create ft6 board
    _ec_wrapper_info.internal_ft6_status_map[101]={10.0,5.0,2.0,100.0,125.0,130.0};
    _ec_wrapper_info.device_info.push_back(std::make_tuple(102,IMU_ANY,23));   // create imu board
    _ec_wrapper_info.internal_imu_status_map[102]={5.0,15.0,20.0,5.0,2.0,3.0,0,0,0,1};
    _ec_wrapper_info.device_info.push_back(std::make_tuple(103,IMU_ANY,24));   // create imu board
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
                if(type==CENT_AC || type==LO_PWR_DC_MC)
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
    if(!_ec_wrapper_info.client)
    {
        create_ssh_cmd();
        QString pid=find_running_process(_server_process,"'udp_server'",_server_stdout);
        if(pid!="")
        {
            create_ec_iface();
        }
        else
        {
            QMessageBox msgBox;
            msgBox.critical(this,msgBox.windowTitle(),tr("Cannot find the server in running mode, please start it and retry!"));
            return;
        }
    }
    
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
    

    if(_ec_wrapper_info.device_info.empty())
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

EcGuiStart::~EcGuiStart()
{
    onStopEtherCATSystem();
    
    _server_process->kill();
    _ec_master_process->kill();
}
