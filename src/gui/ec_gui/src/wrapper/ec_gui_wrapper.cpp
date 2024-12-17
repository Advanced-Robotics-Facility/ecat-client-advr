#include "ec_gui_wrapper.h"

#include <iostream>
#include <csignal>
#include <atomic>

#include <QLabel>
#include <QPixmap>
#include <QFile>

#include <chrono>
using namespace std::chrono;

EcGuiWrapper::EcGuiWrapper(QWidget *parent) :
    QWidget(parent)
{

    _command_dw = parent->findChild<QDockWidget *>("Command");
    connect(_command_dw, SIGNAL(topLevelChanged(bool)), this, SLOT(DwTopLevelChanged(bool)));
    _command_dw->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable);
    
    _pdo_sdo_dw = parent->findChild<QDockWidget *>("DataObject");
    connect(_pdo_sdo_dw, SIGNAL(topLevelChanged(bool)), this, SLOT(DwTopLevelChanged(bool)));
    _pdo_sdo_dw->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable);
    
    _graphics_dw = parent->findChild<QDockWidget *>("Graphics");
    connect(_graphics_dw, SIGNAL(topLevelChanged(bool)), this, SLOT(DwTopLevelChanged(bool)));
    _graphics_dw->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable);

    _receive_action = parent->findChild<QAction *>("actionReceive");
    connect(_receive_action, SIGNAL(triggered()), this, SLOT(start_stop_receive()));
    
    _record_action = parent->findChild<QAction *>("actionRecord");
    connect(_record_action, SIGNAL(triggered()), this, SLOT(start_stop_record()));
    
    _receive_started = _record_started = false;
    
    _ec_gui_slider = std::make_shared<EcGuiSlider>(parent);
    
    _ec_gui_pdo = std::make_shared<EcGuiPdo>(_ec_gui_slider,
                                             parent);
    
    _ec_gui_sdo = std::make_shared<EcGuiSdo>(parent);

    _ec_gui_cmd = std::make_shared<EcGuiCmd>(_ec_gui_slider,
                                             parent);

    _ec_logger=std::make_shared<EcLogger>();

    _send_pdo=false;
        
    // Get Send and Stop button
    _send_stop_btn = parent->findChild<QPushButton *>("SendStopBtn");

    connect(_send_stop_btn, &QPushButton::released,this, &EcGuiWrapper::onSendStopBtnReleased);
    
    // create a timer for sending PDO
    _send_timer = new QTimer(this);

    // setup signal and slot
    connect(_send_timer, SIGNAL(timeout()),this, SLOT(send()));


    // create a timer for receiving PDO
    _receive_timer = new QTimer(this);

    // setup signal and slot
    connect(_receive_timer, SIGNAL(timeout()),this, SLOT(receive()));

    _time_ms = 5;
}

void EcGuiWrapper::DwTopLevelChanged(bool isFloating)
{
    auto dw = qobject_cast<QDockWidget*>(sender());
    if(isFloating){
        dw->setWindowFlags(Qt::Window);
        dw->show();
    }
}

bool EcGuiWrapper::get_wrapper_send_sts()
{
    return _send_pdo;
}

void EcGuiWrapper::restart_gui_wrapper(ec_wrapper_info_t ec_wrapper_info)
{

    stop_receive();
    stop_record();
    
    _ec_wrapper_info = ec_wrapper_info;

    _ec_logger->init_mat_logger(_ec_wrapper_info.device_info);
    
    _ec_gui_slider->create_sliders(_ec_wrapper_info.device_info);
    
    _ec_gui_cmd->restart_ec_gui_cmd(_ec_wrapper_info.client);
    
    _ec_gui_pdo->restart_ec_gui_pdo(_ec_wrapper_info.client,_ec_logger);
    
    _ec_gui_sdo->restart_ec_gui_sdo(_ec_wrapper_info.client,_ec_wrapper_info.sdo_map);
}

bool EcGuiWrapper::check_client_setup()
{
    bool ret=false;
    if(_ec_wrapper_info.client == nullptr){
        QMessageBox msgBox;
        msgBox.critical(this,msgBox.windowTitle(),
                        tr("EtherCAT client not setup"
                           ",please press Start EtherCAT system button.\n"));
    }
    else{
        ret = true;
    }
    
    return ret;
}

void EcGuiWrapper::onSendStopBtnReleased()
{
    // @NOTE to be tested.
    _ec_gui_slider->reset_sliders();

    _send_pdo = _ec_gui_cmd->get_command_sts(); // devices controlled
    count_reset_ref=0;
    
    if((_send_stop_btn->text()=="Start Motion")&&(_send_pdo)){
        _send_stop_btn->setText("Stop Motion");
        _ec_gui_slider->enable_sliders();

        _ec_gui_pdo->set_filter(_time_ms);
        _ec_gui_pdo->restart_send_timer();
        _send_timer->start(_time_ms);
    }
    else{
        _send_pdo=false;      
        _ec_gui_slider->disable_sliders();

        if(_send_stop_btn->text()=="Start Motion"){
            QMessageBox msgBox;
            msgBox.setText("Cannot start motion without starting the devices"
                           ", please launch START EtherCAT command ");
            msgBox.exec();
        }
        else{
            _send_stop_btn->setText("Start Motion");
        }
    }
}

void EcGuiWrapper::send()
{

    // **************Delay stop**************
    if(!_send_pdo){
        count_reset_ref++;
        _ec_gui_pdo->set_filter(_time_ms);//STOP align all references to zero or with the actual position for the motors
        _ec_gui_pdo->restart_send_timer();
        if(count_reset_ref>3){ 
            _send_timer->stop();
            return;
        }
    }
    // **************Delay stop**************

    bool client_run_loop=_ec_wrapper_info.client->get_client_status().run_loop; // client thread still running.
    if(client_run_loop){
        _ec_gui_pdo->write();
        _ec_wrapper_info.client->write();
    }

    if(!_ec_gui_cmd->get_command_sts() || !client_run_loop){
        if(count_reset_ref==0){
            onSendStopBtnReleased(); // stop sending references with delay
        }
    } // stop motors command

}

void EcGuiWrapper::start_stop_record()
{
    if(check_client_setup()){
        if(_ec_wrapper_info.device_info.empty()){
            QMessageBox msgBox;
            msgBox.warning(this,msgBox.windowTitle(),
                           tr("Cannot recod PDO.\n"
                              "Please press scan device button"),
                           QMessageBox::Ok);
        }
        else{
            if(!_record_started){
                _record_started = true;
                _ec_logger->start_mat_logger();
                _record_action->setIcon(QIcon(":/icon/stop_record.png"));
                _record_action->setText("Stop Record");
            }
            else{
                stop_record();
            }
        }
    }
}

void EcGuiWrapper::stop_record()
{
    if(_record_started && check_client_setup()){
        _record_started = false;
        _ec_logger->stop_mat_logger();
        _record_action->setIcon(QIcon(":/icon/record.png"));
        _record_action->setText("Record");
    }
}

void EcGuiWrapper::start_stop_receive()
{
    if(check_client_setup()){
        if(_ec_wrapper_info.device_info.empty()){
            QMessageBox msgBox;
            msgBox.warning(this,msgBox.windowTitle(),
                           tr("Cannot receive PDO.\n"
                              "Please press scan device button"),
                           QMessageBox::Ok);
        }
        else{
            if(!_receive_started){
                _receive_started = true;
                _receive_action->setIcon(QIcon(":/icon/stop_read.png"));
                _receive_action->setText("Stop Receive");
                _ec_gui_pdo->restart_receive_timer();
                _receive_timer->start(_time_ms);
            }
            else{
                stop_receive();
            }   
        }
    }
}

void EcGuiWrapper::stop_receive()
{
    if(_receive_started && check_client_setup()){
        _receive_started = false;
        _receive_action->setIcon(QIcon(":/icon/read.png"));
        _receive_action->setText("Receive");
        _receive_timer->stop();
    }
}

void EcGuiWrapper::receive()
{
    if(_ec_wrapper_info.client->get_client_status().run_loop){
        _ec_wrapper_info.client->read();
        _ec_gui_pdo->read();
    }
    else{
        stop_receive();
    }
}

EcGuiWrapper::~EcGuiWrapper()
{
    _ec_logger->stop_mat_logger();
}
