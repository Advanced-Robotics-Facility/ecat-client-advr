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

    _receive_action=new QAction();
    connect(_receive_action, SIGNAL(triggered()), this, SLOT(start_stop_receive()));

    _record_action = parent->findChild<QAction *>("actionRecord");
    connect(_record_action, SIGNAL(triggered()), this, SLOT(start_stop_record()));
    
    _receive_started = _record_started = false;
    
    _ec_gui_slider = std::make_shared<EcGuiSlider>(parent);
    
    _ec_gui_pdo = std::make_shared<EcGuiPdo>(_ec_gui_slider,parent);
    
    _ec_gui_sdo = std::make_shared<EcGuiSdo>(parent);

    _ec_gui_cmd = std::make_shared<EcGuiCmd>(_ec_gui_slider,parent);

    _ec_logger=std::make_shared<EcLogger>(false);

    _send_pdo=false;
        
    // Get Send and Stop button
    _send_stop_btn = parent->findChild<QPushButton *>("SendStopBtn");

    connect(_send_stop_btn, &QPushButton::released,this, &EcGuiWrapper::onSendStopBtnReleased);
    
    // create a timer for receiving PDO
    _receive_timer = new QTimer(this);
    _receive_timer->setTimerType(Qt::PreciseTimer);

    // setup signal and slot
    connect(_receive_timer, SIGNAL(timeout()),this, SLOT(receive()));

    // create a timer for showing PDO
    _log_timer = new QTimer(this);
    _log_timer->setTimerType(Qt::PreciseTimer);

    // setup signal and slot
    connect(_log_timer, SIGNAL(timeout()),this, SLOT(log()));

    _time_ms = 4;
    _max_stop_write=4;
    _stopping_write_counter=_max_stop_write;
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

bool EcGuiWrapper::get_wrapper_cmd_sts()
{
    return _ec_gui_cmd->get_command_sts();
}

void EcGuiWrapper::clear_gui_wrapper()
{
    stop_receive();
    stop_record();
    stop_wrapper_thread();

    _ec_wrapper_info.client.reset();
    _ec_wrapper_info.device_info.clear();
    _ec_wrapper_info.sdo_map.clear();
    _ec_wrapper_info.device_ctrl.device_gains.clear();
    _ec_wrapper_info.device_ctrl.device_limits.clear();

    _ec_gui_slider->delete_sliders();

    _ec_gui_cmd->restart_ec_gui_cmd(_ec_wrapper_info.client);

    _ec_gui_pdo->restart_ec_gui_pdo(_ec_wrapper_info.client,_ec_logger);

    _ec_gui_sdo->restart_ec_gui_sdo(_ec_wrapper_info.client,_ec_wrapper_info.sdo_map);
}

void EcGuiWrapper::restart_gui_wrapper(ec_wrapper_info_t ec_wrapper_info)
{

    _ec_wrapper_info = ec_wrapper_info;

    _ec_logger->init_mat_logger(_ec_wrapper_info.device_info);
    
    _ec_gui_slider->create_sliders(_ec_wrapper_info.device_info,_ec_wrapper_info.device_ctrl);
    
    _ec_gui_cmd->restart_ec_gui_cmd(_ec_wrapper_info.client);
    
    _ec_gui_pdo->restart_ec_gui_pdo(_ec_wrapper_info.client,_ec_logger);
    
    _ec_gui_sdo->restart_ec_gui_sdo(_ec_wrapper_info.client,_ec_wrapper_info.sdo_map);

    _ec_wrapper_thread = std::make_shared<std::thread>(&EcGuiWrapper::wrapper_thread,this);
    
    _start_loop_time = std::chrono::high_resolution_clock::now();
    _loop_time = _start_loop_time;
    _run_wrapper_thread=true;

    start_stop_receive();
}

bool EcGuiWrapper::check_client_setup()
{
    bool ret=false;
    QMessageBox msgBox;
    if(_ec_wrapper_info.client == nullptr){
        msgBox.critical(this,msgBox.windowTitle(),
                        tr("EtherCAT client not setup"
                           ",please scan device button.\n"));
    }
    else{
        if(!_ec_wrapper_info.client->get_client_status().run_loop){
            msgBox.critical(this,msgBox.windowTitle(),
                tr("EtherCAT Client loop is not running state"
                    ",please press scan device button.\n"));
        }
        else{
            ret = true;
        }
    }
    
    return ret;
}

void EcGuiWrapper::onSendStopBtnReleased()
{
    // @NOTE to be tested.
    _ec_gui_slider->reset_sliders();

    bool devices_controlled = _ec_gui_cmd->get_command_sts(); // devices controlled
    
    if((_send_stop_btn->text()=="Start Motion")&&(devices_controlled)){
        _send_stop_btn->setText("Stop Motion");
        _record_action->setEnabled(false);
        _ec_gui_slider->enable_sliders();
        _mutex_send.lock();
        _send_pdo=true;
        _stopping_write_counter=_max_stop_write;
        _ec_gui_pdo->starting_write(_time_ms);
        _ec_wrapper_info.client->set_reference_flag(1); // multi-reference for UDP protocol
        _mutex_send.unlock();
    }
    else{ 
        _record_action->setEnabled(true);
        _ec_gui_slider->disable_sliders();
        _mutex_send.lock();
        _send_pdo=false;
        _stopping_write_counter=0;     
        _ec_gui_pdo->stopping_write();//STOP align all references to zero or with the actual position for the motors
        _mutex_send.unlock();
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
            if(!_record_started && _run_wrapper_thread){
                _ec_logger->start_mat_logger();
                _record_started = true;
                _record_action->setIcon(QIcon(":/icon/stop_record.png"));
                _record_action->setText("Stop Record");
                _log_timer->start(_time_ms+1); // not precise timer.
                return;
            }
        }
    }

    stop_record();
}

void EcGuiWrapper::stop_record()
{
    if(_record_started){
        _ec_logger->stop_mat_logger();
        _record_started = false;
        _record_action->setIcon(QIcon(":/icon/record.png"));
        _record_action->setText("Record");
        _log_timer->stop();
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
            if(!_receive_started && _run_wrapper_thread){
                _receive_started = true;
                _ec_gui_pdo->restart_receive_timer();
                _receive_timer->start(_time_ms+1); // not precise timer.
                return; 
            }
        }
    }

    stop_receive();
}

void EcGuiWrapper::stop_receive()
{
    if(_receive_started){
        _receive_started = false;
        _receive_timer->stop();
    }
}

void EcGuiWrapper::receive()
{
    if(_receive_started){
        _ec_wrapper_info.client->read();
        _ec_gui_pdo->read();
    }
}

void EcGuiWrapper::log()
{
    _mutex_send.lock();
    _ec_gui_pdo->sync_write();
    _mutex_send.unlock();
    _ec_gui_pdo->log();
}

////*************************** EC GUI WRAPPER THREAD ************************************
void EcGuiWrapper::send()
{
    _mutex_send.lock();
    if(_send_pdo || _stopping_write_counter< _max_stop_write){
        // **************Delay stop**************
        if(!_send_pdo){
            if(_stopping_write_counter== _max_stop_write-1){
                _ec_wrapper_info.client->set_reference_flag(2); // last reference for UDP protocol
            }
            _stopping_write_counter++; //4*ts delayed write
        }
        // **************Delay stop**************

        _ec_gui_pdo->write();
        _ec_wrapper_info.client->write();

        if(!_ec_gui_cmd->get_command_sts() && _send_pdo){
            _send_pdo=false;
            _send_stop_btn->click();
        } 
    }
    _mutex_send.unlock();
}

void EcGuiWrapper::wrapper_thread()
{
    while(_run_wrapper_thread){
        
        if(_ec_wrapper_info.client->get_client_status().run_loop){
            
            send();
            
            _loop_time = _loop_time + std::chrono::milliseconds(_time_ms);
            std::this_thread::sleep_until(_loop_time);
        }
        else{
            _run_wrapper_thread=false;

            if(_send_pdo){
                _send_stop_btn->click();
            }

            _receive_action->trigger(); // ONLY send a trigger to receive action for showing that client is not alive!
            stop_record();

            _ec_gui_cmd->set_command_sts(false);
        }
    }
}

void EcGuiWrapper::stop_wrapper_thread()
{
    bool delay_stop_send = _send_pdo;
    _mutex_send.lock();
    _send_pdo=false;
    _stopping_write_counter=_max_stop_write;
    if(delay_stop_send){
        _stopping_write_counter=0;
    }
    _ec_gui_pdo->stopping_write();//STOP align all references to zero or with the actual position for the motors
    _mutex_send.unlock();
    
    if(delay_stop_send){
        std::this_thread::sleep_for(std::chrono::milliseconds(5*_time_ms));
    }

    _ec_logger->stop_mat_logger();

    _run_wrapper_thread=false;
    if(_ec_wrapper_thread){
        if(_ec_wrapper_thread->joinable()){
            _ec_wrapper_thread->join();
        }
        _ec_wrapper_thread.reset();
    }
}
////*************************** EC GUI WRAPPER THREAD ************************************

EcGuiWrapper::~EcGuiWrapper()
{
    _ec_gui_slider->reset_sliders();
    stop_wrapper_thread();
}
