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
    
    _pdo_sdo_dw = parent->findChild<QDockWidget *>("DataObject");
    connect(_pdo_sdo_dw, SIGNAL(topLevelChanged(bool)), this, SLOT(DwTopLevelChanged(bool)));
    
    _graphics_dw = parent->findChild<QDockWidget *>("Graphics");
    connect(_graphics_dw, SIGNAL(topLevelChanged(bool)), this, SLOT(DwTopLevelChanged(bool)));
    
    
    /*frequency */

    _period_combobox = parent->findChild<QComboBox *>("Period");
    _period_combobox->setCurrentIndex(9); // set Default 4ms.

    // change FONT
    QFont font;
    font.setPointSize(font.pointSize() + 5);
    _period_combobox->setFont(font);

    /* connection of frequency function */
    connect(_period_combobox, SIGNAL(currentIndexChanged(int)),this,
        SLOT(OnPeriodChanged())
    );

    _receive_action = parent->findChild<QAction *>("actionReceive");
    connect(_receive_action, SIGNAL(triggered()), this, SLOT(start_receive()));
    
    _stop_receive_action = parent->findChild<QAction *>("actionStopReceive");
    connect(_stop_receive_action, SIGNAL(triggered()), this, SLOT(stop_receive()));
    
    _record_action = parent->findChild<QAction *>("actionRecord");
    connect(_record_action, SIGNAL(triggered()), this, SLOT(start_record()));
    
    _stop_record_action = parent->findChild<QAction *>("actionStopRecord");
    connect(_stop_record_action, SIGNAL(triggered()), this, SLOT(stop_record()));
    
    _receive_started = _record_started = false;
    
    _ec_gui_slider = std::make_shared<EcGuiSlider>(parent);
    
    _ec_gui_pdo = std::make_shared<EcGuiPdo>(_ec_gui_slider,
                                             parent);
    
    _ec_gui_sdo = std::make_shared<EcGuiSdo>(parent);

    _ec_gui_cmd = std::make_shared<EcGuiCmd>(_ec_gui_slider,
                                             parent);

    _send_ref=false;
        
    // Get Send and Stop button
    _send_stop_btn = parent->findChild<QPushButton *>("SendStopBtn");

    connect(_send_stop_btn, &QPushButton::released,
            this, &EcGuiWrapper::onSendStopBtnReleased);
    
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

    _time_ms = 4; //62.5 Hz
}

void EcGuiWrapper::DwTopLevelChanged(bool isFloating)
{
    auto dw = qobject_cast<QDockWidget*>(sender());
    if(isFloating)
    {
        dw->setWindowFlags(Qt::Window);
        dw->show();
    }
}

bool EcGuiWrapper::get_wrapper_send_sts()
{
    return _send_ref;
}

void EcGuiWrapper::restart_gui_wrapper(ec_wrapper_info_t ec_wrapper_info)
{
    if(_receive_started)
    {
        stop_receive();
    }
    if(_record_started)
    {
        stop_record();
    }
    
    _ec_wrapper_info = ec_wrapper_info;
    
    _ec_gui_slider->create_sliders(_ec_wrapper_info.device_info);
    
    _ec_gui_cmd->restart_ec_gui_cmd(_ec_wrapper_info.client);
    
    _ec_gui_pdo->restart_ec_gui_pdo(_ec_wrapper_info.client);
    
    _ec_gui_sdo->restart_ec_gui_sdo(_ec_wrapper_info.client,
                                    _ec_wrapper_info.sdo_map);
}

bool EcGuiWrapper::check_client_setup()
{
    bool ret=false;
    if(_ec_wrapper_info.client == nullptr)
    {
        QMessageBox msgBox;
        msgBox.critical(this,msgBox.windowTitle(),tr("EtherCAT client not setup, please press Start EtherCAT system button.\n"));
    }
    else
    {
        ret = true;
    }
    
    return ret;
}

int EcGuiWrapper::get_period_ms()
{
    return _period_combobox->currentText().toInt();
}

void EcGuiWrapper::OnPeriodChanged()
{
    if(_ec_wrapper_info.client == nullptr){
        return;
    }
    else{
        auto client_time = _period_combobox->currentText().toInt();
        _ec_wrapper_info.client->set_loop_time(client_time);
    }

/**** RX STOP and START *****/

    if(_receive_started){
        _receive_timer->stop();

        _ec_gui_pdo->restart_receive_timer();

        _receive_timer->start(_time_ms);
    }


/**** RX STOP and START *****/

}


void EcGuiWrapper::onSendStopBtnReleased()
{
    // @NOTE to be tested.
    _ec_gui_slider->reset_sliders();

    _send_ref = _ec_gui_cmd->get_cmd_sts(_ctrl_cmd);
    _ec_gui_pdo->set_ctrl_mode(_ctrl_cmd);
    count_reset_ref=0;
    
    if((_send_stop_btn->text()=="Start Motion")&&(_send_ref)){
        _period_combobox->setEnabled(false);
        _send_stop_btn->setText("Stop Motion");
        _ec_gui_slider->enable_sliders();

        _ec_gui_pdo->set_filter(_time_ms);
        _ec_gui_pdo->restart_send_timer();
        _send_timer->start(_time_ms);
    }
    else{
        _send_ref=false;
        _period_combobox->setEnabled(true);        
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

    if(!_send_ref){
        count_reset_ref++;
        _ec_gui_pdo->set_filter(_time_ms);//STOP align all references to zero or with the actual position for the motors
        _ec_gui_pdo->restart_send_timer();
        if(count_reset_ref>3){ 
            _ec_gui_pdo->clear_write();
            _send_timer->stop(); //delay stop
        }
    }

    if(_ec_wrapper_info.client->get_client_status().run_loop){
        _ec_gui_pdo->write();
        _ec_wrapper_info.client->write();
    }
    else{
        
    }

    if(!_ec_gui_cmd->get_cmd_sts(_ctrl_cmd)){
        if(count_reset_ref==0){
            onSendStopBtnReleased(); // stop sending references
        }
    } // stop motors command

}

void EcGuiWrapper::start_record()
{
    if(!check_client_setup())
        return;
    
    if(!_record_started){
        if(_ec_wrapper_info.device_info.empty()){
            QMessageBox msgBox;
            msgBox.warning(this,msgBox.windowTitle(),
                        tr("Cannot recod PDO.\n"
                        "Please press scan device button"),
                        QMessageBox::Ok);
        }
        else{
            _record_started = true;
        }
    }
}

void EcGuiWrapper::stop_record()
{
    if(!check_client_setup())
        return;
    
    if(_record_started){
        _record_started = false;
    }
}

void EcGuiWrapper::start_receive()
{
    if(!check_client_setup())
        return;

    if(!_receive_started){
        if(_ec_wrapper_info.device_info.empty()){
            QMessageBox msgBox;
            msgBox.warning(this,msgBox.windowTitle(),
                        tr("Cannot receive PDO.\n"
                        "Please press scan device button"),
                            QMessageBox::Ok);
        }
        else{
            _receive_started = true;
        
            _ec_gui_pdo->restart_receive_timer();

            _receive_timer->start(_time_ms);
        }
    }
}

void EcGuiWrapper::stop_receive()
{
    if(!check_client_setup())
        return;
    
    if(_receive_started){
        _receive_started = false;
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
    
}
