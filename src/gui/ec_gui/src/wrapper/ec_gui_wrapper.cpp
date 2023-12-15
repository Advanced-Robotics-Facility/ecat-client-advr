#include "ec_gui_wrapper.h"

#include <iostream>
#include <csignal>
#include <atomic>

#include <QLabel>
#include <QPixmap>
#include <QFile>

#include <chrono>
#define _HYST_THRESHOLD 5 // 5s

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
        SLOT(OnFreqChanged())
    );

    _battery_level = parent->findChild<QLCDNumber *>("BatteryLevel");
    _battery_level->setDigitCount(6);
    _battery_level->display(888888);
    _battery_level->setStyleSheet("background: black; color: #00FF00");
    
    _timer_change_color = new QTimer(this);
    _flashing=false;
    _first_detection=true;
    _count_warning = _count_not_warning = 0;
    connect(_timer_change_color, SIGNAL(timeout()), this, SLOT(warnig_level_batt()));
    
    _receive_action = parent->findChild<QAction *>("actionReceive");
    connect(_receive_action, SIGNAL(triggered()), this, SLOT(start_receive()));
    
    _stop_receive_action = parent->findChild<QAction *>("actionStopReceive");
    connect(_stop_receive_action, SIGNAL(triggered()), this, SLOT(stop_receive()));
    
    _record_action = parent->findChild<QAction *>("actionRecord");
    connect(_record_action, SIGNAL(triggered()), this, SLOT(stat_record()));
    
    _stop_record_action = parent->findChild<QAction *>("actionStopRecord");
    connect(_stop_record_action, SIGNAL(triggered()), this, SLOT(stop_record()));
    
    _receive_started = _record_started = false;
    
    _ec_gui_slider = std::make_shared<EcGuiSlider>(parent);
    
    _ec_gui_pdo = std::make_shared<EcGuiPdo>(_ec_gui_slider,
                                             parent);
    
    _ec_gui_sdo = std::make_shared<EcGuiSdo>(parent);

    _ec_gui_cmd = std::make_shared<EcGuiCmd>(_ec_gui_slider,
                                             parent);

    _send_ref=_first_send=false;
        
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


    setFreq();
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
    
    _ec_gui_slider->create_sliders(_ec_wrapper_info.joint_info_map);
    
    _ec_gui_cmd->restart_ec_gui_cmd(_ec_wrapper_info.client);
    
    _ec_gui_pdo->restart_ec_gui_pdo(_ec_wrapper_info.client);
    
    _ec_gui_pdo->set_internal_map(_ec_wrapper_info.internal_motor_status_map,
                                  _ec_wrapper_info.internal_ft6_status_map,
                                  _ec_wrapper_info.internal_pow_status_map,
                                  _ec_wrapper_info.internal_imu_status_map);
    
    _ec_gui_sdo->set_internal_sdo_map(_ec_wrapper_info.internal_sdo_map);
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

void EcGuiWrapper::setFreq()
{
    double freq=(_period_combobox->currentText().toDouble())/1000;
    
    _hysteresis_battery_level=  _HYST_THRESHOLD * freq; 
    
    if(_hysteresis_battery_level <= 1)
    {
        _hysteresis_battery_level= 2; // twice of communication time (i.e 5s-->10s or 10s-->20s)
    }

    _time_ms=_period_combobox->currentText().toInt();
}

int EcGuiWrapper::get_period_ms()
{
    return _time_ms;
}

void EcGuiWrapper::OnFreqChanged()
{
    setFreq();
    
    if(!check_client_setup())
    {
        return;
    }
    else
    {
        _ec_wrapper_info.client->set_loop_time(_time_ms);
    }

/**** RX STOP and START *****/

    if(_receive_started)
    {
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
    
    if((_send_stop_btn->text()=="Send")&&(_send_ref))
    {
        _send_timer->start(_time_ms);
        _period_combobox->setEnabled(false);
        _send_stop_btn->setText("Stop");
        _first_send=true;

        _ec_gui_slider->enable_sliders();

    }
    else
    {
        _send_timer->stop();
        _period_combobox->setEnabled(true);        
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



void EcGuiWrapper::send()
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

void EcGuiWrapper::stat_record()
{
    if(!check_client_setup())
        return;
    
    if(!_record_started)
    {
        if(_ec_wrapper_info.device_info.empty())
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
            _ec_wrapper_info.client->start_logging();
        }
    }
}

void EcGuiWrapper::stop_record()
{
    if(!check_client_setup())
        return;
    
    if(_record_started)
    {
        _record_started = false;
        _ec_wrapper_info.client->stop_logging();
    }
}

void EcGuiWrapper::start_receive()
{
    if(!check_client_setup())
        return;

    if(!_receive_started)
    {
        if(_ec_wrapper_info.device_info.empty())
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

void EcGuiWrapper::stop_receive()
{
    if(!check_client_setup())
        return;
    
    if(_receive_started)
    {
        _receive_started = false;
        
        _receive_timer->stop();
    }
}

void EcGuiWrapper::receive()
{
    if(_ec_wrapper_info.client->is_client_alive())
    {
        /************************************* READ PDOs  ********************************************/
        _ec_gui_pdo->read();
        /************************************* READ PDOs  ********************************************/
        
        /************************************* CHECK BATTERY LEVEL ********************************************/
        auto pow_status_map= _ec_wrapper_info.client->get_pow_status();
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
        stop_receive();
    }
}

void EcGuiWrapper::warnig_level_batt()
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

EcGuiWrapper::~EcGuiWrapper()
{
    
}
