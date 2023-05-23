#include "ec_gui_start.h"

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

EcGuiStart::EcGuiStart(std::map<int ,EcGuiSlider::joint_info_t> joint_info_map,EcUtils::EC_CONFIG ec_config,EcIface::Ptr client,QWidget *parent) :
    QMainWindow(parent),
    _ec_config(ec_config),
    _joint_info_map(joint_info_map),
    _client(client)
{

    /* Load ui */
    auto wid = LoadUiFile(this);

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
    
     
    _slave_id_led = _ec_config.slave_id_led;
    
    _ec_gui_slider = std::make_shared<EcGuiSlider>(_joint_info_map,
                                                   this);
    
    _ec_gui_pdo = std::make_shared<EcGuiPdo>(_ec_gui_slider,
                                             _client,
                                             this);

    _ec_gui_cmd = std::make_shared<EcGuiCmd>(_ec_gui_slider,
                                             _slave_id_led,
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

    _receive_timer->stop();

    _ec_gui_pdo->restart_receive_timer();

    _receive_timer->start(_time_ms);

    _client->set_loop_time(_time_ms);


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
    if(_ec_gui_cmd->get_cmd_sts(_ctrl_cmd))
    {
        _ec_gui_pdo->write();
    }
    else
    {
        onSendStopBtnReleased(); // stop sending references
    }
}

void EcGuiStart::receive()
{
    if(_client->is_client_alive())
    {
        /************************************* READ PDOs  ********************************************/
        _ec_gui_pdo->read();
        /************************************* READ PDOs  ********************************************/
        
        EcGuiSlider::slider_map_t slider_map=_ec_gui_slider->get_sliders();
        auto motors_status_map= _client->get_motors_status();
        if(!motors_status_map.empty())
        {
            for ( const auto &[esc_id, motor_status] : motors_status_map)
            {
                /************************************* ALIGN POSITION SLIDERS with the motor position ********************************************/
                double motor_pos=std::get<1>(motor_status);
                if(slider_map.position_sw_map.count(esc_id)>0)
                {
                    slider_map.position_sw_map[esc_id]->set_actual_slider_value(motor_pos);
                    slider_map.position_t_sw_map[esc_id]->set_actual_slider_value(motor_pos);
                }
                /************************************* ALIGN POSITION SLIDERS with the motor position ********************************************/
            }
        }
        
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
