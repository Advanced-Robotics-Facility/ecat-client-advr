#ifndef EC_GUI_WRAPPER_H
#define EC_GUI_WRAPPER_H

#include <QtUiTools>
#include <QWidget>
#include <QLineEdit>
#include "cmn_utils.h"
#include "ec_utils.h"

#include "ec_gui_slider.h"
#include "ec_gui_pdo.h"
#include "ec_gui_sdo.h"
#include "ec_gui_cmd.h"


class EcGuiWrapper : public QWidget
{
    Q_OBJECT

public:
    
    struct ec_wrapper_info_t{
        EcIface::Ptr client;
        SSI device_info;
        
        std::map<int ,EcGuiSlider::joint_info_t > joint_info_map;
        
        MotorStatusMap internal_motor_status_map;
        FtStatusMap internal_ft6_status_map;
        PwrStatusMap internal_pow_status_map;
        ImuStatusMap internal_imu_status_map;
        
        SRD_SDO internal_sdo_map;
        SRD_SDO sdo_map;
    };

    typedef std::shared_ptr<EcGuiWrapper> Ptr;
    
    explicit EcGuiWrapper(QWidget *parent = nullptr);

    ~EcGuiWrapper();

    void restart_gui_wrapper(ec_wrapper_info_t ec_wrapper_info);
    bool get_wrapper_send_sts();
    void setFreq();
    void onSendStopBtnReleased();
    int get_period_ms();

public slots:
    void DwTopLevelChanged(bool isFloating);
    void OnFreqChanged();
    void send();
    void receive();
    void warnig_level_batt();
    void start_receive();
    void stop_receive();
    void stat_record();
    void stop_record();
private:
  
  ec_wrapper_info_t _ec_wrapper_info;
  QDockWidget *_command_dw,*_pdo_sdo_dw,*_graphics_dw;
  
  EcGuiSlider::Ptr _ec_gui_slider;
  EcGuiSlider::slider_map_t _slider_map;
  EcGuiPdo::Ptr _ec_gui_pdo;
  EcGuiSdo::Ptr _ec_gui_sdo;
  EcGuiCmd::Ptr _ec_gui_cmd;

  int _time_ms;
  int _hysteresis_battery_level;

  float _ctrl_cmd;
  bool _send_ref;
  bool _first_send;

  QComboBox * _freq_combobox;
  QPushButton *_send_stop_btn;

  QLCDNumber *_battery_level;
  QTimer *_timer_change_color;
  bool _flashing,_first_detection;
  int _count_warning,_count_not_warning;

  QTimer *_send_timer,*_receive_timer;
  QAction *_receive_action,*_stop_receive_action;
  bool _receive_started;

  QAction *_record_action,*_stop_record_action;
  bool _record_started;
  
  bool check_client_setup();
};

#endif // EC_GUI_WRAPPER_H
