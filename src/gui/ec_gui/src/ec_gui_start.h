#ifndef EC_GUI_START_H
#define EC_GUI_START_H

#include <QtUiTools>
#include <QWidget>
#include <QLineEdit>

#include "slider_widget.h"

#include "cmn_utils.h"
#include "ec_udp.h"
#include "ec_pdo_read.h"
#include "ec_utils.h"
#include <QMainWindow>

class EcGuiStart : public QWidget
{
    Q_OBJECT

public:

    struct joint_info_t{

    std::string joint_name;

    double min_pos;
    double max_pos;
    double max_vel;
    double max_torq;

    double actual_pos;
    double actual_vel;
    double actual_torq;

    };

    explicit EcGuiStart(std::map<int ,joint_info_t > joint_info_map,
                        EcUtils::EC_CONFIG ec_config,
                        std::shared_ptr<EcUDP> client,
                        QWidget *parent = nullptr);

    ~EcGuiStart();

    std::string getFieldType() const;
    std::string getModeType() const;
    double getUDPFreq() const;

    void onApplyCmdReleased();
    void onSendStopBtnReleased();
    void onReceiveStopBtnReleased();
    void onNotAllCmdReleased();
    void onAllCmdReleased();
    void onLED_ON_OFF_Released();
    
public slots:
    void readCommand();
    void readModeType();
    void OnUDPFreqChanged();
    void UDP_Communication_send();
    double filtering(SecondOrderFilter<double>::Ptr filter,double actual_value);
    void UDP_Communication_receive();
    void warnig_level_batt();
    

private:

  EcUtils::EC_CONFIG  _ec_config;
  std::map<int ,joint_info_t > _joint_info_map;
  std::vector<int> _slave_id_led;
  QVBoxLayout *_l;

  std::map<int, SliderWidget*> _position_sw_map;
  std::map<int, SliderWidget*> _velocity_sw_map;

  std::map<int, SliderWidget*> _position_t_sw_map;
  std::map<int, SliderWidget*> _torque_sw_map;
  std::map<int, SliderWidget*> _sw_map_selected;

  EcUDP::ClientCmdType _ctrl_cmd_type;

  MST _motors_start = {};
  PAC _brake_cmds = {};
  std::vector<float> _gains;
  float _value;
  int _udp_ms_req;
  int _hysteresis_battery_level;

  bool _motor_start_req,_send_ref;
  bool _first_send_udp_comm;

  QTreeWidget * _tree_wid;

  QComboBox * _fieldtype_combobox;
  QComboBox * _mode_type_combobox;
  QComboBox * _udp_freq_combobox;


  QVBoxLayout *_sliders_poslayout,*_sliders_vellayout,*_sliders_torqlayout;
  QTabWidget *_tabcontrol;

  QDialogButtonBox  * _cmd_manager;
  QPushButton * _applybtn;
  QPushButton *_send_stop_btn,*_notallbtn,*_allbtn;

  QLCDNumber *_battery_level;
  QTimer *_timer_change_color;
  bool _flashing,_first_detection;
  int _count_warning,_count_not_warning;

  QTimer *_UDPTimer_send,*_UDPTimer_receive;

  std::shared_ptr<EcUDP> _client;
  std::vector<MR> _motors_ref;
  MotorRefFlags _motor_ref_flags;
  
  std::shared_ptr<EcPDORead> _ec_pdo_read;
};

#endif // EC_GUI_START_H
