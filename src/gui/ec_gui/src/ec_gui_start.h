#ifndef EC_GUI_START_H
#define EC_GUI_START_H

#include <QtUiTools>
#include <QWidget>
#include <QLineEdit>
#include <QMainWindow>

#include "cmn_utils.h"
#include "ec_utils.h"

#include "ec_gui_slider.h"
#include "ec_gui_pdo.h"
#include "ec_gui_cmd.h"


class EcGuiStart : public QMainWindow
{
    Q_OBJECT

public:

    explicit EcGuiStart(std::map<int ,EcGuiSlider::joint_info_t > joint_info_map,
                        EcUtils::EC_CONFIG ec_config,
                        EcIface::Ptr client,
                        QWidget *parent = nullptr);

    ~EcGuiStart();

    double getFreq() const;
    void onSendStopBtnReleased();

public slots:
    void OnFreqChanged();
    void send();
    void receive();
    void warnig_level_batt();
    void stat_receive();
    void stop_receive();
    void stat_record();
    void stop_record();
private:
  
  EcUtils::EC_CONFIG  _ec_config;
  std::map<int ,EcGuiSlider::joint_info_t> _joint_info_map;
  std::vector<int> _slave_id_led;
  
  EcGuiSlider::Ptr _ec_gui_slider;
  EcGuiSlider::slider_map_t _slider_map;
  EcGuiPdo::Ptr _ec_gui_pdo;
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
  EcIface::Ptr _client;
  QAction *_receive_action,*_stop_receive_action;
  bool _receive_started;

  QAction *_record_action,*_stop_record_action;
  bool _record_started;

};

#endif // EC_GUI_START_H
