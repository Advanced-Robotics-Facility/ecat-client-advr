#ifndef EC_GUI_START_H
#define EC_GUI_START_H

#include <QtUiTools>
#include <QWidget>
#include <QLineEdit>

#include "slider_widget.h"

#include "cmn_utils.h"
#include "ec_pdo_read.h"
#include "ec_utils.h"
#include "ec_gui_cmd.h"
#include <QMainWindow>

class EcGuiStart : public QMainWindow
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
                        EcIface::Ptr client,
                        QWidget *parent = nullptr);

    ~EcGuiStart();

    double getFreq() const;
    void onSendStopBtnReleased();
    void onStopPlotting();
    
public slots:
    void OnFreqChanged();
    void send();
    void receive();
    void warnig_level_batt();
    

private:
  double filtering(SecondOrderFilter<double>::Ptr filter,double actual_value);
  
  EcUtils::EC_CONFIG  _ec_config;
  std::map<int ,joint_info_t > _joint_info_map;
  std::vector<int> _slave_id_led;
  
  std::map<int, SliderWidget*> _position_sw_map;
  std::map<int, SliderWidget*> _velocity_sw_map;

  std::map<int, SliderWidget*> _position_t_sw_map;
  std::map<int, SliderWidget*> _torque_sw_map;
  std::map<int, SliderWidget*> _sw_map_selected;
  std::shared_ptr<EcGuiCmd> _ec_gui_cmd;

  std::vector<float> _gains;
  float _value;
  int _time_ms;
  int _hysteresis_battery_level;

  bool _send_ref;
  bool _first_send;

  QTreeWidget * _tree_wid;
  QComboBox * _freq_combobox;


  QVBoxLayout *_sliders_poslayout,*_sliders_vellayout,*_sliders_torqlayout;

  QPushButton *_send_stop_btn;
  QPushButton * _stop_plotting_btn;

  QLCDNumber *_battery_level;
  QTimer *_timer_change_color;
  bool _flashing,_first_detection;
  int _count_warning,_count_not_warning;

  QTimer *_send_timer,*_receive_timer;

  EcIface::Ptr _client;
  std::vector<MR> _motors_ref;
  MotorRefFlags _motor_ref_flags;
  
  std::shared_ptr<EcPDORead> _ec_pdo_read;
};

#endif // EC_GUI_START_H
