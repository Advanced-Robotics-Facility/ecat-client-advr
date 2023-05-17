#ifndef EC_GUI_CMD_H
#define EC_GUI_CMD_H

#include <QtUiTools>
#include <QWidget>

#include "ec_utils.h"

class EcGuiCmd : public QWidget
{
     Q_OBJECT
public:

    EcGuiCmd(EcIface::Ptr client,
             QWidget * parent = 0);

    ~EcGuiCmd();

    void onApplyCmdReleased();
    void onNotAllCmdReleased();
    void onAllCmdReleased();
    void onLED_ON_OFF_Released();
    
public slots:
    void readCommand();
    void readModeType();
private:

  std::string getFieldType() const;
  std::string getModeType() const;
  void enable_disable_pid();
  void launch_cmd_message(QString message);
  void fill_start_stop_cmd();
  bool braking_cmd_req();

  ClientCmdType _ctrl_cmd_type;

  MST _motors_start = {};
  PAC _brake_cmds = {};
  std::vector<float> _gains;
  float _value;


  bool _motor_start_req,_send_ref,_motors_selected;


  QComboBox * _fieldtype_combobox;
  QComboBox * _mode_type_combobox;
  QTabWidget *_tabcontrol;

  QDialogButtonBox  * _cmd_manager;
  QPushButton * _applybtn;
  QPushButton *_notallbtn,*_allbtn;

  EcIface::Ptr _client;
  
//   std::map<int, SliderWidget*> _position_sw_map;
//   std::map<int, SliderWidget*> _velocity_sw_map;
// 
//   std::map<int, SliderWidget*> _position_t_sw_map;
//   std::map<int, SliderWidget*> _torque_sw_map;
//   std::map<int, SliderWidget*> _sw_map_selected;

};

#endif // EC_GUI:CMD_H
