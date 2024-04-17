#ifndef EC_GUI_CMD_H
#define EC_GUI_CMD_H

#include <QtUiTools>
#include <QWidget>

#include "utils/ec_utils.h"
#include "ec_gui_slider.h"

class EcGuiCmd : public QWidget
{
     Q_OBJECT
public:

    typedef std::shared_ptr<EcGuiCmd> Ptr;
    EcGuiCmd(EcGuiSlider::Ptr ec_gui_slider,
             QWidget * parent = 0);

    ~EcGuiCmd();

    bool get_cmd_sts(float &ctrl_cmd);
    void onApplyCmdReleased();
    void onNotAllCmdReleased();
    void onAllCmdReleased();
    void onNotAllBrakeReleased();
    void onAllBrakeReleased();
    void restart_ec_gui_cmd(EcIface::Ptr client);
    
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
  float _ctrl_cmd;


  bool _motor_start_req,_send_ref,_motors_selected;


  QComboBox * _fieldtype_combobox;
  QComboBox * _mode_type_combobox;
  QTabWidget *_devicecontrol;
  QTabWidget *_tabcontrol;

  QDialogButtonBox  * _cmd_manager;
  QPushButton * _applybtn;
  QPushButton *_notallbtn,*_allbtn;

  EcIface::Ptr _client;
  EcGuiSlider::Ptr _ec_gui_slider;
  EcGuiSlider::slider_map_t _slider_map;
  std::map<int, SliderWidget*> _actual_sw_map_selected;
  std::vector<int> _slave_id_led;
};

#endif // EC_GUI:CMD_H
