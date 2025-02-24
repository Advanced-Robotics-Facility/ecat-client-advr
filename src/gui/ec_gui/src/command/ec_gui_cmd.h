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

    bool get_command_sts();
    void set_command_sts(bool device_controlled);
    void onApplyCmd();
    void onApplyCmdReleased();
    void onNotAllCmdReleased();
    void onAllCmdReleased();
    void onNotAllBrakeReleased();
    void onAllBrakeReleased();
    void restart_ec_gui_cmd(EcIface::Ptr client);
    
public slots:
    void readCommand();
private:

  std::string getFieldType() const;
  void launch_cmd_message(QString message);
  void fill_start_stop_motor();
  void fill_start_stop_valve();
  void fill_start_stop_pump();
  bool braking_cmd_req();

  ClientCmdType _ctrl_cmd_type;

  DST _start_devices = {};
  PAC _brake_cmds = {};

  QString _cmd_message;
  bool _device_start_req,_device_controlled;
  bool _motors_selected,_valves_selected,_pumps_selected;

  QComboBox * _fieldtype_combobox;

  QDialogButtonBox  * _cmd_manager;
  QPushButton * _applybtn;
  QPushButton *_notallbtn,*_allbtn;
  QListWidget* _device_list_wid;

  EcIface::Ptr _client;
  EcGuiSlider::Ptr _ec_gui_slider;
  EcGuiSlider::slider_map_t _slider_map;
};

#endif // EC_GUI:CMD_H
