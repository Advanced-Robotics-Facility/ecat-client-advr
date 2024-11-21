#ifndef EC_GUI_WRAPPER_H
#define EC_GUI_WRAPPER_H

#include <QtUiTools>
#include <QWidget>
#include <QLineEdit>
#include "cmn_utils.h"
#include "utils/ec_utils.h"

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
        SRD_SDO sdo_map;
    };

    typedef std::shared_ptr<EcGuiWrapper> Ptr;
    
    explicit EcGuiWrapper(QWidget *parent = nullptr);

    ~EcGuiWrapper();

    void restart_gui_wrapper(ec_wrapper_info_t ec_wrapper_info);
    bool get_wrapper_send_sts();
    void onSendStopBtnReleased();

public slots:
    void DwTopLevelChanged(bool isFloating);
    void send();
    void receive();
    void start_receive();
    void stop_receive();
    void start_record();
    void stop_record();
private:
  
  ec_wrapper_info_t _ec_wrapper_info;
  QDockWidget *_command_dw,*_pdo_sdo_dw,*_graphics_dw;
  
  EcGuiSlider::Ptr _ec_gui_slider;
  EcGuiSlider::slider_map_t _slider_map;
  EcGuiPdo::Ptr _ec_gui_pdo;
  EcGuiSdo::Ptr _ec_gui_sdo;
  EcGuiCmd::Ptr _ec_gui_cmd;
  EcLogger::Ptr _ec_logger;

  int _time_ms;

  float _ctrl_cmd;
  bool _send_ref;
  
  QPushButton *_send_stop_btn;

  QTimer *_send_timer,*_receive_timer;
  QAction *_receive_action,*_stop_receive_action;
  bool _receive_started;

  QAction *_record_action,*_stop_record_action;
  bool _record_started;
  
  bool check_client_setup();
  int count_reset_ref=0;
};

#endif // EC_GUI_WRAPPER_H
