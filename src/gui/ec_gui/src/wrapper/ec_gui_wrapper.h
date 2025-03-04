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

    void clear_gui_wrapper();
    void restart_gui_wrapper(ec_wrapper_info_t ec_wrapper_info);
    bool get_wrapper_send_sts();
    bool get_wrapper_cmd_sts();
    void onSendStopBtnReleased();

public slots:
    void DwTopLevelChanged(bool isFloating);
    void start_stop_receive();
    void stop_receive();
    void start_stop_record();
    void stop_record();
    void show();
private:
  
  ec_wrapper_info_t _ec_wrapper_info;
  QDockWidget *_command_dw,*_pdo_sdo_dw,*_graphics_dw;
  
  EcGuiSlider::Ptr _ec_gui_slider;
  EcGuiSlider::slider_map_t _slider_map;
  EcGuiPdo::Ptr _ec_gui_pdo;
  EcGuiSdo::Ptr _ec_gui_sdo;
  EcGuiCmd::Ptr _ec_gui_cmd;
  EcLogger::Ptr _ec_logger;

  std::shared_ptr<std::thread> _ec_wrapper_thread;
  std::mutex _mutex_send,_mutex_log;
  std::chrono::high_resolution_clock::time_point _loop_time,_start_loop_time;

  int _time_ms;
  bool _run_wrapper_thread=false,_send_pdo=false;
  
  QPushButton *_send_stop_btn;

  QTimer *_show_timer;
  QAction *_receive_action,*_record_action;
  bool _receive_started,_record_started;

  bool check_client_setup();
  void stop_wrapper_thread();
  void wrapper_thread();
  void receive();
  void send();
  void log();
  uint8_t _stopping_write_counter=0;
  uint8_t _max_stop_write;
};

#endif // EC_GUI_WRAPPER_H
