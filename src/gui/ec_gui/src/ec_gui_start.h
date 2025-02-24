#ifndef EC_GUI_START_H
#define EC_GUI_START_H

#include <QtUiTools>
#include <QWidget>
#include <QMainWindow>

#include "cmn_utils.h"
#include "utils/ec_utils.h"

#include "ec_gui_net.h"
#include "ec_gui_wrapper.h"
#include "ec_gui_terminal.h"


class EcGuiStart : public QMainWindow
{
    Q_OBJECT

public:

    explicit EcGuiStart(QWidget *parent = nullptr);

    ~EcGuiStart();

public slots:
    void onStartEtherCATSystem();
    void onStopEtherCATSystem();
    void onScanDeviceReleased();
    
private:
  
  EcGuiWrapper::ec_wrapper_info_t _ec_wrapper_info;
  EcGuiWrapper::Ptr _ec_gui_wrapper;
  EcGuiNet::Ptr _ec_gui_net;
  
  QTreeWidget * _net_tree_wid;
  bool _etherCAT_sys_started;
  
  void error_on_scannig();
  void restart_gui();
  void add_device();
  void scan_device();
  void clear_device();
  void clear_gui();
  void create_ec_iface();
  bool stopping_ec_sys();
  void stopping_client();
};

#endif // EC_GUI_START_H
