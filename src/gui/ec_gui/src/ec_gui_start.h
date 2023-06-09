#ifndef EC_GUI_START_H
#define EC_GUI_START_H

#include <QtUiTools>
#include <QWidget>
#include <QLineEdit>
#include <QMainWindow>

#include "cmn_utils.h"
#include "ec_utils.h"

#include "ec_gui_wrapper.h"
#include "ec_gui_terminal.h"


class EcGuiStart : public QMainWindow
{
    Q_OBJECT

public:

    explicit EcGuiStart(EcIface::Ptr client,
                        QWidget *parent = nullptr);

    ~EcGuiStart();

public slots:
    void onStartEtherCATSystem();
    void onStopEtherCATSystem();
    void onScanDeviceReleased();
    
private:
  
  EcGuiWrapper::ec_wrapper_info_t _ec_wrapper_info;
  EcGuiWrapper::Ptr _ec_gui_wrapper;
  
  QTreeWidget * _net_tree_wid;
  
  QProcess *_ec_master_process,*_server_process;
  QString find_exe(QProcess * process,QString exe_name,QString& stdout);
  void start_exe(QProcess *process,QString bin_file_path);
  void kill_exe(QProcess *process,QString exe_name);
  QStringList _ssh_command;
  QString _ec_master_stoud,_server_stdout;
  EcGuiTerminal::Ptr _ec_master_terminal, _server_terminal;
  
  void on_ec_process_readyReadStandardOutput();
  void on_server_process_readyReadStandardOutput();
  void error_on_scannig();
  void restart_gui();
  void try_gui();
  void add_device();
  void scan_device();
  void clear_device();
};

#endif // EC_GUI_START_H
