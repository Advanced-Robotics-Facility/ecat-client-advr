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

    explicit EcGuiStart(QWidget *parent = nullptr);

    ~EcGuiStart();

public slots:
    void onStartEtherCATSystem();
    void onStopEtherCATSystem();
    void onScanDeviceReleased();
    void OnMouseDoubleClicked(QTreeWidgetItem* item, int column);
    void OnMouseClicked(QTreeWidgetItem* item, int column);
    void OnProtocolChanged();
    
protected:
        bool eventFilter( QObject* o, QEvent* e );
    
private:
  
  EcGuiWrapper::ec_wrapper_info_t _ec_wrapper_info;
  EcGuiWrapper::Ptr _ec_gui_wrapper;
  
  QTreeWidget * _net_tree_wid;
  QTreeWidgetItem* _net_item;
  int _net_column;
  
  QProcess *_ec_master_process,*_server_process;
  QString find_running_process(QProcess * process,QString bin_name,QString& stdout);
  QString find_process(QProcess * process,QString bin_name,QString& stdout);
  void start_process(QProcess *process,QString bin_file_path,QString option);
  void kill_process(QProcess *process,QString bin_name,QString& stdout);
  QStringList _ssh_command;
  QString _ec_master_stoud,_server_stdout;
  EcGuiTerminal::Ptr _ec_master_terminal, _server_terminal;
  QString _server_hostname,_server_ip,_server_port,_server_protocol,_server_pwd;
  bool _etherCAT_sys_started;
  
  QComboBox * _protocol_combobox;
  
  void on_ec_process_readyReadStandardOutput();
  void on_server_process_readyReadStandardOutput();
  void set_ec_network();
  void error_on_scannig();
  void restart_gui();
  void try_gui();
  void add_device();
  void scan_device();
  void clear_device();
  void create_ec_iface();
  bool create_ssh_cmd();
};

#endif // EC_GUI_START_H
