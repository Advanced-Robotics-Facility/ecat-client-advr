#ifndef EC_GUI_NET_H
#define EC_GUI_NET_H

#include <QtUiTools>
#include <QWidget>

#include "ec_gui_terminal.h"


class EcGuiNet : public QWidget
{
    Q_OBJECT
public:
    
    struct ec_net_info_t{
        std::string protocol;
        std::string host_name;
        uint32_t host_port;
    };
    
    typedef std::shared_ptr<EcGuiNet> Ptr;

    explicit EcGuiNet(QWidget *parent = nullptr);

    ~EcGuiNet();

    bool start_network();
    void stop_network();
    ec_net_info_t get_net_setup();

public slots:
    void ec_master_processFinished(int, QProcess::ExitStatus);
    void server_processFinished(int, QProcess::ExitStatus);
    void OnMouseClicked(QTreeWidgetItem* item, int column);
    void OnPasswordEntered();
    void OnPasswordChanged();
    void OnProtocolChanged();

protected:
    bool eventFilter( QObject* o, QEvent* e );
    
private:

  QTreeWidget * _net_tree_wid;
  QTreeWidgetItem* _net_item;
  QLineEdit *_password;
  int _net_column;
  
  QProcess *_ec_master_process,*_server_process;
  QStringList _ssh_command;
  QString _ec_master_stdout,_server_stdout;
  QString _ec_master_file_path,_server_file_path,_gui_file_path;
  QFile *_ec_master_file=nullptr,*_server_file=nullptr;
  QTextStream *_ec_master_stream,*_server_stream;
  QString _server_hostname,_server_ip,_server_port,_server_protocol,_server_pwd;
  QString _master_terminal_pid="",_server_terminal_pid="",_gui_terminal_pid="";
  
  QComboBox * _protocol_combobox;
  
  bool create_ssh_cmd(QProcess *process,QString& stdout);
  QString find_running_process(QProcess * process,QString bin_name,QString& stdout);
  QString find_process(QProcess * process,QString bin_name,QString& stdout);
  void start_process(QProcess *process,QString bin_file_path,QString option);
  void kill_process(QProcess *process,QString bin_name,QString& stdout);
  
  void kill_view_process(const QString &terminal_pid);
  void view_process(const QString &file_path,QString &terminal_pid);
  void ec_master_readyStdO();
  void view_master_process();
  void server_readyStdO();
  void view_server_process();
  void set_ec_network();    
  void view_gui_process();
};
#endif // EC_GUI_NET_H
