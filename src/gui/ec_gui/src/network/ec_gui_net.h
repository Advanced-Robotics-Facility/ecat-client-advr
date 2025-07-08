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
    bool stop_network();
    void copy_files_network(const QStringList &files_list);
    void open_firmware_config();
    void start_firmware_update();
    void stop_firmware_update();
    void set_repl_config(const QString& repl_bin);
    void set_protocol_enabled(bool enable);
    void set_net_enabled(bool enable);
    ec_net_info_t get_net_setup();

private slots:
    void ec_master_processFinished(int, QProcess::ExitStatus);
    void server_processFinished(int, QProcess::ExitStatus);
    void OnMouseClicked(QTreeWidgetItem* item, int column);
    void OnPasswordEntered();
    void OnPasswordChanged();
    void OnProtocolChanged();
    void stopping_network(bool force_stop=false);
    void connect_to_network();

protected:
    bool eventFilter( QObject* o, QEvent* e );
    
private:

  QTreeWidget * _net_tree_wid;
  QTreeWidgetItem* _net_item;
  QLineEdit *_password;
  int _net_column;
  bool _net_enabled;
  
  QProcess *_ec_master_process,*_server_process;
  QStringList _ssh_command;
  QString _ec_master_stdout,_server_stdout;
  QString _ec_master_file_path,_server_file_path,_gui_file_path;
  QFile *_ec_master_file=nullptr,*_server_file=nullptr;
  QTextStream *_ec_master_stream,*_server_stream;
  QString _server_username,_server_hostname,_server_port,_server_protocol,_server_pwd;
  QString _real_server_username;
  QString _master_terminal_pid="",_server_terminal_pid="",_gui_terminal_pid="";
  QString _repl_config;
  QAction *_connect_action;
  
  QComboBox * _protocol_combobox;
  bool _open_config_file=false;
  
  bool create_ssh_cmd(QProcess *process,QString& stdout);
  QString find_running_process(QProcess * process,QString bin_name,QString& stdout);
  QString find_process(QProcess * process,QString bin_name,QString& stdout);
  void start_process(QProcess *process,QString bin_file_path,QString option);
  void kill_process(QProcess *process,QString bin_name,QString& stdout);
  
  void kill_view_process(QString &terminal_pid);
  void view_process(const QString &file_path,QString &terminal_pid);
  bool start_master_process(const QString &bin_file_name,
                            const QString &option,
                            QString &error);
  void ec_master_readyStdO();
  void view_master_process();
  void server_readyStdO();
  void view_server_process();
  void set_ec_network();    
  void view_gui_process();
  bool copy_config_file();
  void save_config_file();
  void close_net_setup();
};
#endif // EC_GUI_NET_H
