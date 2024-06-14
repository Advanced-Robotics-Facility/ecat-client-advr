#ifndef EC_GUI_NET_H
#define EC_GUI_NET_H

#include <QtUiTools>
#include <QWidget>

#include "ec_gui_terminal.h"


class EcGuiNet : public QThread
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
    bool check_network();
    ec_net_info_t get_net_setup();

signals:

   void start_timer();

public slots:
    void ec_master_processFinished(int, QProcess::ExitStatus);
    void server_processFinished(int, QProcess::ExitStatus);
    void OnMouseDoubleClicked(QTreeWidgetItem* item, int column);
    void OnMouseClicked(QTreeWidgetItem* item, int column);
    void OnPasswordEntered();
    void OnPasswordChanged();
    void OnProtocolChanged();
    void update_std_out();
    void slot_timer_start();

protected:
    bool eventFilter( QObject* o, QEvent* e );
    void run();
    
private:

  QTreeWidget * _net_tree_wid;
  QTreeWidgetItem* _net_item;
  QLineEdit *_password;
  int _net_column;
  
  QProcess *_ec_master_process,*_server_process;
  QStringList _ssh_command;
  QString _ec_master_stoud,_server_stdout;
  EcGuiTerminal::Ptr _ec_master_terminal, _server_terminal;
  QString _server_hostname,_server_ip,_server_port,_server_protocol,_server_pwd;
  QTimer *_recv_std_out;
  
  QComboBox * _protocol_combobox;
  
  bool create_ssh_cmd(QProcess *process,QString& stdout);
  QString find_running_process(QProcess * process,QString bin_name,QString& stdout);
  QString find_process(QProcess * process,QString bin_name,QString& stdout);
  void start_process(QProcess *process,QString bin_file_path,QString option);
  void kill_process(QProcess *process,QString bin_name,QString& stdout);
  
  void ec_master_readyStdO();
  void server_readyStdO();
  void set_ec_network();      
};
#endif // EC_GUI_NET_H
