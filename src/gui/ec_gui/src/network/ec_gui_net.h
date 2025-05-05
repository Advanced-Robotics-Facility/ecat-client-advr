#ifndef EC_GUI_NET_H
#define EC_GUI_NET_H

#include <QtUiTools>
#include <QWidget>

#include "ec_gui_terminal.h"

class EcGuiFirmware : public QWidget
{
    Q_OBJECT
public:
    
    EcGuiFirmware(QWidget * parent = 0){
    
        _firmware_files_tree = new QTreeWidget();
        _firmware_files_tree->setColumnCount(1);
        _firmware_files_tree->setHeaderLabels({"Firmware files"});

        auto firmware_manager = new QDialogButtonBox(QDialogButtonBox::RestoreDefaults | 
                                                     QDialogButtonBox::Ignore |
                                                     QDialogButtonBox::Retry |
                                                     QDialogButtonBox::Save );

        auto select_files = firmware_manager->button(QDialogButtonBox::RestoreDefaults);
        select_files->setText("Select all bin or config files");
        connect(select_files, &QPushButton::released,this, &EcGuiFirmware::onSelectFilesReleased);
        
        auto copy_files_btn = firmware_manager->button(QDialogButtonBox::Ignore);
        copy_files_btn->setText("Copy files to embedded PC");
        _firmware_update_btns.push_back(copy_files_btn);
        
        auto open_config_btn = firmware_manager->button(QDialogButtonBox::Retry);
        open_config_btn->setText("Open firmware configuration file");
        _firmware_update_btns.push_back(open_config_btn);

        auto start_update_btn = firmware_manager->button(QDialogButtonBox::Save);
        start_update_btn->setText("Start firmware update");
        _firmware_update_btns.push_back(start_update_btn);

        QWizardPage *firmware_wizard_page = new QWizardPage();
        firmware_wizard_page->setTitle("Firmware update wizard");
        QVBoxLayout *layout = new QVBoxLayout;
        layout->addWidget(_firmware_files_tree);
        layout->addWidget(firmware_manager);
        firmware_wizard_page->setLayout(layout);
        _firmware_wizard.addPage(firmware_wizard_page);
    };

    ~EcGuiFirmware(){};

    std::vector<QPushButton *> get_firmware_update_btns(){
        return _firmware_update_btns;
    }

    QStringList get_files_list(){
        return _files_list;
    }

    void run_wizard(){
        _firmware_files_tree->clear();
        _files_list.clear();
        _firmware_wizard.exec();
    };

private slots:
    void onSelectFilesReleased(){

        QStringList fileNames = QFileDialog::getOpenFileNames(
            this,
            "Select one or more files",
            QDir::homePath(),
            "Bin files (*.bin);;Config files (*.csv)");
        for(const auto& file_path:fileNames){
            if(!file_listed(file_path)){
                QTreeWidgetItem * file_item = new QTreeWidgetItem();
                file_item->setText(0,file_path);
                _firmware_files_tree->addTopLevelItem(file_item);
                _files_list.append(file_path);
            }
        }
    };

private:
    QWizard _firmware_wizard;
    QTreeWidget *_firmware_files_tree;
    QStringList _files_list;
    std::vector<QPushButton *> _firmware_update_btns;
    bool file_listed(const QString &file_name){
        for(int i=0;i<_firmware_files_tree->topLevelItemCount();i++){
            auto topLevel =_firmware_files_tree->topLevelItem(i);
            if(topLevel->text(0)==file_name){
                return true;
            }
        }
        return false;
   };
};

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
    void set_expert_user();

private slots:
    void ec_master_processFinished(int, QProcess::ExitStatus);
    void server_processFinished(int, QProcess::ExitStatus);
    void OnMouseClicked(QTreeWidgetItem* item, int column);
    void OnPasswordEntered();
    void OnPasswordChanged();
    void OnProtocolChanged();
    void onFirmwareUpdateReleased();
    void onFirmwareUpdateCopyFiles(); 
    void onFirmwareUpdateOpenConfig(); 
    void onFirmwareUpdateStart();


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
  QPushButton *_firmware_update_btn;
  EcGuiFirmware _firmware_update_wizard;
  bool _open_config_file=false;
  
  bool create_ssh_cmd(QProcess *process,QString& stdout);
  QString find_running_process(QProcess * process,QString bin_name,QString& stdout);
  QString find_process(QProcess * process,QString bin_name,QString& stdout);
  void start_process(QProcess *process,QString bin_file_path,QString option);
  void kill_process(QProcess *process,QString bin_name,QString& stdout);
  
  void kill_view_process(const QString &terminal_pid);
  void view_process(const QString &file_path,QString &terminal_pid);
  void start_master_process(const QString &bin_file_name,const QString &option);
  void ec_master_readyStdO();
  void view_master_process();
  void server_readyStdO();
  void view_server_process();
  void set_ec_network();    
  void view_gui_process();
  bool copy_config_file();
  void save_config_file();
};
#endif // EC_GUI_NET_H
