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

class EcGuiFirmware : public QWizard
{
    Q_OBJECT
public:
    
    EcGuiFirmware(QWidget * parent = 0){
    
        _firmware_files_tree = new QTreeWidget();
        _firmware_files_tree->setColumnCount(2);
        _firmware_files_tree->setHeaderLabels({"Firmware files",""});
        _firmware_files_tree->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
        _firmware_files_tree->header()->setMinimumSectionSize(0);
        _firmware_files_tree->header()->resizeSection(1,0);
        _firmware_files_tree->header()->setStretchLastSection(false); // QTreeWidget problem for resizing added another column!!


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

        QVBoxLayout *layout = new QVBoxLayout;
        layout->addWidget(_firmware_files_tree);
        layout->addWidget(firmware_manager);
        QHBoxLayout *layout_page = new QHBoxLayout;
        QLabel *firmware_image=new QLabel;
        QPixmap firmware_logo_pic;
        firmware_logo_pic.load(":/icon/firmware.png");
        firmware_image->setPixmap(firmware_logo_pic);
        layout_page->addWidget(firmware_image);
        layout_page->addLayout(layout);

        QWizardPage *firmware_wizard_page = new QWizardPage();
        firmware_wizard_page->setTitle("Firmware update wizard");
        firmware_wizard_page->setLayout(layout_page);
        this->addPage(firmware_wizard_page);
        this->setFixedSize(layout_page->geometry().width(),layout_page->geometry().height());
        this->setWindowFlags(Qt::Window);
        this->setAttribute(Qt::WA_QuitOnClose,false);
    };

    ~EcGuiFirmware(){
    };

    std::vector<QPushButton *> get_firmware_update_btns(){
        return _firmware_update_btns;
    }

    QStringList get_files_list(){
        return _files_list;
    }

    void run_wizard(){
        if(!this->isVisible()){
            _firmware_files_tree->clear();
            _files_list.clear();
        }
        this->close();
        this->setWindowState(Qt::WindowActive);
        this->show();
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

class EcGuiStart : public QMainWindow
{
    Q_OBJECT

public:

    explicit EcGuiStart(QWidget *parent = nullptr);

    ~EcGuiStart();

private slots:
    void onStartEtherCATSystem();
    void onStopEtherCATSystem();
    void onScanDeviceReleased();
    void ExpertUserPassChanged();
    void onFirmwareUpdateReleased();
    void onFirmwareUpdateCopyFiles(); 
    void onFirmwareUpdateOpenConfig(); 
    void onFirmwareUpdateStart();
    void onFirmwarewizardClosed(int ret);
    
private:
  
  EcGuiWrapper::ec_wrapper_info_t _ec_wrapper_info;
  EcGuiWrapper::Ptr _ec_gui_wrapper;
  EcGuiNet::Ptr _ec_gui_net;
  QPushButton *_ec_sys_start,*_ec_sys_stop,*_firmware_update_btn;
  
  QTreeWidget * _net_tree_wid;
  QLineEdit *_expert_user;
  bool _etherCAT_sys_started;
  EcGuiFirmware *_firmware_update_wizard;
  
  void error_on_scannig();
  void restart_gui();
  void add_device();
  void scan_device();
  void read_sdo_info(const int32_t device_id,
                     const std::vector<std::string> sdo_name,
                     std::vector<float> &sdo_info);
  void setup_motor_device(int32_t device_id,int32_t device_type);
  void clear_device();
  void clear_gui();
  bool create_ec_iface();
  bool stopping_ec_sys();
  void stopping_client();
};

#endif // EC_GUI_START_H
