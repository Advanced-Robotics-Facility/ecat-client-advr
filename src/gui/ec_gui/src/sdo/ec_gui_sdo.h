#ifndef EC_GUI_SDO_H
#define EC_GUI_SDO_H

#include <QtUiTools>
#include <QWidget>

#include "utils/ec_utils.h"

class EcGuiSdo : public QWidget
{
     Q_OBJECT
     
public:
    
        typedef std::shared_ptr<EcGuiSdo> Ptr;

        EcGuiSdo(QWidget * parent = 0);

        ~EcGuiSdo();

        void set_expert_user();
        void restart_ec_gui_sdo(EcIface::Ptr client,SRD_SDO sdo_map);
 
private slots:
        void OnMouseClicked(QTreeWidgetItem* item, int column);
        void OnItemExapanded(QTreeWidgetItem* item);
        void SdoSearchChanged();
        void onSaveSdoReleased();
        void onRescanSdoReleased();
        void onRestoreSdoReleased();
        void onLoadSdoReleased();
        void onSaveFileSdoReleased();
        void onOpenFileSdoReleased();
        
protected:
        bool eventFilter( QObject* o, QEvent* e );

private:
        EcIface::Ptr _client;
        QTreeWidget *_sdo_tree_wid;
        QTreeWidgetItem *_sdo_item;
        QLineEdit *_sdo_search;
        QDialogButtonBox *_sdo_manager,*_sdo_flash_manager;
        int _sdo_column;
        std::map<uint32_t,std::map<std::string,QTreeWidgetItem *>> _sdo_item_map;
        SRD_SDO _sdo_map;
        std::string _old_sdo_value;
        bool _user_expert;
        QString _sdo_search_req;
        
        void add_esc_sdo();
        void search_sdo();
        void rescan_sdo();
        void flash_cmd(int value);
        void save_sdo_file();
        void open_sdo_file();
        void cmd_feedback(bool try_cmd,bool cmd_ok,QString cmd_name);
};

class EcGuiSdoWizard : public QWidget
{
  Q_OBJECT

public:
        EcGuiSdoWizard(QWidget * parent = 0){
            _sdo_wizard_tree = new QTreeWidget();
            _sdo_wizard_tree->setColumnCount(2);
            _sdo_wizard_tree->setHeaderLabels({"SDO Name","SDO Value"});
            _sdo_wizard_tree->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
            _sdo_wizard_tree->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
            _sdo_wizard_tree->setMinimumSize(600,600);

            auto sdo_sel_manager = new QDialogButtonBox(QDialogButtonBox::YesToAll| QDialogButtonBox::NoToAll);
            sdo_sel_manager->setLayoutDirection(Qt::RightToLeft);
            auto yes_all_btn = sdo_sel_manager->button(QDialogButtonBox::YesToAll);
            connect(yes_all_btn, &QPushButton::released,this, &EcGuiSdoWizard::onYesToAllSdoReleased);
            auto no_all_btn = sdo_sel_manager->button(QDialogButtonBox::NoToAll);
            connect(no_all_btn, &QPushButton::released,this, &EcGuiSdoWizard::onNoToAllSdoReleased);

            QWizardPage *sdo_wizard_page = new QWizardPage();
            sdo_wizard_page->setTitle("Calibration Wizard");
            QVBoxLayout *layout = new QVBoxLayout;
            layout->addWidget(_sdo_wizard_tree);
            layout->addWidget(sdo_sel_manager);

            QHBoxLayout *layout_page = new QHBoxLayout;
            QLabel *calib_image=new QLabel;
            QPixmap calib_logo_pic;
            calib_logo_pic.load(":/icon/calibration.png");
            calib_image->setPixmap(calib_logo_pic);
            layout_page->addWidget(calib_image);
            layout_page->addLayout(layout);
            sdo_wizard_page->setLayout(layout_page);
            _sdo_wizard.addPage(sdo_wizard_page);
            _sdo_wizard.setFixedSize(layout_page->geometry().width(),layout_page->geometry().height());
            //_sdo_wizard.setWindowFlags(Qt::Window);
        };

        WR_SDO run_sdo_wizard(){
            _write_new_sdo.clear();
            if(init_wizard()){
                if(_sdo_wizard.exec()){
                    fill_write_sdo();
                }
            }
            return _write_new_sdo;
        };


        ~EcGuiSdoWizard(){};

private slots:
        void onYesToAllSdoReleased(){
           _sdo_check_state=Qt::Checked;
           sdo_checking();
        };

        void onNoToAllSdoReleased(){
           _sdo_check_state=Qt::Unchecked;
           sdo_checking();
        };
private:
        QTreeWidget *_sdo_wizard_tree;
        QWizard _sdo_wizard;
        Qt::CheckState _sdo_check_state=Qt::Unchecked;
        WR_SDO _write_new_sdo;

        bool init_wizard(){
           QFileDialog dialog(this);
           dialog.setFileMode(QFileDialog::AnyFile);
           dialog.setNameFilter(tr(".csv (*.csv)"));
           QStringList fileNames;
           if (dialog.exec()){
                fileNames = dialog.selectedFiles();
           }
           if(!fileNames.empty()){
                QFile *sdo_file=new QFile(fileNames[0]);
                if(sdo_file->open(QFile::ReadOnly)){
                   QTextStream in(sdo_file);
                   while (!in.atEnd()){
                        QStringList split_line = in.readLine().split("\t");
                        add_content(split_line);
                   }
                   sdo_file->close();
                   return true;
                }
           }
           return false;
        };
        
        void add_content(QStringList &list){
             if(!list.empty()){
                int i=0;
                QTreeWidgetItem * sdo_wizard_item = new QTreeWidgetItem();
                for(auto &value:list){
                   sdo_wizard_item->setText(i,value);
                   sdo_wizard_item->setFlags(sdo_wizard_item->flags() | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
                   sdo_wizard_item->setCheckState(0,Qt::Unchecked);
                   i++;
                }
                _sdo_wizard_tree->addTopLevelItem(sdo_wizard_item);
             }
        };

        void sdo_checking(){
             for(int i=0;i<_sdo_wizard_tree->topLevelItemCount();i++){
                 auto topLevel =_sdo_wizard_tree->topLevelItem(i);
                 topLevel->setCheckState(0,_sdo_check_state);
             }
        };
         void fill_write_sdo(){
             for(int i=0;i<_sdo_wizard_tree->topLevelItemCount();i++){
                 auto topLevel =_sdo_wizard_tree->topLevelItem(i);
                 if(topLevel->checkState(0)==Qt::Checked){
                     std::string sdo_name = topLevel->text(0).toStdString();
                     std::string sdo_value = topLevel->text(1).toStdString();
                    _write_new_sdo.push_back(std::make_tuple(sdo_name,sdo_value));
                 }
             }
        };
};

#endif // EC_GUI_SDO_H


