#include "ec_gui_sdo.h"
#include "ec_gui_utils.h"
#define SDO_VALUE_COL 2

static const std::string expert_user_password="facility";

EcGuiSdo::EcGuiSdo(QWidget *parent):
QWidget(parent)
{
    _sdo_tree_wid = parent->findChild<QTreeWidget *>("SDO");
    _sdo_tree_wid->setEditTriggers(QAbstractItemView::EditKeyPressed| QAbstractItemView::SelectedClicked);
    
    connect(_sdo_tree_wid, SIGNAL(itemClicked(QTreeWidgetItem*, int)),this, SLOT(OnMouseClicked(QTreeWidgetItem*, int)));
    connect(_sdo_tree_wid, SIGNAL(itemExpanded(QTreeWidgetItem *)), this, SLOT(OnItemExapanded(QTreeWidgetItem *)));
    
    _sdo_item = nullptr;
    _sdo_column=-1;
    _old_sdo_value="";

    _expert_user = parent->findChild<QLineEdit *>("ExpertUserPass");
    connect(_expert_user, &QLineEdit::returnPressed,std::bind(&EcGuiSdo::ExpertUserPassChanged, this));
    _user_expert=false;


    _sdo_search = parent->findChild<QLineEdit *>("SDOSearch");
    connect(_sdo_search, &QLineEdit::textChanged,std::bind(&EcGuiSdo::SdoSearchChanged, this));
    _sdo_search_req="";

    _sdo_tree_wid->installEventFilter(this);
    _sdo_manager = parent->findChild<QDialogButtonBox *>("SDOManager");
    _sdo_manager->setEnabled(false);

    auto rescan_btn = _sdo_manager->button(QDialogButtonBox::Retry);
    rescan_btn->setText("Rescan");
    connect(rescan_btn, &QPushButton::released,this, &EcGuiSdo::onRescanSdoReleased);

    auto save_btn = _sdo_manager->button(QDialogButtonBox::Save);
    connect(save_btn, &QPushButton::released,this, &EcGuiSdo::onSaveSdoReleased);

    auto load_btn = _sdo_manager->button(QDialogButtonBox::Ignore);
    load_btn->setText("Load");
    connect(load_btn, &QPushButton::released,this, &EcGuiSdo::onLoadSdoReleased);

    auto restore_btn = _sdo_manager->button(QDialogButtonBox::RestoreDefaults);
    connect(restore_btn, &QPushButton::released,this, &EcGuiSdo::onRestoreSdoReleased);
}
      
EcGuiSdo::~EcGuiSdo(){}

void EcGuiSdo::restart_ec_gui_sdo(EcIface::Ptr client,SRD_SDO sdo_map)
{
    _client.reset();
    _client=client;
    
    _sdo_tree_wid->clear();
    _sdo_item_map.clear();
    
    _sdo_map.clear();
    _sdo_map = sdo_map;

    add_esc_sdo();

    search_sdo();
}

void EcGuiSdo::add_esc_sdo()
{
    for ( auto &[esc_id, rr_sdo] : _sdo_map ){
        QTreeWidgetItem * esc_item = new QTreeWidgetItem();
        std::string esc_id_name = "esc_id_"+std::to_string(esc_id);
        esc_item->setText(0,QString::fromStdString(esc_id_name));
        for ( auto &[sdo_name, sdo_value] : rr_sdo ){
            QTreeWidgetItem * sdo_entry = new QTreeWidgetItem();
            sdo_entry->setText(1,QString::fromStdString(sdo_name));
            sdo_entry->setText(2,QString::fromStdString(sdo_value));
            esc_item->addChild(sdo_entry);
            _sdo_item_map[esc_id][sdo_name]=sdo_entry;
        }
        _sdo_tree_wid->addTopLevelItem(esc_item);
    }
    _sdo_tree_wid->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    _sdo_tree_wid->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    _sdo_tree_wid->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
}

void EcGuiSdo::search_sdo()
{
    for(auto&[esc_id,name_item_map]: _sdo_item_map){
        for(auto&[sdo_name,sdo_item]: name_item_map){
            if(sdo_item->parent()->isExpanded()){
                sdo_item->setHidden(false);
                if(_sdo_search_req!=""){
                    if(sdo_item->text(1).indexOf(_sdo_search_req,0,Qt::CaseInsensitive) != 0){
                        sdo_item->setHidden(true);
                    }
                }
            }    
        }
    }
}

void EcGuiSdo::rescan_sdo()
{   
    for(auto&[esc_id,name_item_map]: _sdo_item_map){
        for(auto&[sdo_name,sdo_item]: name_item_map){
            if(sdo_item->parent()->isExpanded()){
                RD_SDO rd_sdo={sdo_name};
                RR_SDOS rr_sdo;
                if(_client->retrieve_rr_sdo(esc_id,rd_sdo,{},rr_sdo)){
                    if(rr_sdo.count(sdo_name)>0){
                        std::string sdo_value= rr_sdo[sdo_name];
                        sdo_item->setText(1,QString::fromStdString(sdo_name));
                        sdo_item->setText(2,QString::fromStdString(sdo_value));
                        _sdo_map[esc_id][sdo_name]=sdo_value;
                    }
                }
            }
        }
    }
}

void EcGuiSdo::flash_cmd(int value)
{
    bool flash_cmd_ok=true;
    int flash_cmd_ack=0x7800+value;
    std::string flash_cmd_str=std::to_string(value);
    std::string flash_cmd_ack_str=std::to_string(flash_cmd_ack);
    WR_SDO wr_sdo{std::make_tuple("flash_params_cmd",flash_cmd_str)};
    RD_SDO rd_sdo={"flash_params_cmd_ack"};

    for(auto&[esc_id,name_item_map]: _sdo_item_map){
        if(name_item_map.count("flash_params_cmd")>0 && name_item_map.count("flash_params_cmd_ack")>0){
            if(name_item_map["flash_params_cmd"]->parent()->isExpanded()){
                if(_client->set_wr_sdo(esc_id,{},wr_sdo)){
                    RR_SDOS rr_sdo;
                    if(_client->retrieve_rr_sdo(esc_id,rd_sdo,{},rr_sdo)){
                        if(rr_sdo["flash_params_cmd_ack"] != flash_cmd_ack_str){
                            flash_cmd_ok=false;
                        }
                    }
                }
            }
        }
    }
    
    rescan_sdo();

    QMessageBox msgBox;
    QString message="Success on flash command on all devices!";
    if(!flash_cmd_ok){
        message="Unsuccess on flash command for all or some devices";
    }
    msgBox.setText(message);
    msgBox.exec();
}

void EcGuiSdo::save_sdo_file()
{

    QDateTime date = QDateTime::currentDateTime();
    QString formatted_time = date.toString("dd_MM_yyyy__hh_mm_ss");

    for(auto&[esc_id,name_item_map]: _sdo_item_map){
        QFile *sdo_file;
        bool sdo_file_open=false;
        for(auto&[sdo_name,sdo_item]: name_item_map){
            if(sdo_item->parent()->isExpanded()){
                if(!sdo_file){
                    QString sdo_file_path =QDir::homePath()+"/"+sdo_item->parent()->text(0)+"_"+formatted_time+".csv";
                    sdo_file=new QFile(sdo_file_path);
                    sdo_file_open=sdo_file->open(QFile::WriteOnly|QFile::Truncate);
                }
                if(sdo_file_open){
                    QTextStream stream(sdo_file);
                    stream << sdo_item->text(1).toStdString().c_str() << "\t" 
                           << sdo_item->text(2).toStdString().c_str() << "\n";
                }
            }
        }
        if(sdo_file_open){
            sdo_file->close();
        }
    }
}


bool EcGuiSdo::eventFilter( QObject* o, QEvent* e )
{
    if( o == _sdo_tree_wid && e->type() == QEvent::KeyRelease){
        QKeyEvent *qkey = static_cast<QKeyEvent*>(e);
        if(qkey->key() == Qt::Key_Return){
            if(_sdo_column == SDO_VALUE_COL){
                QStringList esc_id_pieces = _sdo_item->parent()->text(0).split( "_" );
                
                uint32_t esc_id =esc_id_pieces.value(2).toUInt();
                std::string sdo_name= _sdo_item->text(1).toStdString();
                std::string new_sdo_value= _sdo_item->text(2).toStdString();
                
                RD_SDO rd_sdo = {sdo_name};
                WR_SDO wr_sdo{std::make_tuple(sdo_name,new_sdo_value)};
                
                bool write_read=false;
                if(_client->set_wr_sdo(esc_id,{},wr_sdo)){
                    RR_SDOS rr_sdo_new;
                    if(_client->retrieve_rr_sdo(esc_id,rd_sdo,{},rr_sdo_new)){
                        if(rr_sdo_new[sdo_name] == new_sdo_value){
                            write_read=true;
                        }
                    }
                }
                
                if(!write_read){
                    _sdo_item->setText(2,QString::fromStdString(_old_sdo_value));
                }

                
                _sdo_tree_wid->closePersistentEditor(_sdo_item,_sdo_column); // close actual editor
                _sdo_item=nullptr;
                _sdo_column=-1;
                _old_sdo_value="";
            }
        }
    }
    
    return false;  //You need to return false from eventFilter to let Qt to process the events which are not interesting for your eventFilter.
}
void EcGuiSdo::OnMouseClicked(QTreeWidgetItem* item, int column)
{
    if(!_user_expert){
        return;
    }

    if(_sdo_item != nullptr){
        _sdo_tree_wid->closePersistentEditor(_sdo_item,_sdo_column); // close old editor
    }

    _sdo_item = item;
    _sdo_column=column;
    _old_sdo_value= _sdo_item->text(2).toStdString();

    if((column == SDO_VALUE_COL ) && (_sdo_item->parent() != nullptr)){
        _sdo_tree_wid->openPersistentEditor(item,column);
    }
    else{
        _sdo_tree_wid->closePersistentEditor(item,column);
    } 
}

void EcGuiSdo::OnItemExapanded(QTreeWidgetItem* item)
{
    search_sdo();
}

void EcGuiSdo::ExpertUserPassChanged()
{
    QMessageBox msgBox;
    QString message="export user password incorrect!";
    if(_expert_user->text().toStdString()==expert_user_password){
        message="export user password correct!";
        _user_expert=true;
        _sdo_manager->setEnabled(true);
    }
    msgBox.setText(message);
    msgBox.exec();
}

void EcGuiSdo::SdoSearchChanged()
{
    _sdo_search_req = _sdo_search->text();
    search_sdo();
}

void EcGuiSdo::onRescanSdoReleased()
{
    rescan_sdo();
}

void EcGuiSdo::onSaveSdoReleased()
{
    flash_cmd(0x0012); //iit::ecat::Flash_cmd_type::SAVE_PARAMS_TO_FLASH
}

void EcGuiSdo::onLoadSdoReleased()
{
    flash_cmd(0x0034); //iit::ecat::Flash_cmd_type::LOAD_PARAMS_FROM_FLASH
}

void EcGuiSdo::onRestoreSdoReleased()
{
    flash_cmd(0x0056); //iit::ecat::Flash_cmd_type::LOAD_DEFAULT_PARAMS
}


