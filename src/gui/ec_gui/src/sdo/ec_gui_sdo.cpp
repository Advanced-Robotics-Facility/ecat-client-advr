#include "ec_gui_sdo.h"
#include "ec_gui_utils.h"
#define SDO_VALUE_COL 2

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
    _user_expert=false;

    _sdo_search = parent->findChild<QLineEdit *>("SDOSearch");
    connect(_sdo_search, &QLineEdit::textChanged,std::bind(&EcGuiSdo::SdoSearchChanged, this));
    _sdo_search_req="";

    _sdo_tree_wid->installEventFilter(this);
    _sdo_flash_manager = parent->findChild<QDialogButtonBox *>("SDOFlashManager"); 
    _sdo_flash_manager->setEnabled(false);

    auto escid_sdo_manager = parent->findChild<QDialogButtonBox *>("EscIDSdoManager");

    auto yes_all_btn = escid_sdo_manager->button(QDialogButtonBox::YesToAll);
    connect(yes_all_btn, &QPushButton::released,this, &EcGuiSdo::onYesToAllSdoReleased);
    auto no_all_btn = escid_sdo_manager->button(QDialogButtonBox::NoToAll);
    connect(no_all_btn, &QPushButton::released,this, &EcGuiSdo::onNoToAllSdoReleased);

    auto save_btn = _sdo_flash_manager->button(QDialogButtonBox::Save);
    connect(save_btn, &QPushButton::released,this, &EcGuiSdo::onSaveSdoReleased);

    auto load_btn = _sdo_flash_manager->button(QDialogButtonBox::Retry);
    load_btn->setText("Load");
    connect(load_btn, &QPushButton::released,this, &EcGuiSdo::onLoadSdoReleased);

    auto restore_btn = _sdo_flash_manager->button(QDialogButtonBox::Ignore);
    restore_btn->setText("Restore default");
    connect(restore_btn, &QPushButton::released,this, &EcGuiSdo::onRestoreSdoReleased);

    auto open_file_btn = _sdo_flash_manager->button(QDialogButtonBox::RestoreDefaults);
    open_file_btn->setText("Open SDO file");
    connect(open_file_btn, &QPushButton::released,this, &EcGuiSdo::onOpenFileSdoReleased);
    

    _sdo_manager = parent->findChild<QDialogButtonBox *>("SDOManager");

    auto rescan_btn = _sdo_manager->button(QDialogButtonBox::Retry);
    rescan_btn->setText("Rescan");
    connect(rescan_btn, &QPushButton::released,this, &EcGuiSdo::onRescanSdoReleased);

    auto save_file_btn = _sdo_manager->button(QDialogButtonBox::RestoreDefaults);
    save_file_btn->setText(" Save  SDO file");
    connect(save_file_btn, &QPushButton::released,this, &EcGuiSdo::onSaveFileSdoReleased);
}
      
EcGuiSdo::~EcGuiSdo(){}

void EcGuiSdo::restart_ec_gui_sdo(EcIface::Ptr client,SRD_SDO sdo_map)
{
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
        esc_item->setCheckState(0,_esc_id_state);
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

void EcGuiSdo::onYesToAllSdoReleased()
{   
    _esc_id_state=Qt::Checked;
    esc_id_check();
}

void EcGuiSdo::onNoToAllSdoReleased()
{
    _esc_id_state=Qt::Unchecked;
    esc_id_check();
}

void EcGuiSdo::esc_id_check()
{
    for(auto&[esc_id,name_item_map]: _sdo_item_map){
        auto sdo_item=name_item_map.begin()->second;
        sdo_item->parent()->setCheckState(0,_esc_id_state);
    }
}

void EcGuiSdo::search_sdo()
{
    for(auto&[esc_id,name_item_map]: _sdo_item_map){
        for(auto&[sdo_name,sdo_item]: name_item_map){
            if(sdo_item->parent()->isExpanded()){
                sdo_item->setHidden(false);
                if(_sdo_search_req!=""){
                    if(sdo_item->text(1).indexOf(_sdo_search_req,0,Qt::CaseInsensitive)==-1){
                        sdo_item->setHidden(true);
                    }
                }
            }    
        }
    }
}

bool EcGuiSdo::check_client_setup()
{
    bool ret=false;
    QMessageBox msgBox;
    if(_client == nullptr){
        msgBox.critical(this,msgBox.windowTitle(),
                        tr("EtherCAT client not setup"
                           ",please scan device button.\n"));
    }
    else{
        if(!_client->get_client_status().run_loop){
            msgBox.critical(this,msgBox.windowTitle(),
                tr("EtherCAT Client loop is not running state"
                    ",please press scan device button.\n"));
        }
        else{
            ret = true;
        }
    }
    
    return ret;
}

void EcGuiSdo::rescan_sdo()
{   
    if(check_client_setup()){
        bool try_rescan_cmd=false;
        bool rescan_cmd_ok=true;
        QString cmd_name="";
        for(auto&[esc_id,name_item_map]: _sdo_item_map){
            for(auto&[sdo_name,sdo_item]: name_item_map){
                if(sdo_item->parent()->checkState(0)==Qt::Checked){
                    RD_SDO rd_sdo={sdo_name};
                    RR_SDOS rr_sdo;
                    cmd_name="rescan";
                    try_rescan_cmd=true;
                    if(_client->retrieve_rr_sdo(esc_id,rd_sdo,{},rr_sdo)){
                        if(rr_sdo.count(sdo_name)>0){
                            std::string sdo_value= rr_sdo[sdo_name];
                            sdo_item->setText(1,QString::fromStdString(sdo_name));
                            sdo_item->setText(2,QString::fromStdString(sdo_value));
                            _sdo_map[esc_id][sdo_name]=sdo_value;
                        }else{
                            rescan_cmd_ok=false;
                        }
                    }else{
                        rescan_cmd_ok=false;
                    }
                }
            }
        }

        cmd_feedback(try_rescan_cmd,rescan_cmd_ok,cmd_name);
    }
}

void EcGuiSdo::flash_cmd(int value)
{
    if(check_client_setup()){
        bool flash_cmd_ok=true;
        bool try_flash_cmd=false;
        int flash_cmd_ack=0x7800+value;
        std::string flash_cmd_str=std::to_string(value);
        std::string flash_cmd_ack_str=std::to_string(flash_cmd_ack);
        WR_SDO wr_sdo{std::make_tuple("flash_params_cmd",flash_cmd_str)};
        RD_SDO rd_sdo={"flash_params_cmd_ack"};

        QString cmd_name="";
        bool flash_cmd_done=false;
        for(auto&[esc_id,name_item_map]: _sdo_item_map){
            auto sdo_item=name_item_map.begin()->second;
            if(sdo_item->parent()->checkState(0)==Qt::Checked){
                cmd_name="flash";
                if(name_item_map.count("flash_params_cmd")>0 && name_item_map.count("flash_params_cmd_ack")>0){
                    try_flash_cmd=true;
                    if(_client->set_wr_sdo(esc_id,{},wr_sdo)){
                        RR_SDOS rr_sdo;
                        flash_cmd_done=true;
                        if(_client->retrieve_rr_sdo(esc_id,rd_sdo,{},rr_sdo)){
                            if(rr_sdo["flash_params_cmd_ack"] != flash_cmd_ack_str){
                                flash_cmd_ok=false;
                            }
                        }
                    }
                }
            }
        }

        cmd_feedback(try_flash_cmd,flash_cmd_ok,cmd_name);
        
        if(flash_cmd_done){
            rescan_sdo();
        }
    }
}

void EcGuiSdo::save_sdo_file()
{

    QDateTime date = QDateTime::currentDateTime();
    QString formatted_time = date.toString("dd_MM_yyyy__hh_mm_ss");
    bool try_save_cmd=false;
    bool save_cmd_ok=true;
    QString cmd_name="";
    for(auto&[esc_id,name_item_map]: _sdo_item_map){
        QFile *sdo_file=nullptr;
        bool sdo_file_open=false;
        for(auto&[sdo_name,sdo_item]: name_item_map){
            if(sdo_item->parent()->checkState(0)==Qt::Checked){
                cmd_name="save";
                try_save_cmd=true;
                if(!sdo_file){
                    QString sdo_file_path =QDir::homePath()+"/"+sdo_item->parent()->text(0)+"_"+formatted_time+".csv";
                    sdo_file=new QFile(sdo_file_path);
                    sdo_file_open=sdo_file->open(QFile::WriteOnly|QFile::Truncate);
                }
                if(sdo_file_open){
                    QTextStream stream(sdo_file);
                    stream << sdo_item->text(1).toStdString().c_str() << "\t" 
                           << sdo_item->text(2).toStdString().c_str() << "\n";
                }else{
                    save_cmd_ok=false;
                }
            }
        }

        if(sdo_file_open){
            sdo_file->close();
        }
    }

    cmd_feedback(try_save_cmd,save_cmd_ok,cmd_name);
}

void EcGuiSdo::open_sdo_file()
{
    if(check_client_setup()){
        std::vector<uint32_t> esc_id_v;
        for(auto&[esc_id,name_item_map]: _sdo_item_map){
            auto sdo_item=name_item_map.begin()->second;
            if(sdo_item->parent()->checkState(0)==Qt::Checked){
                esc_id_v.push_back(esc_id);
            }
        }
        bool try_write_cmd=false;
        bool new_write_ok=true;
        QString cmd_name="";
        if(!esc_id_v.empty()){
            EcGuiSdoWizard sdo_wizard;
            auto new_wr_sdo = sdo_wizard.run_sdo_wizard();
            if(new_wr_sdo.empty()){
                return;
            }

            cmd_name="write";
            try_write_cmd=true;
            for(auto &esc_id: esc_id_v){
                if(!_client->set_wr_sdo(esc_id,{},new_wr_sdo)){
                    new_write_ok=false;
                }
            }
        }

        cmd_feedback(try_write_cmd,new_write_ok,cmd_name);

        if(new_write_ok){
            rescan_sdo();
        }
    }
}

void EcGuiSdo::cmd_feedback(bool try_cmd,bool cmd_ok,QString cmd_name)
{   
    QMessageBox msgBox;
    QString message="No device selected, please select at least one";
    if(try_cmd){
        message="Success on "+cmd_name+" for all devices";
        if(!cmd_ok){
            message="Unsuccess on "+cmd_name+" for all or some devices";
        }
    }
    else{
        if(cmd_name!=""){
            message="No "+cmd_name+" command executed";
        }
    }
    msgBox.setText(message);
    msgBox.exec();
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
            //retun true; ??
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

void EcGuiSdo::set_expert_user()
{
    _user_expert=true;
    _sdo_flash_manager->setEnabled(true);
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

void EcGuiSdo::onSaveFileSdoReleased()
{
    save_sdo_file();
}

void EcGuiSdo::onOpenFileSdoReleased()
{
    open_sdo_file();
}


