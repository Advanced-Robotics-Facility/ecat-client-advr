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
    
    _sdo_item = nullptr;
    _sdo_column=-1;
    _old_sdo_value="";

    _expert_user = parent->findChild<QLineEdit *>("ExpertUserPass");
    connect(_expert_user, &QLineEdit::returnPressed,
    std::bind(&EcGuiSdo::ExpertUserPassChanged, this));
    
    _sdo_tree_wid->installEventFilter(this);
    _sdo_manager = parent->findChild<QDialogButtonBox *>("SDOManager");
    _sdo_manager->setEnabled(false);

    _user_expert=false;
}
      
EcGuiSdo::~EcGuiSdo(){}

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

void EcGuiSdo::restart_ec_gui_sdo(EcIface::Ptr client,SRD_SDO sdo_map)
{
    _client.reset();
    _client=client;
    
    _sdo_tree_wid->clear();
    
    _sdo_map.clear();
    _sdo_map = sdo_map;

    add_esc_sdo();
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
        }
        _sdo_tree_wid->addTopLevelItem(esc_item);
    }
    _sdo_tree_wid->resizeColumnToContents(0);
}

void EcGuiSdo::rescan_esc_sdo()
{
    _sdo_map.clear();
    
    for ( auto &[esc_id, rr_sdo] : _sdo_map ){
        RR_SDOS new_rr_sdo_info;
        _client->retrieve_all_sdo(esc_id,new_rr_sdo_info);
        _sdo_map[esc_id] = new_rr_sdo_info;
    }
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