#include "ec_gui_sdo.h"
#include "ec_gui_utils.h"

EcGuiSdo::EcGuiSdo(EcIface::Ptr client, QWidget *parent):
QWidget(parent),
_client(client)
{
    _sdo_tree_wid = parent->findChild<QTreeWidget *>("SDO");
    
    auto sdo_manager = parent->findChild<QDialogButtonBox *>("SDOManager");
}
      
EcGuiSdo::~EcGuiSdo(){}

void EcGuiSdo::restart_ec_gui_sdo(SRD_SDO sdo_map)
{
    _sdo_tree_wid->clear();
    
    _sdo_map.clear();
    _sdo_map = sdo_map;

    add_esc_sdo();
}

void EcGuiSdo::set_internal_sdo_map(SRD_SDO internal_sdo_map)
{
    _internal_sdo_map.clear();
    _internal_sdo_map=internal_sdo_map;
}

void EcGuiSdo::add_esc_sdo()
{
    if(_sdo_map.empty())
    {
        _sdo_map = _internal_sdo_map;
    }
    
    for ( auto &[esc_id, rr_sdo] : _sdo_map )
    {
        QTreeWidgetItem * esc_item = new QTreeWidgetItem();
        std::string esc_id_name = "esc_id_"+std::to_string(esc_id);
        esc_item->setText(0,QString::fromStdString(esc_id_name));
        
        for ( auto &[sdo, value] : rr_sdo )
        {
            QTreeWidgetItem * sdo_value = new QTreeWidgetItem();
            sdo_value->setText(1,QString::fromStdString(sdo));
            std::string value_str = std::to_string(value);
            sdo_value->setText(2,QString::fromStdString(value_str));
            sdo_value->setFlags(sdo_value->flags() | Qt::ItemIsEditable);
            esc_item->addChild(sdo_value);
        }
        _sdo_tree_wid->addTopLevelItem(esc_item);
    }
    _sdo_tree_wid->resizeColumnToContents(0);
}
