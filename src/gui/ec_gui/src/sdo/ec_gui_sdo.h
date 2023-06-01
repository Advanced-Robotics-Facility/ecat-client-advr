#ifndef EC_GUI_SDO_H
#define EC_GUI_SDO_H

#include <QtUiTools>
#include <QWidget>

#include "ec_utils.h"

class EcGuiSdo : public QWidget
{
     Q_OBJECT
     
public:
    
        typedef std::shared_ptr<EcGuiSdo> Ptr;

        EcGuiSdo(EcIface::Ptr client,
                 QWidget * parent = 0);

        ~EcGuiSdo();

        void restart_ec_gui_sdo(SRD_SDO sdo_map);
        void set_internal_sdo_map(SRD_SDO internal_sdo_map);

private:
        EcIface::Ptr _client;
        QTreeWidget *_sdo_tree_wid;
        SRD_SDO _sdo_map;
        SRD_SDO _internal_sdo_map;
        
        void add_esc_sdo();
};

#endif // EC_GUI_SDO_H


