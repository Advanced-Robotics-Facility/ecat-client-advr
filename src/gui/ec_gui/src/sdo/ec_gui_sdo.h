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
        
        void restart_ec_gui_sdo(EcIface::Ptr client,SRD_SDO sdo_map);
 
public slots:
        void OnMouseClicked(QTreeWidgetItem* item, int column);
        
protected:
        bool eventFilter( QObject* o, QEvent* e );

private:
        EcIface::Ptr _client;
        QTreeWidget *_sdo_tree_wid;
        QTreeWidgetItem* _sdo_item;
        int _sdo_column;
        SRD_SDO _sdo_map;
        std::string _old_sdo_value;
        
        void add_esc_sdo();
        void rescan_esc_sdo();
};


#endif // EC_GUI_SDO_H


