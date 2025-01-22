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
        void OnItemExapanded(QTreeWidgetItem* item);
        void ExpertUserPassChanged();
        void SdoSearchChanged();
        void onSaveSdoReleased();
        void onRescanSdoReleased();
        void onRestoreSdoReleased();
        void onLoadSdoReleased();
        
protected:
        bool eventFilter( QObject* o, QEvent* e );

private:
        EcIface::Ptr _client;
        QTreeWidget *_sdo_tree_wid;
        QTreeWidgetItem *_sdo_item;
        QLineEdit *_expert_user, *_sdo_search;
        QDialogButtonBox *_sdo_manager;
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
};


#endif // EC_GUI_SDO_H


