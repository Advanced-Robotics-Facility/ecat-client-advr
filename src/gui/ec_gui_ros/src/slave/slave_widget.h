#ifndef SLAVE_WIDGET_H
#define SLAVE_WIDGET_H

#include "sliders_widget.h"
#include "slave_cmd_widget.h"

#include <XBotInterface/Utils.h>
#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <robot_monitoring/custom_qt_widget.h>

#include "ec_msgs/SlaveDescription.h"


enum REPL_STATUS : int {
    NOT_STARTED=               0,
    PRE_OPERATIONAL=           1,
    OPERATIONAL=               2,
};


class SlaveWidget : public XBot::Ui::CustomQtWidget
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "robot_monitoring.CustomQtWidget.1.0.0" FILE "slave_widget.json")
    Q_INTERFACES(XBot::Ui::CustomQtWidget)

public:

    SlaveWidget();

    bool init(Args&) override;

    QString name() override;

    void update() override;

    ~SlaveWidget() override;

    void setAmpereTab();
    void setLimitsTab();
    void removeOperationalCmd();
    void addOperationalCmd();

    void onSetCmdReleased();
    void onGetCmdReleased();

    void onApplyCmdReleased();

    void  XBotCoreNodePresence();

    void onSaveCmdReleased();
    void onResetCmdReleased();
    void onRestoreCmdReleased();
    void onStartCmdReleased();
    void onClearCmdReleased();

    /************** GETTING AREA *************/

    std::string getFieldType() const;
    std::string getModeType() const;
    std::string getMotorCtrlType() const;
    std::string getTrjType() const;
    QStackedWidget * get_stacked_widget();
    std::unordered_map<std::string,QTabWidget *>   get_tab_map_widget();
    bool get_xbotcore_node_presence();
    int get_numb_chains();
    std::map<int ,std::map<int,std::map<std::string, SliderWidget *>>> get_setup_slaves();
    std::vector<ec_msgs::SlaveDescription> getSlavesInfo();
    std::string get_ec_client_name();
    void set_internal_repl_sts(REPL_STATUS sts);
    REPL_STATUS get_internal_repl_sts();

    /************** GETTING AREA *************/


public slots:
    void readCommand();
    void readModeType();
    void readMotorCtrlType();
    void readTrjType();

private:
    void try_construct();
    void construct();
    void on_reload();

    /************** COMPONENTS AREA *************/

    QComboBox * _fieldtype_combobox;
    QComboBox * _mode_type_combobox,*_motor_ctrl_combobox,*_trj_combobox;

    QRadioButton * _setbtn;
    QRadioButton * _getbtn;

    QGroupBox * _chain_sel_group;
    QComboBox * _chain_select;

    QStackedWidget * _wid_stack,*_wid_stack2,*_wid_stack3;
    QStatusBar * _status;
    QTabWidget * _wid_main_tab;
    QTextEdit *_msgTxt;
    QPushButton * _applybtn;

    QDialogButtonBox *_flash_cmd_manager,*_trj_cmd_manager;
    QPushButton * _savebtn,*_resetbtn,*_restorebtn;
    QPushButton * _applytbtn,*_cancelbtn;



    /************** COMPONENTS AREA *************/

    bool _load_success;
    bool _xbotcorenodepresent=false;

    int _ch_numb;
    int _set_type;
    int _numb_slider;

    XBot::XBotInterface::Ptr _robot;
    std::unordered_map<std::string,QTabWidget *> _tab_wid_map,_tab2_wid_map,_tab3_wid_map;
    std::vector<ec_msgs::SlaveDescription> _slaves_descr_vector;
    std::map<int ,std::map<int,std::map<std::string, SliderWidget *>>> _setup_slaves,_setup_motor_ctrl,_setup_trj;
    std::map<std::string,std::vector<std::string>> _groups_map;

    std::string _ec_client_node="ec_client";
    REPL_STATUS _internal_repl_sts=REPL_STATUS::OPERATIONAL;
    SLAVE_CMD _slave_cmd_type;
    SlaveCmdWidget *_slave_cmd_wid;
};

#endif // SLAVE_WIDGET_H

