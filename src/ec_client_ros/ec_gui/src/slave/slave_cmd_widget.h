#ifndef SLAVE_CMD_WIDGET_H
#define SLAVE_CMD_WIDGET_H

#include <QWidget>
#include <QComboBox>
#include <QDialogButtonBox>
#include <unordered_map>
#include <QRadioButton>
#include <QLineEdit>
#include <QTextEdit>
#include <QPushButton>

#include "joint_widget.h"
#include "sliders_widget.h"

#include <ros/ros.h>
#include "ec_msgs/SlaveDescription.h"
#include "ec_msgs/SlaveCmdInfo.h"
#include "ec_msgs/CmdStatus.h"

enum SLAVE_CMD : int {
    STOP_SLAVE=                0,
    START_SLAVE_POS=           1,
    START_SLAVE_VEL=           2,
    START_SLAVE_IMP=           3,
    GET_SDO_INFO=              4,
    GET_SLAVE_FIRM_VER=        5,
    GET_SLAVE_AMP_INFO=        6,
    GET_SLAVE_LIM_INFO=        7,
    GET_SLAVE_GAIN_INFO=       8,
    SET_POS_GAINS=             8,
    SET_VEL_GAINS=            10,
    SET_IMP_GAINS=            11,
    SET_MAX_AMP=              12,
    SET_FIRM_VER=             13,
    SET_LIM=                  14,
    SET_HOME_TRJ=             15,
    SET_SINE_TRJ=             16,
    SET_SMOOTH_TRJ=           17,
    SET_POSITION=             18,
    SET_VELOCITY=             19,
    SET_TORQUE=               20,
    SET_AMPERAGE=             21,
    SET_HOME_POS=             22,
    SET_LED=                  23,
    SET_FAN=                  24

};


class SlaveCmdWidget : public QWidget
{
Q_OBJECT
public:

    explicit SlaveCmdWidget(std::vector<ec_msgs::SlaveDescription> slaves_descr_vector,
                           QWidget * parent = nullptr);

    void ApplyCmd(std::map<int ,std::map<int,std::map<std::string, SliderWidget *>>> setup_slave,
                  SLAVE_CMD slave_cmd_type,
                  bool setting,
                  int  set_type,
                  int  numb_slider);

    void ApplyTrj(std::map<int ,std::map<int,std::map<std::string, SliderWidget *>>> setup_trj,
                  SLAVE_CMD slave_cmd_type,
                  bool setting,
                  int  set_type,
                  int  numb_slider);

    void ApplyMotorCtrl(std::map<int ,std::map<int,std::map<std::string, SliderWidget *>>> setup_motor_ctrl,
                  SLAVE_CMD slave_cmd_type,
                  bool setting,
                  int  set_type,
                  int  numb_slider);

    void ApplyFlashCmd(std::string flash_type);
    void ApplyTrjCmd(std::string trj_cmd);

    void onNotAllCmdReleased();
    void onAllCmdReleased();
    bool slave_selected();

    void onFirmBtn1Released();
    void onFirmBtn2Released();
    void onTransferFileReleased();

    void ClearTxtReleased();
    void HideTxt();
    void HideRadio();

    void InfoScan();

    void EnableFirmwareComponents();
    void DisableFirmwareComponents();
    void HideFirmwareComponents();
    void ShowFirmwareComponents();

    void GetSlaveCmd(SLAVE_CMD slave_cmd_type,std::vector<std::string> slaves_name_vector);
    void FlashCmd(std::string flash_type,std::vector<std::string> slaves_name_vector);
    void TrjCmd(std::string trj_cmd,std::vector<std::string> slaves_name_vector);

    void SetSlaveCmd(SLAVE_CMD slave_cmd_type,std::string slaves_name,std::vector<double> values);
    void SetMotorCtrl(SLAVE_CMD slave_cmd_type,std::string slaves_name,std::vector<double> values);
    void SetSlaveTrj(SLAVE_CMD slave_cmd_type,std::string slaves_name,std::vector<double> values);

    void SetPosVelGains(std::string slaves_name,ros::ServiceClient client,std::vector<double> values);
    void SetImpGains(std::string slaves_name,ros::ServiceClient client,std::vector<double> values);
    void SetMaxCurrent(std::string slaves_name,ros::ServiceClient client,std::vector<double> values);
    void SetLimits(std::string slaves_name,ros::ServiceClient client,std::vector<double> values);
    void SetSlaveFirmware(std::string slaves_name,ros::ServiceClient client);
    void printSlaveCmdFdback(std::vector <ec_msgs::SlaveCmdInfo> slaves_info,ec_msgs::CmdStatus cmd_status);

    void SetHomeTrj(std::string slaves_name,ros::ServiceClient client,std::vector<double> values);
    void SetSineTrj(std::string slaves_name,ros::ServiceClient client,std::vector<double> values);
    void SetSmoothTrj(std::string slaves_name,ros::ServiceClient client,std::vector<double> values);

    void set_ec_client_node(std::string ec_client_node);
    void set_chain_number(int ch_numb);
    void set_msgText(QTextEdit * msgText);
    void set_data_object_type(std::string data_object_type);
    QPushButton * get_apply_btn();

    std::unordered_map<std::string, JointWidget *> _wid_map;


private:

    QDialogButtonBox  * _cmd_manager;
    QDialogButtonBox  * _firm1_manager;
    QDialogButtonBox  * _firm2_manager;
    QPushButton * _applybtn;

    QLineEdit * _firmware_line1,*_firmware_password_line1;
    QLineEdit * _firmware_line2,*_firmware_password_line2;
    QLabel *_c28_label,*_c28_pass_label;
    QLabel *_m3_label,*_m3_pass_label;

    QLineEdit * _host_ip_line,*_host_name_line;
    QLabel *_host_name_label,*_host_ip_label;
    QPushButton * _transfer_file_btn;

    QString _start_dir="/home";

    std::vector<std::string> _jnames;
    QRadioButton * _radiotbtn;
    QTextEdit * _msgText;

    int _ch_numb;
    std::string _ec_client_node="";
    std::string _data_object_type;

    QList<QString> _slave_fields;
    QTreeWidget *_qwid;
};

#endif // SLAVE_CMD_WIDGET_H
