#ifndef EC_GUI_H
#define EC_GUI_H

#include <QWidget>
#include <QPushButton>
#include <QTextEdit>
#include  <QComboBox>
#include <ros/ros.h>
#include "master/master_cmd_widget.h"
#include "slave/slave_cmd_widget.h"
#include "rviz/rviz_widget.h"
#include "slave/slave_widget.h"

#include "ec_msgs/SlaveCmdInfo.h"
#include "ec_msgs/SlaveDescription.h"
#include "ec_srvs/GetSlaveInfo.h"
#include "ec_srvs/PrintMotorInfo.h"

#include <robot_monitoring/custom_qt_widget.h>

class EcGui : public QWidget
{
    Q_OBJECT

public:

    explicit EcGui(QWidget *parent = nullptr);

private:

    bool _slave_found;
    MasterCmdWidget * _master_cmd_wid;
    QTextEdit * _msg_Txt;
    std::vector<ec_msgs::SlaveDescription> _slaveinfo;
    //std::unique_ptr<ros::NodeHandle>  _node;
};

#endif // EC_GUI_H
