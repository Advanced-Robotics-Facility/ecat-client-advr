#ifndef PDO_WIDGET_H
#define PDO_WIDGET_H

#include <QtUiTools/QtUiTools>
#include <QWidget>
#include <ros/ros.h>
#include "ec_msgs/SlaveDescription.h"
#include "ec_msgs/MotorPDO.h"
#include "ec_msgs/PowPDO.h"
#include "ec_msgs/ImuPDO.h"
#include "ec_msgs/FtPDO.h"

class PDOWidget : public QWidget
{
    Q_OBJECT

public:

    explicit PDOWidget(QWidget *parent = nullptr);

    void motorPDOCallback(const ros::MessageEvent<ec_msgs::MotorPDO>& event);
    void powPDOCallback(const ros::MessageEvent<ec_msgs::PowPDO>& event);
    void imuPDOCallback(const ros::MessageEvent<ec_msgs::ImuPDO>& event);
    void ftPDOCallback(const ros::MessageEvent<ec_msgs::FtPDO>& event);
    void onStartStopBtnReleased();


private slots:
    void spin_ros();
private:
    QTimer *_rosTimer;

private:
    std::vector<ec_msgs::SlaveDescription> _slaves_descr_vector;
    std::vector<std::string> _jnames;
    QTreeWidget * _tree_wid;
    QList<QString> _motor_pdo_fields,_pow_pdo_fields,_imu_pdo_fields,_ft_pdo_fields;

    std::unique_ptr<ros::NodeHandle>  _node;
    ros::Subscriber  _sub_motor,_sub_pow,_sub_imu,_sub_ft;

    QPushButton *_start_stop_btn;
    double _pdo_init_pub_time;

};

#endif // MASTER_CMD_WIDGET_H
