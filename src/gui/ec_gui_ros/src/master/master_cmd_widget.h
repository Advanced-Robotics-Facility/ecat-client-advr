#ifndef MASTER_CMD_WIDGET_H
#define MASTER_CMD_WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QTextEdit>
#include <QLabel>
#include <ros/ros.h>
#include <QRadioButton>

#include "ec_msgs/SlaveCmdInfo.h"
#include "../slave/slave_widget.h"

enum MASTER_CMD : int {
    STOP_MASTER=            0,
    START_MASTER=           1,
    GET_SLAVE_DESCR=        2,
};


class MasterCmdWidget : public QWidget
{
    Q_OBJECT

public:

    explicit MasterCmdWidget(QWidget *parent = nullptr,QTextEdit * msgText= nullptr,SlaveWidget *slave_wid= nullptr);
    void set_repl_status(REPL_STATUS repl_status);

private:

    void onStopMasterCmdReleased();
    void onStartMasterCmdReleased();
    void onGetSlaveDescrCmdReleased();
    void getSlavesInfo();
    void PrintSlaveCmdInfo(ec_msgs::SlaveCmdInfo cmd_info);

    QPushButton * _stopMasterBtn;
    QPushButton * _startMasterBtn;
    QPushButton * _slavesDescrBtn;
    QLabel * _status_label;

    QTextEdit * _msgText;
    QRadioButton * _preopbtn;
    QRadioButton * _opbtn;
    SlaveWidget *_slave_wid;

};

#endif // MASTER_CMD_WIDGET_H
