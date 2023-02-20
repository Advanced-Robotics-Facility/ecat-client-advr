#include "master_cmd_widget.h"
#include <fmt/format.h>
#include <QtUiTools>

#include "ec_srvs/GetSlaveInfo.h"
#include "ec_srvs/GetSlaveInfoStartMaster.h"

void ec_main_cmd_widget_qrc_init()
{
    Q_INIT_RESOURCE(master_resources);
}

namespace
{

QWidget * LoadUiFile(QWidget * parent)
{
    ec_main_cmd_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/master_cmd_widget.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;
}

}


MasterCmdWidget::MasterCmdWidget(QWidget *parent,QTextEdit * msgText,SlaveWidget *slave_wid) :
    QWidget(parent),
    _msgText(msgText),
    _slave_wid(slave_wid)
{
    /* Load ui */
    auto wid = LoadUiFile(this);

    auto l = new QVBoxLayout;
    l->addWidget(wid);

    setLayout(l);

    // status label
    _status_label = findChild<QLabel*>("statusLabel");
    /* Setup behavior */
    _stopMasterBtn = findChild<QPushButton*>("stopMasterBtn");
    if(!_stopMasterBtn)
    {
        throw std::runtime_error("_stopMasterBtn");
    }

    connect(_stopMasterBtn, &QPushButton::released,
            this, &MasterCmdWidget::onStopMasterCmdReleased);

    /* Setup behavior */
    _startMasterBtn = findChild<QPushButton*>("startMasterBtn");
    if(!_startMasterBtn)
    {
        throw std::runtime_error("_startMasterBtn");
    }

    connect(_startMasterBtn, &QPushButton::released,
            this, &MasterCmdWidget::onStartMasterCmdReleased);

    /* Setup behavior */
    _slavesDescrBtn = findChild<QPushButton*>("slavesDescrBtn");
    if(!_slavesDescrBtn)
    {
        throw std::runtime_error("_slavesDescrBtn");
    }

    connect(_slavesDescrBtn, &QPushButton::released,
            this, &MasterCmdWidget::onGetSlaveDescrCmdReleased);


    _preopbtn = findChild<QRadioButton *>("PreOpBtn");

    if(!_preopbtn)
    {
        throw std::runtime_error("pre operational radio button not exists");
    }

    _opbtn = findChild<QRadioButton *>("OpBtn");

    if(!_opbtn)
    {
        throw std::runtime_error("pre operational radio button not exists");
    }

    _preopbtn->setChecked(true);
    _opbtn->setChecked(false);
}

void MasterCmdWidget::set_repl_status(REPL_STATUS repl_status)
{
    if(repl_status==REPL_STATUS::NOT_STARTED)
    {
        _status_label->setText("Not Started ");

        _status_label->setStyleSheet(
            "font-size: 18px; "
            "background-color: #D3D3D3;"
            "border-radius: 4px;"
            "color: gray");
    }
    else if(repl_status==REPL_STATUS::PRE_OPERATIONAL)
    {
        _status_label->setText("Pre-Operational ");
        _slave_wid->set_internal_repl_sts(REPL_STATUS::PRE_OPERATIONAL);

        _status_label->setStyleSheet(
            "font-size: 18px; "
            "background-color: #ff9248;"
            "border-radius: 4px;"
            "color: black");
    }
    else if(repl_status==REPL_STATUS::OPERATIONAL)
    {
        _status_label->setText("Operational ");
        _slave_wid->set_internal_repl_sts(REPL_STATUS::OPERATIONAL);

        _status_label->setStyleSheet(
            "font-size: 18px; "
            "background-color: #99ff99;"
            "border-radius: 4px;"
            "color: green");
    }

}

void MasterCmdWidget::onStopMasterCmdReleased()
{
    _msgText->clear();
    ros::NodeHandle n;
    ros::ServiceClient client;
    client= n.serviceClient<ec_srvs::GetSlaveInfo>("ec_client/stop_master");
    ec_srvs::GetSlaveInfo srv;
    if(client.call(srv))
    {
        ec_msgs::SlaveCmdInfo cmd_info=srv.response.cmd_info;
        PrintSlaveCmdInfo(cmd_info);
    }
    else
    {
        _msgText->textCursor().insertText("Failed to call the service! \n");
    }
}

void MasterCmdWidget::onStartMasterCmdReleased()
{
    _msgText->clear();
    ros::NodeHandle n;
    ros::ServiceClient client;
    client= n.serviceClient<ec_srvs::GetSlaveInfoStartMaster>("ec_client/start_master");
    ec_srvs::GetSlaveInfoStartMaster srv;

    if(_opbtn->isChecked())
    {
        srv.request.mode="run_mode";
        set_repl_status(REPL_STATUS::OPERATIONAL);
    }
    else if(_preopbtn->isChecked())
    {
        srv.request.mode="config_mode";
        //set_repl_status(REPL_STATUS::PRE_OPERATIONAL);
    }
    else
    {
        srv.request.mode="";
        //set_repl_status(REPL_STATUS::NOT_STARTED);
    }

   if(client.call(srv))
   {
       ec_msgs::SlaveCmdInfo cmd_info=srv.response.cmd_info;
       PrintSlaveCmdInfo(cmd_info);
       if(cmd_info.status=="Master started successfully")
       {
            if(srv.request.mode=="run_mode")
                set_repl_status(REPL_STATUS::OPERATIONAL);
            else if(srv.request.mode=="config_mode")
                set_repl_status(REPL_STATUS::PRE_OPERATIONAL);
            else
                set_repl_status(REPL_STATUS::OPERATIONAL);
       }
   }
   else
   {
       _msgText->textCursor().insertText("Failed to call the service! \n");
       set_repl_status(REPL_STATUS::NOT_STARTED);
   }
}

void MasterCmdWidget::onGetSlaveDescrCmdReleased()
{
   _msgText->clear();
   ros::NodeHandle n;
   ros::ServiceClient client;
   client= n.serviceClient<ec_srvs::GetSlaveInfo>("ec_client/get_slaves_description");
   ec_srvs::GetSlaveInfo srv;
   if(client.call(srv))
   {
       ec_msgs::SlaveCmdInfo cmd_info=srv.response.cmd_info;
       PrintSlaveCmdInfo(cmd_info);
   }
   else
   {
       _msgText->textCursor().insertText("Failed to call the service! \n");
   }
}

void MasterCmdWidget::PrintSlaveCmdInfo(ec_msgs::SlaveCmdInfo cmd_info)
{
    QString status=QString::fromStdString("Status: "+cmd_info.status+"\n");
    _msgText->textCursor().insertText(status);
    QString fault_cmd=QString::fromStdString("Fault cmd: "+cmd_info.fault_cmd+"\n");
    _msgText->textCursor().insertText(fault_cmd);
    QString fault_recovery=QString::fromStdString("Fault recovery: "+cmd_info.fault_recovery+"\n");
    _msgText->textCursor().insertText(fault_recovery);
    QString slave_name=QString::fromStdString("Slave name: "+cmd_info.slave_name+"\n");
    _msgText->textCursor().insertText(slave_name);
    QString info_cmd=QString::fromStdString("Info command: "+cmd_info.cmd+"\n");
    _msgText->textCursor().insertText(info_cmd);
    QString msg=QString::fromStdString("Message: "+cmd_info.msg+"\n" );
    _msgText->textCursor().insertText(msg);
}
