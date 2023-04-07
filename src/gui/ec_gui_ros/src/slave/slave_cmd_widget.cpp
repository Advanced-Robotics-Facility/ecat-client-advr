#include "slave_cmd_widget.h"
#include "joint_widget.h"

#include <QUiLoader>
#include <QFile>
#include <QVBoxLayout>
#include <iostream>


#include "ec_srvs/SelectSlave.h"
#include "ec_srvs/SetMotorPosVelGains.h"
#include "ec_srvs/SetMotorImpGains.h"
#include "ec_srvs/SetSlaveCurrent.h"
#include "ec_srvs/SetMotorLimits.h"
#include "ec_srvs/SetSlavesFirmware.h"

#include "ec_srvs/SetMotorHomingTrj.h"
#include "ec_srvs/SetMotorPeriodTrj.h"
#include "ec_srvs/SetMotorSmoothTrj.h"

#include "ec_srvs/SetMotorCtrlValue.h"

void slave_cmd_widget_qrc_init()
{
    Q_INIT_RESOURCE(slave_resources);
}

namespace  {


QWidget * LoadUiFile(QWidget * parent)
{
    slave_cmd_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/slave_cmd_widget.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;


}

}

SlaveCmdWidget::SlaveCmdWidget(std::vector<ec_msgs::SlaveDescription> slaves_descr_vector,
                               QWidget *parent) :
    QWidget(parent)
{
    //std::cout << "ClassName: " << this->staticMetaObject.className() << std::endl;
    /* Create GUI layout */
    auto * ui = ::LoadUiFile(this);
    auto * layout = new QVBoxLayout;
    layout->addWidget(ui);
    setLayout(layout);

    /* GET different bar division */
    auto bars_layout = findChild<QVBoxLayout *>("BarsLayout");

    bars_layout->setSpacing(1);


    _qwid= new QTreeWidget();

    _qwid->setHeaderLabel("Slave Selection");

    /* Getting joint/slave names using the slaves descr ROS Message */
    std::vector<std::string> id_low_level;
    for(int i=0;i<slaves_descr_vector.size();i++)
    {
      ec_msgs::SlaveDescription slave_descr=slaves_descr_vector[i];
      _jnames.push_back(slave_descr.slave_name);
      id_low_level.push_back(slave_descr.id_low_level);
    }

    _slave_fields.append("Slave_ID: ");
    _slave_fields.append("Firmware: ");
    _slave_fields.append("Limits: ");
    _slave_fields.append("Gains: ");
    _slave_fields.append("Current: ");
    _slave_fields.append("Motor Status: ");

    /* Create Joint Widget distributing them in different bars */
    for(int i = 0; i < _jnames.size(); i++)
    {
        QTreeWidgetItem * topLevel = new QTreeWidgetItem();
        QTreeWidgetItem *childItem = new QTreeWidgetItem();


        topLevel->setText(0,QString::fromStdString(_jnames[i]));
        topLevel->setCheckState(0,Qt::Unchecked);

        //topLevel->addChild(childItem);

        auto wid = new JointWidget(QString::fromStdString(_jnames[i]),
                                      this);

        for(int j=0; j<_slave_fields.size(); j++)
        {
            QTreeWidgetItem * item = new QTreeWidgetItem();
            item->setText(0,_slave_fields.at(j));
            if(_slave_fields.at(j)=="Slave_ID: ")
            {
                item->setText(0,_slave_fields.at(j)+QString::fromStdString(id_low_level[i]));
            }
            else if(_slave_fields.at(j)=="Motor Status: ")
            {
                item->setText(0,_slave_fields.at(j)+"Unknown");
            }
            else
            {
                item->setText(0,_slave_fields.at(j));
            }


            topLevel->addChild(item);
        }

        _qwid->addTopLevelItem(topLevel);
        //_qwid->setItemWidget(childItem,0,wid);

        bars_layout->addWidget(_qwid);

        _wid_map[_jnames[i]] = wid;

    }

    HideTxt();
    HideRadio();

    /* Getting command manager (Apply NoToAll YesToAll) */
    _cmd_manager = findChild<QDialogButtonBox *>("CmdManager");

    if(!_cmd_manager)
    {
        throw std::runtime_error("_cmd_manager");
    }


   _applybtn = _cmd_manager->button(QDialogButtonBox::Apply);

   auto notallbtn = _cmd_manager->button(QDialogButtonBox::NoToAll);

   connect(notallbtn, &QPushButton::released,
           this, &SlaveCmdWidget::onNotAllCmdReleased);

   auto allbtn = _cmd_manager->button(QDialogButtonBox::YesToAll);

   connect(allbtn, &QPushButton::released,
           this, &SlaveCmdWidget::onAllCmdReleased);

   /* Getting firmware manager (Firm Btn1 and Btn2) */
   _firm1_manager = findChild<QDialogButtonBox *>("FirmBtn1");
   _firm2_manager = findChild<QDialogButtonBox *>("FirmBtn2");

   /* Firmware components */
   auto firmware_btn1 = _firm1_manager->button(QDialogButtonBox::Open);

   if(!firmware_btn1)
   {
       throw std::runtime_error("firmware_btn1");
   }

   connect(firmware_btn1, &QPushButton::released,
           this, &SlaveCmdWidget::onFirmBtn1Released);

   auto firmware_btn2 = _firm2_manager->button(QDialogButtonBox::Open);

   if(!firmware_btn2)
   {
       throw std::runtime_error("firmware_bt2");
   }

   connect(firmware_btn2, &QPushButton::released,
           this, &SlaveCmdWidget::onFirmBtn2Released);

   _firmware_line1 = findChild<QLineEdit *>("FirmLine1");

   if(!_firmware_line1)
   {
       throw std::runtime_error("_firmware_line1");
   }

   _firmware_password_line1 = findChild<QLineEdit *>("PasswordLine1");

   if(!_firmware_password_line1)
   {
       throw std::runtime_error("_firmware_password_line1");
   }

   _firmware_line2 = findChild<QLineEdit *>("FirmLine2");

   if(!_firmware_line2)
   {
       throw std::runtime_error("_firmware_line2");
   }

   _firmware_password_line2 = findChild<QLineEdit *>("PasswordLine2");

   if(!_firmware_password_line2)
   {
       throw std::runtime_error("_firmware_password_line2");
   }


   _c28_label = findChild<QLabel *>("C28File");
   if(!_c28_label)
   {
       throw std::runtime_error("_c28_label");
   }

   _m3_label = findChild<QLabel *>("M3File");
   if(!_m3_label)
   {
       throw std::runtime_error("_m3_label");

   }
   _c28_pass_label  = findChild<QLabel *>("PasswordLabel1");
   if(!_c28_pass_label)
   {
       throw std::runtime_error("_c28_pass_label");
   }

   _m3_pass_label  = findChild<QLabel *>("PasswordLabel2");
   if(!_m3_pass_label)
   {
       throw std::runtime_error("_c28_pass_label");
   }

   _host_ip_label  = findChild<QLabel *>("HostIPLabel");
   if(!_host_ip_label)
   {
       throw std::runtime_error("_c28_pass_label");
   }

   _host_name_label  = findChild<QLabel *>("HostNameLabel");
   if(!_host_name_label)
   {
       throw std::runtime_error("_host_name_label");
   }

   if(!_firmware_password_line1)
   {
       throw std::runtime_error("_firmware_password_line1");
   }

   _host_ip_line = findChild<QLineEdit *>("HostIPLineEdit");

   if(!_host_ip_line)
   {
       throw std::runtime_error("_host_ip_line");
   }

   _host_name_line = findChild<QLineEdit *>("HostNameLineEdit");

   if(!_host_name_line)
   {
       throw std::runtime_error("_host_name_line");
   }

   _transfer_file_btn = findChild<QPushButton *>("TransferFiles");

   if(!_transfer_file_btn)
   {
       throw std::runtime_error("_transfer_file_btn");
   }

   connect(_transfer_file_btn, &QPushButton::released,
           this, &SlaveCmdWidget::onTransferFileReleased);

   auto info_scan_btn = findChild<QPushButton *>("InfoScan");

   if(!info_scan_btn)
   {
       throw std::runtime_error("motor_scan_btn");
   }

   connect(info_scan_btn, &QPushButton::released,
           this, &SlaveCmdWidget::InfoScan);

}

void SlaveCmdWidget::set_ec_client_node(std::string ec_client_node)
{
    _ec_client_node=ec_client_node;
}
void SlaveCmdWidget::set_chain_number(int ch_numb)
{
    _ch_numb=ch_numb;
}
void SlaveCmdWidget::set_msgText(QTextEdit * msgText)
{
   _msgText=msgText;
}

void SlaveCmdWidget::set_data_object_type(std::string data_object_type)
{
    _data_object_type=data_object_type;
}

QPushButton * SlaveCmdWidget::get_apply_btn()
{
    return _applybtn;
}

void SlaveCmdWidget::ClearTxtReleased()
{
    /* Clear Joint WID text box */
    for(int i = 0; i < _jnames.size(); i++)
    {
        JointWidget *wid = _wid_map.at(_jnames[i]);
        auto txtmsg = wid->findChild<QLineEdit *>("TxtInfo");
        txtmsg->clear();
    }

}

void SlaveCmdWidget::HideTxt()
{
    /* Hide Joint WID text box */
    for(int i = 0; i < _jnames.size(); i++)
    {
        JointWidget *wid = _wid_map.at(_jnames[i]);
        auto txtmsg = wid->findChild<QLineEdit *>("TxtInfo");
        txtmsg->hide();
    }
}

void SlaveCmdWidget::HideRadio()
{
    /* Hide Joint WID text box */
    for(int i = 0; i < _jnames.size(); i++)
    {
        JointWidget *wid = _wid_map.at(_jnames[i]);
        auto radiotbtn = wid->findChild<QRadioButton *>("JointSelect");

        if(!radiotbtn)
        {
            throw std::runtime_error("radio button not exists");
        }
        radiotbtn->hide();
    }
}


void SlaveCmdWidget::ApplyCmd(std::map<int ,std::map<int,std::map<std::string, SliderWidget *>>> setup_slave,
                              SLAVE_CMD slave_cmd_type,
                              bool setting,
                              int set_type,
                              int numb_slider)
{
    std::vector<std::string> slaves_name_vector;
    _msgText->clear();

    for(int i = 0; i < _jnames.size(); i++)
    {
        QTreeWidgetItem * topLevel = _qwid->topLevelItem(i);
        /* get Joint checked for the command */
        JointWidget *wid = _wid_map.at(_jnames[i]);

        auto radiotbtn = wid->findChild<QRadioButton *>("JointSelect");

        if(!radiotbtn)
        {
            throw std::runtime_error("radio button not exists");
        }

        if(topLevel->checkState(0)==Qt::Checked)
        {
            radiotbtn->setChecked(true);
        }
        else
        {
            radiotbtn->setChecked(false);
        }

        if(radiotbtn->isChecked())
        {
            /* create a slave name vector */
            slaves_name_vector.push_back(_jnames[i]);
            /* check if a SET CMD*/
            if(setting)
            {
                for(int j=0;j<_ch_numb;j++)
                {
                    /* GET slave setup using CHAIN, set type (POSITION, VELOCITY,IMPEDANCE and CURRENT, Slave Name*/
                    if(setup_slave[j][set_type][_jnames[i]])
                    {
                    auto slider_wid= setup_slave[j][set_type][_jnames[i]];
                    /* GET VALUES from slider */
                    std::vector <double> values;
                    for(int k=0;k<numb_slider;k++)
                        values.push_back(slider_wid->get_slider_value(k));
                    /* PERFORM SET CMD */
                    SetSlaveCmd(slave_cmd_type,_jnames[i],values); // Start motor can ovverride the gains
                    }
                }
            }
        }
    }

    //std::cout << "PRIMAY COMMAD: " << slave_cmd_type << std::endl;

    if(!slaves_name_vector.empty())
    {
        /* PERFORM GET CMD */
        GetSlaveCmd(slave_cmd_type,slaves_name_vector);
    }
}


void SlaveCmdWidget::ApplyMotorCtrl(std::map<int ,std::map<int,std::map<std::string, SliderWidget *>>> setup_motor_ctrl,
                                    SLAVE_CMD slave_cmd_type,
                                    bool setting,
                                    int set_type,
                                    int numb_slider)
{
    std::vector<std::string> slaves_name_vector;

    for(int i = 0; i < _jnames.size(); i++)
    {
        QTreeWidgetItem * topLevel = _qwid->topLevelItem(i);
        /* get Joint checked for the command */
        JointWidget *wid = _wid_map.at(_jnames[i]);

        auto radiotbtn = wid->findChild<QRadioButton *>("JointSelect");

        if(!radiotbtn)
        {
            throw std::runtime_error("radio button not exists");
        }

        if(topLevel->checkState(0)==Qt::Checked)
        {
            radiotbtn->setChecked(true);
        }
        else
        {
            radiotbtn->setChecked(false);
        }

        if(radiotbtn->isChecked())
        {
            /* create a slave name vector */
            slaves_name_vector.push_back(_jnames[i]);
            /* check if a SET CMD*/
            if(setting)
            {
                for(int j=0;j<_ch_numb;j++)
                {
                    if(setup_motor_ctrl[j][set_type][_jnames[i]])
                    {
                    auto slider_wid= setup_motor_ctrl[j][set_type][_jnames[i]];
                    /* GET VALUES from slider */
                    std::vector <double> values;
                    for(int k=0;k<numb_slider;k++)
                    {
                        if((slave_cmd_type==SET_LED)||(slave_cmd_type==SET_FAN))
                        {
                            values.push_back(slider_wid->get_radiobtn_value(k));
                        }
                        else
                        {
                            values.push_back(slider_wid->get_slider_value(k));
                        }
                    }
                    /* PERFORM Motor CMD */
                    SetMotorCtrl(slave_cmd_type,_jnames[i],values); // Start motor can ovverride the gains
                    }
                }
            }
        }
    }
}

void SlaveCmdWidget::ApplyTrj(std::map<int ,std::map<int,std::map<std::string, SliderWidget *>>> setup_trj,
                              SLAVE_CMD slave_cmd_type,
                              bool setting,
                              int set_type,
                              int numb_slider)
{
    std::vector<std::string> slaves_name_vector;

    for(int i = 0; i < _jnames.size(); i++)
    {
        QTreeWidgetItem * topLevel = _qwid->topLevelItem(i);
        /* get Joint checked for the command */
        JointWidget *wid = _wid_map.at(_jnames[i]);

        auto radiotbtn = wid->findChild<QRadioButton *>("JointSelect");

        if(!radiotbtn)
        {
            throw std::runtime_error("radio button not exists");
        }

        if(topLevel->checkState(0)==Qt::Checked)
        {
            radiotbtn->setChecked(true);
        }
        else
        {
            radiotbtn->setChecked(false);
        }

        if(radiotbtn->isChecked())
        {
            /* create a slave name vector */
            slaves_name_vector.push_back(_jnames[i]);
            /* check if a SET CMD*/
            if(setting)
            {
                for(int j=0;j<_ch_numb;j++)
                {
                    if(setup_trj[j][set_type][_jnames[i]])
                    {
                    auto slider_wid= setup_trj[j][set_type][_jnames[i]];
                    /* GET VALUES from slider */
                    std::vector <double> values;
                    for(int k=0;k<numb_slider;k++)
                        values.push_back(slider_wid->get_slider_value(k));
                    /* PERFORM TRJ CMD */
                    SetSlaveTrj(slave_cmd_type,_jnames[i],values);
                    }
                }
            }
        }
    }
}


void SlaveCmdWidget::ApplyFlashCmd(std::string flash_type)
{
    std::vector<std::string> slaves_name_vector;
    _msgText->clear();

    for(int i = 0; i < _jnames.size(); i++)
    {
        QTreeWidgetItem * topLevel = _qwid->topLevelItem(i);
        /* get Joint checked for the command */
        JointWidget *wid = _wid_map.at(_jnames[i]);

        auto radiotbtn = wid->findChild<QRadioButton *>("JointSelect");

        if(!radiotbtn)
        {
            throw std::runtime_error("radio button not exists");
        }

        if(topLevel->checkState(0)==Qt::Checked)
        {
            radiotbtn->setChecked(true);
        }
        else
        {
            radiotbtn->setChecked(false);
        }

        if(radiotbtn->isChecked())
        {
            slaves_name_vector.push_back(_jnames[i]);

        }
    }

    FlashCmd(flash_type,slaves_name_vector);
}

void SlaveCmdWidget::ApplyTrjCmd(std::string trj_cmd)
{
    std::vector<std::string> slaves_name_vector;
    _msgText->clear();

    for(int i = 0; i < _jnames.size(); i++)
    {
        QTreeWidgetItem * topLevel = _qwid->topLevelItem(i);
        /* get Joint checked for the command */
        JointWidget *wid = _wid_map.at(_jnames[i]);

        auto radiotbtn = wid->findChild<QRadioButton *>("JointSelect");

        if(!radiotbtn)
        {
            throw std::runtime_error("radio button not exists");
        }

        if(topLevel->checkState(0)==Qt::Checked)
        {
            radiotbtn->setChecked(true);
        }
        else
        {
            radiotbtn->setChecked(false);
        }

        if(radiotbtn->isChecked())
        {
            slaves_name_vector.push_back(_jnames[i]);

        }
    }

    TrjCmd(trj_cmd,slaves_name_vector);
}

void SlaveCmdWidget::onNotAllCmdReleased()
{
    /* Uncheck all radio buttons of Joint WID */
    for(int i = 0; i < _jnames.size(); i++)
    {
        QTreeWidgetItem * topLevel = _qwid->topLevelItem(i);

        JointWidget *wid = _wid_map.at(_jnames[i]);

        auto radiotbtn = wid->findChild<QRadioButton *>("JointSelect");

        if(!radiotbtn)
        {
            throw std::runtime_error("radio button not exists");
        }

        radiotbtn->setChecked(false);
        topLevel->setCheckState(0,Qt::Unchecked);

    }

}

void SlaveCmdWidget::onAllCmdReleased()
{
    /* Check all radio buttons of Joint WID */
    for(int i = 0; i < _jnames.size(); i++)
    {
        QTreeWidgetItem * topLevel = _qwid->topLevelItem(i);

        JointWidget *wid = _wid_map.at(_jnames[i]);

        auto radiotbtn = wid->findChild<QRadioButton *>("JointSelect");

        if(!radiotbtn)
        {
            throw std::runtime_error("radio button not exists");
        }

        radiotbtn->setChecked(true);
        topLevel->setCheckState(0,Qt::Checked);
    }

}

bool SlaveCmdWidget::slave_selected()
{
    /* Check all radio buttons of Joint WID */
    for(int i = 0; i < _jnames.size(); i++)
    {
        QTreeWidgetItem * topLevel = _qwid->topLevelItem(i);

        JointWidget *wid = _wid_map.at(_jnames[i]);

        auto radiotbtn = wid->findChild<QRadioButton *>("JointSelect");

        if(!radiotbtn)
        {
            throw std::runtime_error("radio button not exists");
        }

        if(topLevel->checkState(0)==Qt::Checked)
        {
            radiotbtn->setChecked(true);
        }
        else
        {
            radiotbtn->setChecked(false);
        }

        if(radiotbtn->isChecked())
            return true;
    }
    return false;
}

void SlaveCmdWidget::onFirmBtn1Released()
{
    _firmware_line1->clear();

    QString firmware_file = QFileDialog::getOpenFileName(this,
        tr("Open BIN"), _start_dir, tr("BIN Files (*.bin)"), Q_NULLPTR, QFileDialog::DontUseNativeDialog);
    if(!firmware_file.isEmpty())
    {
        _firmware_line1->setText(firmware_file);
        QDir firmware_dir = QFileInfo(firmware_file).absoluteDir();
        _start_dir=firmware_dir.absolutePath();
    }
}

void SlaveCmdWidget::onFirmBtn2Released()
{
    _firmware_line2->clear();
    QString firmware_file = QFileDialog::getOpenFileName(this,
        tr("Open BIN"), _start_dir, tr("BIN Files (*.bin)"), Q_NULLPTR, QFileDialog::DontUseNativeDialog);
    if(!firmware_file.isEmpty())
    {
        _firmware_line2->setText(firmware_file);
        QDir firmware_dir = QFileInfo(firmware_file).absoluteDir();
        _start_dir=firmware_dir.absolutePath();
    }

}


void SlaveCmdWidget::DisableFirmwareComponents()
{
    HideFirmwareComponents();

    _firm1_manager->setDisabled(true);
    _firm2_manager->setDisabled(true);

    _firmware_line1->setDisabled(true);
    _firmware_line2->setDisabled(true);

    _c28_label->setDisabled(true);
    _m3_label->setDisabled(true);

    _c28_pass_label->setDisabled(true);
    _m3_pass_label->setDisabled(true);

    _firmware_password_line1->setDisabled(true);
    _firmware_password_line2->setDisabled(true);

    _host_ip_line->setDisabled(true);
    _host_name_line->setDisabled(true);
    _host_name_label->setDisabled(true);
    _host_ip_label->setDisabled(true);
    _transfer_file_btn->setDisabled(true);


}

void SlaveCmdWidget::EnableFirmwareComponents()
{
    ShowFirmwareComponents();

    _firm1_manager->setDisabled(false);
    _firm2_manager->setDisabled(false);

    _firmware_line1->setDisabled(false);
    _firmware_line2->setDisabled(false);

    _c28_label->setDisabled(false);
    _m3_label->setDisabled(false);

    _c28_pass_label->setDisabled(false);
    _m3_pass_label->setDisabled(false);

    _firmware_password_line1->setDisabled(false);
    _firmware_password_line2->setDisabled(false);

    _host_ip_line->setDisabled(false);
    _host_name_line->setDisabled(false);
    _host_name_label->setDisabled(false);
    _host_ip_label->setDisabled(false);
    _transfer_file_btn->setDisabled(false);

}

void SlaveCmdWidget::HideFirmwareComponents()
{
    _firm1_manager->hide();
    _firm2_manager->hide();

    _firmware_line1->hide();
    _firmware_line2->hide();

    _c28_label->hide();
    _m3_label->hide();

    _c28_pass_label->hide();
    _m3_pass_label->hide();

    _firmware_password_line1->hide();
    _firmware_password_line2->hide();

    _host_ip_line->hide();
    _host_name_line->hide();
    _host_name_label->hide();
    _host_ip_label->hide();
    _transfer_file_btn->hide();
}

void SlaveCmdWidget::ShowFirmwareComponents()
{
    _firm1_manager->show();
    _firm2_manager->show();

    _firmware_line1->show();
    _firmware_line2->show();

    _c28_label->show();
    _m3_label->show();

    _c28_pass_label->show();
    _m3_pass_label->show();

    _firmware_password_line1->show();
    _firmware_password_line2->show();

    _host_ip_line->show();
    _host_name_line->show();
    _host_name_label->show();
    _host_ip_label->show();
    _transfer_file_btn->show();
}


void SlaveCmdWidget::GetSlaveCmd(SLAVE_CMD slave_cmd_type,std::vector<std::string> slaves_name_vector)
{
    ros::NodeHandle n;
    ros::ServiceClient client;

    if(slave_cmd_type==STOP_SLAVE)
    {
      client= n.serviceClient<ec_srvs::SelectSlave>(_ec_client_node+"/stop_motors");
    }
    else if(slave_cmd_type==START_SLAVE_POS)
    {
      client= n.serviceClient<ec_srvs::SelectSlave>(_ec_client_node+"/start_motors_posmode");
    }
    else if(slave_cmd_type==START_SLAVE_VEL)
    {
      client= n.serviceClient<ec_srvs::SelectSlave>(_ec_client_node+"/start_motors_velmode");
    }
    else if(slave_cmd_type==START_SLAVE_IMP)
    {
      client= n.serviceClient<ec_srvs::SelectSlave>(_ec_client_node+"/start_motors_impmode");
    }

    else if(slave_cmd_type==GET_SLAVE_FIRM_VER)
    {
      client= n.serviceClient<ec_srvs::SelectSlave>(_ec_client_node+"/get_slaves_firmware");
    }
    else if(slave_cmd_type==GET_SLAVE_AMP_INFO)
    {
      client= n.serviceClient<ec_srvs::SelectSlave>(_ec_client_node+"/get_motors_ampere_info");
    }
    else if(slave_cmd_type==GET_SLAVE_LIM_INFO)
    {
      client= n.serviceClient<ec_srvs::SelectSlave>(_ec_client_node+"/get_motors_limits_info");
    }

    else if(slave_cmd_type==GET_SLAVE_GAIN_INFO)
    {
      client= n.serviceClient<ec_srvs::SelectSlave>(_ec_client_node+"/get_motors_gains_info");
    }

    else if(slave_cmd_type==GET_SDO_INFO)
    {
      client= n.serviceClient<ec_srvs::SelectSlave>(_ec_client_node+"/get_slaves_sdo_info_by_name");
    }

    else
    {
       return;
    }

    ec_srvs::SelectSlave srv;
    for(int i=0;i<slaves_name_vector.size();i++)
        srv.request.slave_name.push_back(slaves_name_vector[i]);

    if(client.call(srv))
    {
        std::vector <ec_msgs::SlaveCmdInfo> slaves_info=srv.response.slaves_info;
        ec_msgs::CmdStatus cmd_status=srv.response.cmd_status;
        printSlaveCmdFdback(slaves_info,cmd_status);
    }
    else
    {
        _msgText->textCursor().insertText("Failed to call the service! \n");
    }
}



void SlaveCmdWidget::SetSlaveCmd(SLAVE_CMD slave_cmd_type,std::string slaves_name,std::vector<double> values)
{
    ros::NodeHandle n;
    ros::ServiceClient client;
    int set_type=0;

    if((slave_cmd_type==START_SLAVE_POS)||(slave_cmd_type==SET_POS_GAINS))
    {
      client= n.serviceClient<ec_srvs::SetMotorPosVelGains>(_ec_client_node+"/set_motors_posgains");
      set_type=1;
    }
    else if((slave_cmd_type==START_SLAVE_VEL)||(slave_cmd_type==SET_VEL_GAINS))
    {
      client= n.serviceClient<ec_srvs::SetMotorPosVelGains>(_ec_client_node+"/set_motors_velgains");
      set_type=1;
    }
    else if((slave_cmd_type==START_SLAVE_IMP)||(slave_cmd_type==SET_IMP_GAINS))
    {
      client= n.serviceClient<ec_srvs::SetMotorImpGains>(_ec_client_node+"/set_motors_impgains");
      set_type=2;
    }
    else if(slave_cmd_type==SET_MAX_AMP)
    {
      client= n.serviceClient<ec_srvs::SetSlaveCurrent>(_ec_client_node+"/set_motors_max_ampere");
      set_type=3;
    }
    else if(slave_cmd_type==SET_LIM)
    {
      client= n.serviceClient<ec_srvs::SetMotorLimits>(_ec_client_node+"/set_motors_limits");
      set_type=4;
    }
    else if(slave_cmd_type==SET_FIRM_VER)
    {
      client= n.serviceClient<ec_srvs::SetSlavesFirmware>(_ec_client_node+"/set_slaves_firmware");
      set_type=5;
    }
    else
    {
       return;
    }

    if(set_type==1)
        SetPosVelGains(slaves_name,client,values);
    else if(set_type==2)
        SetImpGains(slaves_name,client,values);
    else if(set_type==3)
        SetMaxCurrent(slaves_name,client,values);
    else if(set_type==4)
        SetLimits(slaves_name,client,values);
    else
    {
        SetSlaveFirmware(slaves_name,client);
    }

}

void SlaveCmdWidget::SetPosVelGains(std::string slaves_name,ros::ServiceClient client,std::vector<double> values)
{
   ec_srvs::SetMotorPosVelGains srv;
   srv.request.motor_name=slaves_name;
   srv.request.data_object_type=_data_object_type;
   for(int i=0;i<values.size();i++)
   {
       srv.request.gains[i]=values[i];
   }

   if(client.call(srv))
   {
       std::vector <ec_msgs::SlaveCmdInfo> slaves_info=srv.response.slaves_info;
       ec_msgs::CmdStatus cmd_status=srv.response.cmd_status;
       printSlaveCmdFdback(slaves_info,cmd_status);
   }
   else
   {
       _msgText->textCursor().insertText("Failed to call the service! \n");
   }
}

void SlaveCmdWidget::SetImpGains(std::string slaves_name,ros::ServiceClient client,std::vector<double> values)
{
   ec_srvs::SetMotorImpGains srv;
   srv.request.motor_name=slaves_name;
   srv.request.data_object_type=_data_object_type;

   for(int i=0;i<values.size();i++)
   {
       srv.request.gains[i]=values[i];
   }

   if(client.call(srv))
   {
       std::vector <ec_msgs::SlaveCmdInfo> slaves_info=srv.response.slaves_info;
       ec_msgs::CmdStatus cmd_status=srv.response.cmd_status;
       printSlaveCmdFdback(slaves_info,cmd_status);
   }
   else
   {
       _msgText->textCursor().insertText("Failed to call the service! \n");
   }
}

void SlaveCmdWidget::SetMaxCurrent(std::string slaves_name,ros::ServiceClient client,std::vector<double> values)
{
   ec_srvs::SetSlaveCurrent srv;
   srv.request.slave_name.push_back(slaves_name);

   for(int i=0;i<values.size();i++)
   {
       srv.request.current.push_back(values[i]);
   }

   if(client.call(srv))
   {
       std::vector <ec_msgs::SlaveCmdInfo> slaves_info=srv.response.slaves_info;
       ec_msgs::CmdStatus cmd_status=srv.response.cmd_status;
       printSlaveCmdFdback(slaves_info,cmd_status);
   }
   else
   {
       _msgText->textCursor().insertText("Failed to call the service! \n");
   }
}

void SlaveCmdWidget::SetLimits(std::string slaves_name,ros::ServiceClient client,std::vector<double> values)
{
   ec_srvs::SetMotorLimits srv;
   srv.request.motor_name=slaves_name;

   srv.request.min_pos=values[0];
   srv.request.max_pos=values[1];
   srv.request.max_vel=values[2];
   srv.request.max_torq=values[3];

   if(client.call(srv))
   {
       std::vector <ec_msgs::SlaveCmdInfo> slaves_info=srv.response.slaves_info;
       ec_msgs::CmdStatus cmd_status=srv.response.cmd_status;
       printSlaveCmdFdback(slaves_info,cmd_status);
   }
   else
   {
       _msgText->textCursor().insertText("Failed to call the service! \n");
   }
}

void SlaveCmdWidget::onTransferFileReleased()
{
    std::string c28_file=_firmware_line1->text().toStdString();
    std::string c28_password=_firmware_password_line1->text().toStdString();

    std::string m3_file=_firmware_line2->text().toStdString();
    std::string m3_password=_firmware_password_line2->text().toStdString();

    if((c28_file=="")||(m3_file==""))
    {
        QMessageBox msgBox;
        if((c28_file=="")&&(m3_file==""))
            msgBox.setText("C28 and M3 Files not set!");
        else if(c28_file=="")
            msgBox.setText("C28 File not set!");
        else
            msgBox.setText("M3 File not set!");

        msgBox.exec();
        return;
    }

    QProcess OProcess ;
    QString Command = "scp" ; //Contains the command to be executed
    QStringList params ;

    std::stringstream s;
    s << _host_name_line->text().toStdString() << "@"
      << _host_ip_line->text().toStdString() << ":"
      << "~/.ecat_master/firmware";

    std::cout << s.str() << std::endl ;

    params.append("-p");
    params.append("pitagora");
    params.append("scp");

    params.append(c28_file.c_str());
    params.append(m3_file.c_str());
    params.append(QString::fromStdString(s.str()));
    

    OProcess.startDetached("sshpass",params);  //OProcess.startDetached("scp",params); //Starts execution of command
                                               // sudo apt-get install ssh-askpass
                                               // sudo apt-get install -y sshpass
                                               // sshpass -p 'pitagora' scp cent_AC_c28.bin cent_AC_m3.bin embedded@10.24.5.100:~/.ecat_master/firmware/fw_test_GUI
}

void SlaveCmdWidget::SetSlaveFirmware(std::string slaves_name,ros::ServiceClient client)
{
    QFileInfo c28_file_info = QFileInfo(_firmware_line1->text());
    QFileInfo m3_file_info  = QFileInfo(_firmware_line2->text());

    std::string c28_file = c28_file_info.baseName().toStdString();
    std::string c28_password=_firmware_password_line1->text().toStdString();

    std::string m3_file=m3_file_info.baseName().toStdString();
    std::string m3_password=_firmware_password_line2->text().toStdString();


    if((c28_file=="")||(m3_file==""))
    {
        QMessageBox msgBox;
        if((c28_file=="")&&(m3_file==""))
            msgBox.setText("C28 and M3 Files not set!");
        else if(c28_file=="")
            msgBox.setText("C28 File not set!");
        else
            msgBox.setText("M3 File not set!");

        msgBox.exec();
        return;
    }

    if((c28_password=="")||(m3_password==""))
    {
        QMessageBox msgBox;
        if((c28_password=="")&&(m3_password==""))
            msgBox.setText("C28 and M3 Passwords not set!");
        else if(c28_password=="")
            msgBox.setText("C28 Password not set!");
        else
            msgBox.setText("M3 Password not set!");

        msgBox.exec();
        return;
    }

    ec_srvs::SetSlavesFirmware srv;
    srv.request.slave_name.push_back(slaves_name);

    srv.request.c28_file="/home/"+_host_name_line->text().toStdString()+"/.ecat_master/firmware/fw_test_GUI/"+c28_file+".bin";
    srv.request.c28_password=c28_password;
    srv.request.m3_file="/home/"+_host_name_line->text().toStdString()+"/.ecat_master/firmware/fw_test_GUI/"+m3_file+".bin";
    srv.request.m3_password=m3_password;

    if(client.call(srv))
    {
        std::vector <ec_msgs::SlaveCmdInfo> slaves_info=srv.response.slaves_info;
        ec_msgs::CmdStatus cmd_status=srv.response.cmd_status;
        printSlaveCmdFdback(slaves_info,cmd_status);
    }
    else
    {
        _msgText->textCursor().insertText("Failed to call the service! \n");
    }
}

void SlaveCmdWidget::printSlaveCmdFdback(std::vector<ec_msgs::SlaveCmdInfo> slaves_info, ec_msgs::CmdStatus cmd_status)
{
    QString status_cmd=QString::fromStdString("Status: "+cmd_status.status+"\n");
    _msgText->textCursor().insertText(status_cmd);

    _msgText->textCursor().insertText("Succes Slaves: [");
    for(int i=0;i<cmd_status.success_slaves.size();i++)
    {
        QString success_slave=QString::fromStdString(cmd_status.success_slaves[i]+" ");
        _msgText->textCursor().insertText(success_slave);
    }
    _msgText->textCursor().insertText("]\n");

    _msgText->textCursor().insertText("Unsucces Slaves: [");
    for(int i=0;i<cmd_status.unsuccess_slaves.size();i++)
    {
        QString unsuccess_slave=QString::fromStdString(cmd_status.unsuccess_slaves[i]+" ");
        _msgText->textCursor().insertText(unsuccess_slave);
    }
    _msgText->textCursor().insertText("]\n\n\n");

    for(int k=0; k<slaves_info.size();k++)
    {
        QString status=QString::fromStdString("Status: "+slaves_info[k].status+"\n");
        _msgText->textCursor().insertText(status);
        QString fault_cmd=QString::fromStdString("Fault cmd: "+slaves_info[k].fault_cmd+"\n");
        _msgText->textCursor().insertText(fault_cmd);
        QString fault_recovery=QString::fromStdString("Fault recovery: "+slaves_info[k].fault_recovery+"\n");
        _msgText->textCursor().insertText(fault_recovery);
        QString slave_name=QString::fromStdString("Slave name: "+slaves_info[k].slave_name+"\n");
        _msgText->textCursor().insertText(slave_name);
        QString info_cmd=QString::fromStdString("Info command: "+slaves_info[k].cmd+"\n");
        _msgText->textCursor().insertText(info_cmd);

        QString msg="";
        if(slaves_info[k].msg!="")
        {
            msg=QString::fromStdString(slaves_info[k].msg+"\n" );
        }
        //_msgText->textCursor().insertText(msg);

        JointWidget *wid = _wid_map.at(slaves_info[k].slave_name);

        if((slaves_info[k].cmd=="get_slaves_firmware")||
           (slaves_info[k].cmd=="get_motors_limits_info")||
           (slaves_info[k].cmd=="get_motors_gains_info")||
           (slaves_info[k].cmd=="get_motors_ampere_info"))
        {
            for(int index=0;index< _jnames.size();index++)
            {
                QTreeWidgetItem * topLevel =_qwid->topLevelItem(index);

                if(slaves_info[k].slave_name==topLevel->text(0).toStdString())
                {
                    if(slaves_info[k].cmd=="get_slaves_firmware")
                    {
                       QTreeWidgetItem * item = topLevel->child(1);
                       item->setText(0,_slave_fields.at(1)+msg);
                    }
                    else if(slaves_info[k].cmd=="get_motors_limits_info")
                    {
                        QTreeWidgetItem * item = topLevel->child(2);
                        item->setText(0,_slave_fields.at(2)+msg);
                    }
                    else if(slaves_info[k].cmd=="get_motors_gains_info")
                    {
                        QTreeWidgetItem * item = topLevel->child(3);
                        item->setText(0,_slave_fields.at(3)+msg);
                    }
                    else if(slaves_info[k].cmd=="get_motors_ampere_info")
                    {
                        QTreeWidgetItem * item = topLevel->child(4);
                        item->setText(0,_slave_fields.at(4)+msg);
                    }
                }
            }
        }

        if(slaves_info[k].cmd=="get_slaves_sdo_info_by_name")
            _msgText->textCursor().insertText("Message: "+msg);


        auto txtmsg = wid->findChild<QLineEdit *>("TxtInfo");
        txtmsg->clear();
        txtmsg->setText(msg);

        _msgText->textCursor().insertText("\n\n\n");
    }
}


void SlaveCmdWidget::SetMotorCtrl(SLAVE_CMD slave_cmd_type,std::string slaves_name,std::vector<double> values)
{
    ros::NodeHandle n;
    ros::ServiceClient client;
    ec_srvs::SetMotorCtrlValue srv;

    srv.request.motor_name.push_back(slaves_name);

    if(slave_cmd_type==SET_POSITION)
    {
      client= n.serviceClient<ec_srvs::SetMotorCtrlValue>(_ec_client_node+"/set_motors_position");
      srv.request.position.push_back(values[0]);
    }
    else if(slave_cmd_type==SET_VELOCITY)
    {
      client= n.serviceClient<ec_srvs::SetMotorCtrlValue>(_ec_client_node+"/set_motors_velocity");
      srv.request.velocity.push_back(values[0]);
    }
    else if(slave_cmd_type==SET_TORQUE)
    {
      client= n.serviceClient<ec_srvs::SetMotorCtrlValue>(_ec_client_node+"/set_motors_torque");
      srv.request.torque.push_back(values[0]);
    }
    else if(slave_cmd_type==SET_AMPERAGE)
    {
      client= n.serviceClient<ec_srvs::SetMotorCtrlValue>(_ec_client_node+"/set_motors_amperage");
      srv.request.amperage.push_back(values[0]);
    }
    else if(slave_cmd_type==SET_HOME_POS)
    {
      client= n.serviceClient<ec_srvs::SetMotorCtrlValue>(_ec_client_node+"/set_motors_homing_position");
      srv.request.homing_position.push_back(values[0]);
    }
    else if(slave_cmd_type==SET_LED)
    {
      client= n.serviceClient<ec_srvs::SetMotorCtrlValue>(_ec_client_node+"/set_motors_led");
      if(values[0]>0)
      {
        srv.request.led.push_back(true);
      }
      else
      {
        srv.request.led.push_back(false);
      }
    }
    else if(slave_cmd_type==SET_FAN)
    {
      client= n.serviceClient<ec_srvs::SetMotorCtrlValue>(_ec_client_node+"/set_motors_fan");
      if(values[0]>0)
      {
        srv.request.fan.push_back(true);
      }
      else
      {
        srv.request.fan.push_back(false);
      }
    }

    else
    {
       return;
    }

    if(client.call(srv))
    {
        std::vector <ec_msgs::SlaveCmdInfo> slaves_info=srv.response.slaves_info;
        ec_msgs::CmdStatus cmd_status=srv.response.cmd_status;
        printSlaveCmdFdback(slaves_info,cmd_status);
    }
    else
    {
        _msgText->textCursor().insertText("Failed to call the service! \n");
    }
}



void SlaveCmdWidget::SetSlaveTrj(SLAVE_CMD slave_cmd_type,std::string slaves_name,std::vector<double> values)
{
    ros::NodeHandle n;
    ros::ServiceClient client;
    int set_trj=0;

    if(slave_cmd_type==SET_HOME_TRJ)
    {
      client= n.serviceClient<ec_srvs::SetMotorHomingTrj>(_ec_client_node+"/set_motor_homing_trajectory");
      set_trj=1;
    }
    else if(slave_cmd_type==SET_SINE_TRJ)
    {
      client= n.serviceClient<ec_srvs::SetMotorPeriodTrj>(_ec_client_node+"/set_motor_period_trajectory");
      set_trj=2;
    }
    else if(slave_cmd_type==SET_SMOOTH_TRJ)
    {
      client= n.serviceClient<ec_srvs::SetMotorSmoothTrj>(_ec_client_node+"/set_motor_smooth_trajectory");
      set_trj=3;
    }

    else
    {
       return;
    }

    if(set_trj==1)
        SetHomeTrj(slaves_name,client,values);
    else if(set_trj==2)
        SetSineTrj(slaves_name,client,values);
    else if(set_trj==3)
        SetSmoothTrj(slaves_name,client,values);

}


void SlaveCmdWidget::SetHomeTrj(std::string slaves_name,ros::ServiceClient client,std::vector<double> values)
{
    ec_srvs::SetMotorHomingTrj srv;
    srv.request.motor_name=slaves_name;
    srv.request.trajectory_name="home_trj_"+slaves_name;
    for(int i=0;i<values.size();i++)
    {
        srv.request.x_home.push_back(values[i]);
    }

    if(client.call(srv))
    {
        std::vector <ec_msgs::SlaveCmdInfo> slaves_info=srv.response.slaves_info;
        ec_msgs::CmdStatus cmd_status=srv.response.cmd_status;
        printSlaveCmdFdback(slaves_info,cmd_status);
    }
    else
    {
        _msgText->textCursor().insertText("Failed to call the service! \n");
    }
}
void SlaveCmdWidget::SetSineTrj(std::string slaves_name,ros::ServiceClient client,std::vector<double> values)
{
    ec_srvs::SetMotorPeriodTrj srv;
    srv.request.motor_name=slaves_name;
    srv.request.trajectory_name="sine_trj_"+slaves_name;

    srv.request.frequency=values[0];
    srv.request.amplitude=values[1];
    srv.request.theta=values[2];
    srv.request.secs=values[3];

    if(client.call(srv))
    {
        std::vector <ec_msgs::SlaveCmdInfo> slaves_info=srv.response.slaves_info;
        ec_msgs::CmdStatus cmd_status=srv.response.cmd_status;
        printSlaveCmdFdback(slaves_info,cmd_status);
    }
    else
    {
        _msgText->textCursor().insertText("Failed to call the service! \n");
    }
}
void SlaveCmdWidget::SetSmoothTrj(std::string slaves_name,ros::ServiceClient client,std::vector<double> values)
{
    ec_srvs::SetMotorSmoothTrj srv;
    srv.request.motor_name=slaves_name;
    srv.request.trajectory_name="smooth_trj_"+slaves_name;

    srv.request.x_smooth.push_back(values[0]);
    srv.request.x_smooth.push_back(values[2]);
    srv.request.x_smooth.push_back(values[4]);
    srv.request.x_smooth.push_back(values[6]);

    srv.request.y_smooth.push_back(values[1]);
    srv.request.y_smooth.push_back(values[3]);
    srv.request.y_smooth.push_back(values[5]);
    srv.request.y_smooth.push_back(values[7]);


    if(client.call(srv))
    {
        std::vector <ec_msgs::SlaveCmdInfo> slaves_info=srv.response.slaves_info;
        ec_msgs::CmdStatus cmd_status=srv.response.cmd_status;
        printSlaveCmdFdback(slaves_info,cmd_status);
    }
    else
    {
        _msgText->textCursor().insertText("Failed to call the service! \n");
    }
}

void SlaveCmdWidget::FlashCmd(std::string flash_type,std::vector<std::string> slaves_name_vector)
{
    ros::NodeHandle n;
    ros::ServiceClient client;

    if(flash_type=="Save")
    {
      client= n.serviceClient<ec_srvs::SelectSlave>(_ec_client_node+"/save_slaves_params_to_flash");
    }
    else if(flash_type=="Load")
    {
      client= n.serviceClient<ec_srvs::SelectSlave>(_ec_client_node+"/load_slaves_params_from_flash");
    }
    else if(flash_type=="Default")
    {
      client= n.serviceClient<ec_srvs::SelectSlave>(_ec_client_node+"/load_slaves_default_params");
    }
    else
    {
       return;
    }

    ec_srvs::SelectSlave srv;
    for(int i=0;i<slaves_name_vector.size();i++)
        srv.request.slave_name.push_back(slaves_name_vector[i]);

    if(client.call(srv))
    {
        std::vector <ec_msgs::SlaveCmdInfo> slaves_info=srv.response.slaves_info;
        ec_msgs::CmdStatus cmd_status=srv.response.cmd_status;
        printSlaveCmdFdback(slaves_info,cmd_status);
    }
    else
    {
        _msgText->textCursor().insertText("Failed to call the service! \n");
    }
}

void SlaveCmdWidget::TrjCmd(std::string trj_cmd,std::vector<std::string> slaves_name_vector)
{
    ros::NodeHandle n;
    ros::ServiceClient client;

    if(trj_cmd=="Start")
    {
      client= n.serviceClient<ec_srvs::SelectSlave>(_ec_client_node+"/start_motors_trajectory");
    }
    else if(trj_cmd=="Clear")
    {
      client= n.serviceClient<ec_srvs::SelectSlave>(_ec_client_node+"/clear_motors_trajectory");
    }
    else
    {
       return;
    }

    ec_srvs::SelectSlave srv;
    for(int i=0;i<slaves_name_vector.size();i++)
        srv.request.slave_name.push_back(slaves_name_vector[i]);

    if(client.call(srv))
    {
        std::vector <ec_msgs::SlaveCmdInfo> slaves_info=srv.response.slaves_info;
        ec_msgs::CmdStatus cmd_status=srv.response.cmd_status;
        printSlaveCmdFdback(slaves_info,cmd_status);
    }
    else
    {
        _msgText->textCursor().insertText("Failed to call the service! \n");
    }
}

void SlaveCmdWidget::InfoScan()
{
    GetSlaveCmd(GET_SLAVE_FIRM_VER,_jnames);
    GetSlaveCmd(GET_SLAVE_LIM_INFO,_jnames);
    GetSlaveCmd(GET_SLAVE_GAIN_INFO,_jnames);
    GetSlaveCmd(GET_SLAVE_AMP_INFO,_jnames);
}
