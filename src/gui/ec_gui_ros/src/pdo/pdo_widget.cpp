#include "pdo_widget.h"

#include "ec_srvs/PrintMotorInfo.h"
#include <std_srvs/Empty.h>


void pdo_widget_qrc_init()
{
    Q_INIT_RESOURCE(pdo_resources);
}

namespace
{

QWidget * LoadUiFile(QWidget * parent)
{
    pdo_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/pdo_widget.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);

    file.close();

    return formWidget;
}

}

void PDOWidget::motorPDOCallback(const ros::MessageEvent<ec_msgs::MotorPDO>& event)
{
    const std::string& publisher_name = event.getPublisherName();
    ros::Time receipt_time = event.getReceiptTime();

    if(receipt_time.toSec() + 1.00 < ros::Time::now().toSec())
    {
        ROS_INFO("Message too old!");
        return;
    }

    const ec_msgs::MotorPDO::ConstPtr msg = event.getConstMessage();
    if(_start_stop_btn->text()=="Stop")
    {
        double actual_pub_time=ros::Time::now().toSec()-_pdo_init_pub_time;

        for(int j=0; j < msg->slave_name.size();j++)
        {
            QTreeWidgetItem *topLevel=nullptr;

            bool found_slave=false;
            for(int i=0;i<_tree_wid->topLevelItemCount() && !found_slave;i++)
            {
                QTreeWidgetItem * topLevel_read =_tree_wid->topLevelItem(i);
                if(msg->slave_name[j]==topLevel_read->text(1).toStdString())
                {
                    topLevel=topLevel_read;
                    found_slave=true;
                }
            }

            if(!topLevel)
            {
                topLevel = new QTreeWidgetItem();
                topLevel->setText(1,QString::fromStdString(msg->slave_name[j]));
                topLevel->setText(2,"");
                topLevel->setText(3,"Rx");

                for(int index=0; index<_motor_pdo_fields.size(); index++)
                {
                    QTreeWidgetItem * item = new QTreeWidgetItem();
                    item->setText(1,_motor_pdo_fields.at(index));
                    topLevel->addChild(item);
                }

                _tree_wid->addTopLevelItem(topLevel);
            }

            topLevel->setText(0,QString::number(actual_pub_time, 'f', 3));

            for(int k=0; k<_motor_pdo_fields.size(); k++)
            {
                QTreeWidgetItem * item = topLevel->child(k);

                if(item->text(1)=="Link Position")
                    item->setText(2,QString::number(msg->link_pos[j], 'f', 3));
                else if(item->text(1)=="Link Velocity")
                    item->setText(2,QString::number(msg->link_vel[j], 'f', 3));
                else if(item->text(1)=="Motor Position")
                    item->setText(2,QString::number(msg->motor_pos[j], 'f', 3));
                else if(item->text(1)=="Motor Velocity")
                    item->setText(2,QString::number(msg->motor_vel[j], 'f', 3));
                else if(item->text(1)=="Torque")
                    item->setText(2,QString::number(msg->torque[j], 'f', 5));
                else if(item->text(1)=="Motor Temperature")
                    item->setText(2,QString::number(msg->motor_temp[j], 'f', 3));
                else if(item->text(1)=="Board Temperature")
                    item->setText(2,QString::number(msg->board_temp[j], 'f', 3));
                else if(item->text(1)=="Fault")
                    item->setText(2,QString::fromStdString(msg->fault[j]));
                else if(item->text(1)=="Position Reference")
                    item->setText(2,QString::number(msg->pos_ref[j], 'f', 3));
                else if(item->text(1)=="Velocity Reference")
                    item->setText(2,QString::number(msg->vel_ref[j], 'f', 3));
                else if(item->text(1)=="Torque Reference")
                    item->setText(2,QString::number(msg->tor_ref[j], 'f', 5));
            }

            topLevel->setText(4,QString::number(msg->link_pos[j], 'f', 1) + " " +
                                QString::number(msg->link_vel[j], 'f', 1) + " " +
                                QString::number(msg->motor_pos[j], 'f',1) + " " +
                                QString::number(msg->motor_vel[j], 'f',1) + " " +
                                QString::number(msg->torque[j], 'f', 3)   + " " +
                                QString::number(msg->motor_temp[j], 'f',1)   + " " +
                                QString::number(msg->board_temp[j], 'f',1)   + " " +
                                QString::number(msg->fault_numb[j])   + " " +
                                QString::number(msg->pos_ref[j], 'f',1)   + " " +
                                QString::number(msg->vel_ref[j], 'f',1)   + " " +
                                QString::number(msg->tor_ref[j], 'f',3));
      }

    }
}

void PDOWidget::powPDOCallback(const ros::MessageEvent<ec_msgs::PowPDO>& event)
{
    const std::string& publisher_name = event.getPublisherName();
    ros::Time receipt_time = event.getReceiptTime();

    if(receipt_time.toSec() + 1.00 < ros::Time::now().toSec())
    {
        ROS_INFO("Message too old!");
        return;
    }

    const ec_msgs::PowPDO::ConstPtr msg = event.getConstMessage();
    if(_start_stop_btn->text()=="Stop")
    {
        double actual_pub_time=ros::Time::now().toSec()-_pdo_init_pub_time;

        for(int j=0; j < msg->slave_name.size();j++)
        {
            QTreeWidgetItem *topLevel=nullptr;

            bool found_slave=false;
            for(int i=0;i<_tree_wid->topLevelItemCount() && !found_slave;i++)
            {
                QTreeWidgetItem * topLevel_read =_tree_wid->topLevelItem(i);
                if(msg->slave_name[j]==topLevel_read->text(1).toStdString())
                {
                    topLevel=topLevel_read;
                    found_slave=true;
                }
            }

            if(!topLevel)
            {
                topLevel = new QTreeWidgetItem();
                topLevel->setText(1,QString::fromStdString(msg->slave_name[j]));
                topLevel->setText(2,"");
                topLevel->setText(3,"Rx");

                for(int index=0; index<_pow_pdo_fields.size(); index++)
                {
                    QTreeWidgetItem * item = new QTreeWidgetItem();
                    item->setText(1,_pow_pdo_fields.at(index));
                    topLevel->addChild(item);
                }

                _tree_wid->addTopLevelItem(topLevel);
            }


            topLevel->setText(0,QString::number(actual_pub_time, 'f', 3));

            for(int k=0; k<_pow_pdo_fields.size(); k++)
            {
            QTreeWidgetItem * item = topLevel->child(k);

            if(item->text(1)=="Voltage Battery")
              item->setText(2,QString::number(msg->v_batt[j], 'f', 3));
            else if(item->text(1)=="Voltage Load")
              item->setText(2,QString::number(msg->v_load[j], 'f', 3));
            else if(item->text(1)=="Current Load")
              item->setText(2,QString::number(msg->i_load[j], 'f', 3));
            else if(item->text(1)=="Battery Temperature")
              item->setText(2,QString::number(msg->temp_batt[j], 'f', 3));
            else if(item->text(1)=="HeatSink Temperature")
              item->setText(2,QString::number(msg->temp_heatsink[j], 'f', 3));
            else if(item->text(1)=="PCB Temperature")
              item->setText(2,QString::number(msg->temp_pcb[j], 'f', 3));
            else if(item->text(1)=="Status")
              item->setText(2,QString::number(msg->status[j], 'f', 3));
            else if(item->text(1)=="Fault")
              item->setText(2,QString::fromStdString(msg->fault[j]));
            }

            topLevel->setText(4,QString::number(msg->v_batt[j], 'f', 1) + " " +
                                QString::number(msg->v_load[j], 'f', 1) + " " +
                                QString::number(msg->i_load[j], 'f',1) + " " +
                                QString::number(msg->temp_batt[j], 'f',1) + " " +
                                QString::number(msg->temp_heatsink[j], 'f', 3)   + " " +
                                QString::number(msg->temp_pcb[j], 'f',1)   + " " +
                                QString::number(msg->status[j], 'f',1)   + " " +
                                QString::number(msg->fault_numb[j]));
      }

    }
}


void PDOWidget::imuPDOCallback(const ros::MessageEvent<ec_msgs::ImuPDO>& event)
{
    const std::string& publisher_name = event.getPublisherName();
    ros::Time receipt_time = event.getReceiptTime();

    if(receipt_time.toSec() + 1.00 < ros::Time::now().toSec())
    {
        ROS_INFO("Message too old!");
        return;
    }

    const ec_msgs::ImuPDO::ConstPtr msg = event.getConstMessage();
    if(_start_stop_btn->text()=="Stop")
    {
        double actual_pub_time=ros::Time::now().toSec()-_pdo_init_pub_time;

        for(int j=0; j < msg->slave_name.size();j++)
        {
            QTreeWidgetItem *topLevel=nullptr;

            bool found_slave=false;
            for(int i=0;i<_tree_wid->topLevelItemCount() && !found_slave;i++)
            {
                QTreeWidgetItem * topLevel_read =_tree_wid->topLevelItem(i);
                if(msg->slave_name[j]==topLevel_read->text(1).toStdString())
                {
                    topLevel=topLevel_read;
                    found_slave=true;
                }
            }

            if(!topLevel)
            {
                topLevel = new QTreeWidgetItem();
                topLevel->setText(1,QString::fromStdString(msg->slave_name[j]));
                topLevel->setText(2,"");
                topLevel->setText(3,"Rx");

                for(int index=0; index<_imu_pdo_fields.size(); index++)
                {
                    QTreeWidgetItem * item = new QTreeWidgetItem();
                    item->setText(1,_imu_pdo_fields.at(index));
                    topLevel->addChild(item);
                }

                _tree_wid->addTopLevelItem(topLevel);
            }


            topLevel->setText(0,QString::number(actual_pub_time, 'f', 3));

            _imu_pdo_fields.append("X_Quat");
            _imu_pdo_fields.append("Y_Quat");
            _imu_pdo_fields.append("Z_Quat");
            _imu_pdo_fields.append("W_Quat");
            _imu_pdo_fields.append("Status");
            _imu_pdo_fields.append("Imu_ts");
            _imu_pdo_fields.append("Temperature");
            _imu_pdo_fields.append("Digital_In");
            _imu_pdo_fields.append("Fault");


            for(int k=0; k<_imu_pdo_fields.size(); k++)
            {
                QTreeWidgetItem * item = topLevel->child(k);

                if(item->text(1)=="X_Rate")
                  item->setText(2,QString::number(msg->x_rate[j], 'f', 3));
                else if(item->text(1)=="Y_Rate")
                  item->setText(2,QString::number(msg->y_rate[j], 'f', 3));
                else if(item->text(1)=="Z_Rate")
                  item->setText(2,QString::number(msg->z_rate[j], 'f', 3));
                else if(item->text(1)=="X_Acc")
                  item->setText(2,QString::number(msg->x_acc[j], 'f', 3));
                else if(item->text(1)=="Y_Acc")
                  item->setText(2,QString::number(msg->y_acc[j], 'f', 3));
                else if(item->text(1)=="Z_Acc")
                  item->setText(2,QString::number(msg->z_acc[j], 'f', 3));
                else if(item->text(1)=="X_Quat")
                  item->setText(2,QString::number(msg->x_quat[j], 'f', 3));
                else if(item->text(1)=="Y_Quat")
                  item->setText(2,QString::number(msg->y_quat[j], 'f', 3));
                else if(item->text(1)=="Z_Quat")
                  item->setText(2,QString::number(msg->z_quat[j], 'f', 3));
                else if(item->text(1)=="W_Quat")
                  item->setText(2,QString::number(msg->w_quat[j], 'f', 3));
                else if(item->text(1)=="Imu_ts")
                  item->setText(2,QString::number(msg->imu_ts[j]));
                else if(item->text(1)=="Temperature")
                  item->setText(2,QString::number(msg->temperature[j]));
                else if(item->text(1)=="Digital_In")
                  item->setText(2,QString::number(msg->digital_in[j]));
                else if(item->text(1)=="Fault")
                  item->setText(2,QString::fromStdString(msg->fault[j]));
            }

            topLevel->setText(4,QString::number(msg->x_rate[j], 'f', 1) + " " +
                                QString::number(msg->y_rate[j], 'f', 1) + " " +
                                QString::number(msg->z_rate[j], 'f',1) + " " +
                                QString::number(msg->x_acc[j], 'f',1) + " " +
                                QString::number(msg->y_acc[j], 'f', 3)   + " " +
                                QString::number(msg->z_acc[j], 'f',1)   + " " +
                                QString::number(msg->x_quat[j], 'f', 1) + " " +
                                QString::number(msg->y_quat[j], 'f', 1) + " " +
                                QString::number(msg->z_quat[j], 'f',1) + " " +
                                QString::number(msg->w_quat[j], 'f',1) + " " +
                                QString::number(msg->imu_ts[j])   + " " +
                                QString::number(msg->temperature[j])   + " " +
                                QString::number(msg->digital_in[j])   + " " +
                                QString::number(msg->fault_numb[j]));
      }

    }
}

void PDOWidget::ftPDOCallback(const ros::MessageEvent<ec_msgs::FtPDO>& event)
{
    const std::string& publisher_name = event.getPublisherName();
    ros::Time receipt_time = event.getReceiptTime();

    if(receipt_time.toSec() + 1.00 < ros::Time::now().toSec())
    {
        ROS_INFO("Message too old!");
        return;
    }

    const ec_msgs::FtPDO::ConstPtr msg = event.getConstMessage();
    if(_start_stop_btn->text()=="Stop")
    {
        double actual_pub_time=ros::Time::now().toSec()-_pdo_init_pub_time;

        for(int j=0; j < msg->slave_name.size();j++)
        {
            QTreeWidgetItem *topLevel=nullptr;

            bool found_slave=false;
            for(int i=0;i<_tree_wid->topLevelItemCount() && !found_slave;i++)
            {
                QTreeWidgetItem * topLevel_read =_tree_wid->topLevelItem(i);
                if(msg->slave_name[j]==topLevel_read->text(1).toStdString())
                {
                    topLevel=topLevel_read;
                    found_slave=true;
                }
            }

            if(!topLevel)
            {
                topLevel = new QTreeWidgetItem();
                topLevel->setText(1,QString::fromStdString(msg->slave_name[j]));
                topLevel->setText(2,"");
                topLevel->setText(3,"Rx");

                for(int index=0; index<_ft_pdo_fields.size(); index++)
                {
                    QTreeWidgetItem * item = new QTreeWidgetItem();
                    item->setText(1,_ft_pdo_fields.at(index));
                    topLevel->addChild(item);
                }

                _tree_wid->addTopLevelItem(topLevel);
            }
            topLevel->setText(0,QString::number(actual_pub_time, 'f', 3));

            for(int k=0; k<_ft_pdo_fields.size(); k++)
            {
            QTreeWidgetItem * item = topLevel->child(k);

            if(item->text(1)=="Force_X")
              item->setText(2,QString::number(msg->force_x[j], 'f', 3));
            else if(item->text(1)=="Force_Y")
              item->setText(2,QString::number(msg->force_y[j], 'f', 3));
            else if(item->text(1)=="Force_Z")
              item->setText(2,QString::number(msg->force_z[j], 'f', 3));
            else if(item->text(1)=="Torque_X")
              item->setText(2,QString::number(msg->torque_x[j], 'f', 3));
            else if(item->text(1)=="Torque_Y")
              item->setText(2,QString::number(msg->torque_y[j], 'f', 3));
            else if(item->text(1)=="Torque_Z")
              item->setText(2,QString::number(msg->torque_z[j], 'f', 3));
            else if(item->text(1)=="Aux")
              item->setText(2,QString::number(msg->aux[j], 'f', 3));
            else if(item->text(1)=="Op_Idx_Ack")
              item->setText(2,QString::number(msg->op_idx_ack[j]));
            else if(item->text(1)=="Fault")
              item->setText(2,QString::fromStdString(msg->fault[j]));
            }

            topLevel->setText(4,QString::number(msg->force_x[j], 'f', 1) + " " +
                                QString::number(msg->force_y[j], 'f', 1) + " " +
                                QString::number(msg->force_z[j], 'f',1) + " " +
                                QString::number(msg->torque_x[j], 'f',1) + " " +
                                QString::number(msg->torque_y[j], 'f', 3)   + " " +
                                QString::number(msg->torque_z[j], 'f',1)   + " " +
                                QString::number(msg->aux[j], 'f',1)   + " " +
                                QString::number(msg->op_idx_ack[j])   + " " +
                                QString::number(msg->fault_numb[j]));
      }

    }
}

PDOWidget::PDOWidget(QWidget *parent) : QWidget(parent)
{
    /* Load ui */
    auto wid = LoadUiFile(this);

    auto l = new QVBoxLayout;
    l->addWidget(wid);

    setLayout(l);

    _node = std::make_unique<ros::NodeHandle>("");

    _tree_wid = findChild<QTreeWidget *>("PDOtreeWidget");

    if(!_tree_wid)
    {
        throw std::runtime_error("Error: PDOtreeWidget");
    }

     _motor_pdo_fields.append("Link Position");
     _motor_pdo_fields.append("Link Velocity");
     _motor_pdo_fields.append("Motor Position");
     _motor_pdo_fields.append("Motor Velocity");
     _motor_pdo_fields.append("Torque");
     _motor_pdo_fields.append("Motor Temperature");
     _motor_pdo_fields.append("Board Temperature");
     _motor_pdo_fields.append("Fault");
     _motor_pdo_fields.append("Position Reference");
     _motor_pdo_fields.append("Velocity Reference");
     _motor_pdo_fields.append("Torque Reference");

     _pow_pdo_fields.append("Voltage Battery");
     _pow_pdo_fields.append("Voltage Load");
     _pow_pdo_fields.append("Current Load");
     _pow_pdo_fields.append("Battery Temperature");
     _pow_pdo_fields.append("HeatSink Temperature");
     _pow_pdo_fields.append("PCB Temperature");
     _pow_pdo_fields.append("Status");
     _pow_pdo_fields.append("Fault");

     _imu_pdo_fields.append("X_Rate");
     _imu_pdo_fields.append("Y_Rate");
     _imu_pdo_fields.append("Z_Rate");
     _imu_pdo_fields.append("X_Acc");
     _imu_pdo_fields.append("Y_Acc");
     _imu_pdo_fields.append("Z_Acc");
     _imu_pdo_fields.append("X_Quat");
     _imu_pdo_fields.append("Y_Quat");
     _imu_pdo_fields.append("Z_Quat");
     _imu_pdo_fields.append("W_Quat");
     _imu_pdo_fields.append("Status");
     _imu_pdo_fields.append("Imu_ts");
     _imu_pdo_fields.append("Temperature");
     _imu_pdo_fields.append("Digital_In");
     _imu_pdo_fields.append("Fault");


     _ft_pdo_fields.append("Force_X");
     _ft_pdo_fields.append("Force_Y");
     _ft_pdo_fields.append("Force_Z");
     _ft_pdo_fields.append("Torque_X");
     _ft_pdo_fields.append("Torque_Y");
     _ft_pdo_fields.append("Torque_Z");
     _ft_pdo_fields.append("Aux");
     _ft_pdo_fields.append("Op_Idx_Ack");
     _ft_pdo_fields.append("Fault");

    _start_stop_btn = findChild<QPushButton *>("StartStopBtn");

    if(!_start_stop_btn)
    {
        throw std::runtime_error("Error: _start_stop_btn");
    }

    connect(_start_stop_btn, &QPushButton::released,
            this, &PDOWidget::onStartStopBtnReleased);

    _sub_motor  =_node->subscribe("ec_client/motorPDO", 1, &PDOWidget::motorPDOCallback,this);
    _sub_pow    =_node->subscribe("ec_client/powPDO", 1, &PDOWidget::powPDOCallback,this);
    _sub_imu    =_node->subscribe("ec_client/ImuPDO", 1, &PDOWidget::imuPDOCallback,this);
    _sub_ft     =_node->subscribe("ec_client/FtPDO", 1, &PDOWidget::ftPDOCallback,this);



    _rosTimer = new QTimer(this);
    connect(_rosTimer, SIGNAL(timeout()), this, SLOT(spin_ros()));
    _rosTimer->start(1);  // set your spin rate here

    onStartStopBtnReleased();

}

void PDOWidget::spin_ros()
{
    ros::spinOnce();
}


void PDOWidget::onStartStopBtnReleased()
{
    ros::ServiceClient client;
    std_srvs::Empty srv;

    if(_start_stop_btn->text()=="Start")
    {
        client=_node->serviceClient<std_srvs::Empty>("ec_client/start_pub_pdo");
         if(client.call(srv))
         {
            _start_stop_btn->setText("Stop");
            _pdo_init_pub_time=ros::Time::now().toSec();
         }
         else
         {
             std::cout << "Failed to start the pdo reading!" << std::endl;
         }
    }
    else
    {
        client=_node->serviceClient<std_srvs::Empty>("ec_client/stop_pub_pdo");
        if(client.call(srv))
        {
           _start_stop_btn->setText("Start");
           _pdo_init_pub_time=ros::Time::now().toSec();
        }
        else
        {
            std::cout << "Failed to stop the pdo reading!" << std::endl;
        }
    }
}

