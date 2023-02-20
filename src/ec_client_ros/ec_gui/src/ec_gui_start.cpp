#include "ec_gui_start.h"
#include "ec_srvs/PrintMotorInfo.h"
#include "ec_gui.h"
#include "ros/ros.h"

#include <iostream>
#include <csignal>
#include <atomic>
#include <boost/asio.hpp>
#include <stdlib.h>
#include <fstream>


#include <QLabel>
#include <QPixmap>
#include <QFile>

#include "ec_srvs/SetMotorConfigFile.h"


void ec_gui_start_widget_qrc_init()
{
    Q_INIT_RESOURCE(ec_gui_start_resources);
}

namespace
{

QWidget * LoadUiFile(QWidget * parent)
{
    ec_gui_start_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/ec_gui_start.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);

    file.close();

    return formWidget;
}

}

EcGuiStart::EcGuiStart(QWidget *parent) :
    QWidget(parent)
{

    /* Load ui */
    auto wid = LoadUiFile(this);

    _l = new QVBoxLayout;
    _l->addWidget(wid);

    auto logo = findChild<QLabel*>("logo");
    auto robot = findChild<QLabel*>("robot");

    QPixmap pic_logo(":/img/logo.png");
    logo->setPixmap(pic_logo);

    QPixmap pic_robot(":/img/robot.png");
    robot->setPixmap(pic_robot);


    _lowlevelpath_line = findChild<QLineEdit *>("LowLevelPath");

    if(!_lowlevelpath_line)
    {
        throw std::runtime_error("low level path error");
    }

    /* Open Low Level File */
    auto low_level_file = findChild<QDialogButtonBox *>("LowLevelFile");

    if(!low_level_file)
    {
        throw std::runtime_error("low level file button");
    }


   auto lowlevelfilebtn = low_level_file->button(QDialogButtonBox::Open);

   connect(lowlevelfilebtn, &QPushButton::released,
           this, &EcGuiStart::onLowLevelFileCmdReleased);

    /* Start GUI */
    auto start_gui = findChild<QDialogButtonBox *>("StartGui");

    if(!start_gui)
    {
        throw std::runtime_error("start gui button");
    }


   auto startbtn = start_gui->button(QDialogButtonBox::Ok);

   connect(startbtn, &QPushButton::released,
           this, &EcGuiStart::onStarGUICmdReleased);

   auto ignorebtn = start_gui->button(QDialogButtonBox::Ignore);

   connect(ignorebtn, &QPushButton::released,
           this, &EcGuiStart::onIgnoreCmdReleased);


    setLayout(_l);

}

QFileInfoList EcGuiStart::search_file(QDir dir,QString ch_dir,QString targetStr)
{
    QFileInfoList hitList;
    dir.cd(ch_dir);
    QString file_path=dir.absolutePath();
    QDirIterator it(file_path, QDirIterator::NoIteratorFlags);
    while (it.hasNext())
    {
        QString filename = it.next();
        QFileInfo file(filename);

        if (file.isDir()) { // Check if it's a dir
            continue;
        }

        // If the filename contains target string - put it in the hitlist
        if (file.fileName().contains(targetStr, Qt::CaseInsensitive)) {  // What we search for
            hitList.append(file);
        }
    }
    return hitList;
}

void EcGuiStart::onLowLevelFileCmdReleased()
{
    _lowlevelpath_line->clear();
    _low_level_file="";
    _joint_map_file="";
    _robot_file="";

    QFile cfg_file("/tmp/config.txt");
    if(!cfg_file.open(QIODevice::ReadWrite))
    {
        std::cout << "Error opening configuration file: " << std::endl;
        return;
    }

    QTextStream instream(&cfg_file);
    QString default_config_path = instream.readLine();

    if(default_config_path == "")
    {
        _start_dir="/home";
    }
    else
    {
        if(QDir(default_config_path).exists())
        {
            _start_dir=default_config_path;
        }
        else
        {
            _start_dir="/home";
        }

    }

    QString low_level_file = QFileDialog::getOpenFileName(this,
        tr("Open YAML"), _start_dir, tr("YAML Files (*.yaml)"), Q_NULLPTR, QFileDialog::DontUseNativeDialog);

    QFileInfoList hitList_jointmap,hitList_robotinfo; // Container for matches

    if(!low_level_file.isEmpty())
    {
        QDir low_level_dir = QFileInfo(low_level_file).absoluteDir();
        hitList_jointmap=search_file(low_level_dir,"../joint_map","joint_map.yaml");
        hitList_robotinfo=search_file(low_level_dir,"..","basic.yaml");

        cfg_file.resize(0);
        QTextStream stream(&cfg_file);
        stream << low_level_dir.absolutePath() << endl;
    }

    if(!hitList_jointmap.empty())
    {
        QString joint_map_file=hitList_jointmap.at(0).absoluteFilePath();
        _joint_map_file=joint_map_file.toStdString();
    }

    if(!hitList_robotinfo.empty())
    {
        QString robot_info_file=hitList_robotinfo.at(0).absoluteFilePath();
        _robot_file=robot_info_file.toStdString();
    }

    _low_level_file=low_level_file.toStdString();
    _lowlevelpath_line->setText(low_level_file);

    cfg_file.close();
}

std::string yaml_to_string(YAML::Node y)
{
    YAML::Emitter out;
    out << y;
    return out.c_str();
}

void EcGuiStart::startGUI()
{
    // Delete all existing widgets, if any.
    if ( _l->layout() != NULL )
    {
        QLayoutItem* item;
        while ( ( item = _l->layout()->takeAt( 0 ) ) != NULL )
        {
            delete item->widget();
            delete item;
        }
    }

    this->setMaximumSize(QWIDGETSIZE_MAX,QWIDGETSIZE_MAX);
    this->setMinimumSize(0,0);
    this->showMaximized();
    auto ec_gui=new EcGui();
    _l->addWidget(ec_gui);
}

void EcGuiStart::onIgnoreCmdReleased()
{
    ros::NodeHandle n;
    ros::ServiceClient client=n.serviceClient<ec_srvs::PrintMotorInfo>("ec_client/print_motors_info");

    ec_srvs::PrintMotorInfo srv;
    srv.request.slave_name.push_back("all");
    std::vector<ec_msgs::SlaveDescription> slaves_descr_vector;

    if(client.call(srv))
    {
        if(!srv.response.slaves_descr.empty())
        {
            EcGuiStart::startGUI();
        }
        else
        {
            QMessageBox msgBox;
            msgBox.setText("EtherCAT Client not calibrated"
                           ", please configure it!");
            msgBox.exec();
        }
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("Cannot perform print_slaves_info service!,"
                       " please retry");
        msgBox.exec();
    }

}

void EcGuiStart::onStarGUICmdReleased()
{
    if((_low_level_file!="" && _joint_map_file!="") && (_low_level_file!=_joint_map_file))
    {
        ros::NodeHandle n;
        ros::ServiceClient client;
        client= n.serviceClient<ec_srvs::SetMotorConfigFile>("/ec_client/set_slaves");

        ec_srvs::SetMotorConfigFile srv;
        try{
            auto low_level_path_yaml = YAML::LoadFile(_low_level_file);
            auto joint_map_path_yaml = YAML::LoadFile(_joint_map_file);

            std::string urdf="",srdf="",model_type="",is_floating_base="";

            if(_robot_file!="")
            {
                auto robot_path_yaml = YAML::LoadFile(_robot_file);
                auto config=XBot::ConfigOptions::FromConfigFile(_robot_file);
                urdf=config.get_urdf();
                srdf=config.get_srdf();

                model_type=robot_path_yaml["ModelInterface"]["model_type"].as<std::string>();
                is_floating_base=robot_path_yaml["ModelInterface"]["is_model_floating_base"].as<std::string>();
            }

            srv.request.low_level_cfg=yaml_to_string(low_level_path_yaml);
            srv.request.slave_map_cfg=yaml_to_string(joint_map_path_yaml);
            srv.request.robot_info.push_back(urdf);
            srv.request.robot_info.push_back(srdf);
            srv.request.robot_info.push_back(model_type);
            srv.request.robot_info.push_back(is_floating_base);

            if(client.call(srv))
            {
                EcGuiStart::startGUI();
            }
            else
            {
                QMessageBox msgBox;
                msgBox.setText("Problems to setup the ec_client!,"
                               " retry...");
                msgBox.exec();
            }
        }
        catch(std::exception &ex){
            QMessageBox msgBox;
            msgBox.setText("Wrong YAML Files"
                           ", please retry to open a proper low level configuration file, verifying the joint map file!");
            msgBox.exec();
        }
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText("Low Level File setting failed!"
                       ", please retry to open a proper low level configuration file, verifying the joint map file!");
        msgBox.exec();
    }
}
