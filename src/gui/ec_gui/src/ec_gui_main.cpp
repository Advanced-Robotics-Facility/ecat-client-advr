#include <stdlib.h>
#include <boost/asio.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "ec_gui_start.h"
#include <QApplication>

#include "ec_utils.h"

int main(int argc, char *argv[])
{
    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
    QApplication a(argc, argv);

    std::map<int ,EcGuiSlider::joint_info_t > joint_info_map;

    // ********************* TEST ****************************////
#ifdef TEST
    for(int i=11; i<37; i++)
    {

        EcGuiSlider::joint_info_t joint_info_s;

        joint_info_s.joint_name    ="joint_id_"+std::to_string(i);
        joint_info_s.actual_pos    =0.0;
        joint_info_s.min_pos       =-1.0;
        joint_info_s.max_pos       =1.0;
        joint_info_s.actual_vel    =0.0;
        joint_info_s.max_vel       =2.0;
        joint_info_s.actual_torq   =0.0;
        joint_info_s.max_torq      =5.0;

        joint_info_map[i]=joint_info_s;
    }
#endif
    // ********************* TEST ****************************////


    // Find the ec client configuration environment variable for setting
    EcUtils::Ptr ec_client_utils;
    EcUtils::EC_CONFIG ec_client_cfg;
    
    char* ec_client_cfg_path;
    ec_client_cfg_path = getenv ("EC_CLIENT_CFG");
    
    int ret_gui_start=0;

    if (ec_client_cfg_path==NULL)
    {
        QMessageBox msgBox;
        msgBox.setText("EC Client configuration is not found"
                       ", please setup it using the environment variable with name: EC_CLIENT_CFG");
        msgBox.exec();
    }
    else
    {
        try{
        auto ec_client_cfg_file = YAML::LoadFile(ec_client_cfg_path);
        ec_client_utils=std::make_shared<EcUtils>(ec_client_cfg_file);
        ec_client_cfg = ec_client_utils->get_ec_cfg();   
            
        }catch(std::exception &ex){
            
        QMessageBox msgBox;
        QString error="Error on ec client config file, "+QString::fromStdString(ex.what());
        msgBox.setText(error);
        msgBox.exec();
        return 1;
        }
        // *************** AUTODETECTION *************** //
        EcIface::Ptr client=ec_client_utils->make_ec_iface();
        
        if(!joint_info_map.empty())
        {

            // START the GUI
            QMainWindow mw;
            EcGuiStart w(joint_info_map,ec_client_cfg,client);
            w.setMinimumSize(1000,1000);
            mw.setCentralWidget(w.centralWidget());
            mw.setWindowState(Qt::WindowMaximized);
            mw.show();

            ret_gui_start = a.exec();
        }
        else
        {
            if(client->is_client_alive())
            {
                client->stop_client();
            }
        }
    }
    
    return ret_gui_start;
}
