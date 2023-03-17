#include <stdlib.h>
#include <boost/asio.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "ec_gui_start.h"
#include <QApplication>

#include "client.h"
#include "ec_client_utils.h"

#define CENT_AC 0x15

int main(int argc, char *argv[])
{
    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
    QApplication a(argc, argv);

    std::map<int ,EcGuiStart::joint_info_t > joint_info_map;

    // ********************* TEST ****************************////
#ifdef TEST
    for(int i=11; i<37; i++)
    {

        EcGuiStart::joint_info_t joint_info_s;

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
    EC_Client_Utils::Ptr ec_client_utils;
    EC_Client_Utils::EC_CONFIG ec_client_cfg;
    
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
        ec_client_utils=std::make_shared<EC_Client_Utils>(ec_client_cfg_file);
        ec_client_cfg = ec_client_utils->get_ec_client_config();   
            
        }catch(std::exception &ex){
            
        QMessageBox msgBox;
        QString error="Error on ec client config file, "+QString::fromStdString(ex.what());
        msgBox.setText(error);
        msgBox.exec();
        return 1;
        }
        // *************** AUTODETECTION *************** //

        createLogger("console","GUI");
        auto client=std::make_shared<Client>(ec_client_cfg.host_name_s,ec_client_cfg.host_port);
        auto UDP_period_ms_time=std::chrono::milliseconds(ec_client_cfg.UDP_period_ms);
        
        client->set_period(UDP_period_ms_time);
        client->connect();

        // Run asio thread alongside GUI thread
        std::thread t1{[&]{client->run();}};
        
        SSI slave_info;
        
        if(client->retrieve_slaves_info(slave_info))
        {
            if(slave_info.empty())
            {
                QMessageBox msgBox;
                msgBox.setText("Cannot find Joints on the EtherCAT network"
                            ", please control the EtherCAT Master and restart the GUI");
                msgBox.exec();
            }
            else
            {

                // *************** END AUTODETECTION *************** //

                // GET Mechanical Limits
                joint_info_map.clear();
                std::map<int,RR_SDO> motor_info_map;
                RD_SDO rd_sdo = { "motor_pos","Min_pos","Max_pos","motor_vel","Max_vel","torque","Max_tor"};
                WR_SDO wr_sdo = {};
                int motors_counter=0;
                
                for ( auto &[slave_id, type, pos] : slave_info )
                {
                    if(type==CENT_AC)
                    {
                        motors_counter++;
                        
                        RR_SDO rr_sdo_info;
                        if(client->retrieve_rr_sdo(slave_id,rd_sdo,wr_sdo,rr_sdo_info))
                        {    
                            if(!rr_sdo_info.empty())
                            {
                                motor_info_map[slave_id]=rr_sdo_info;
                            }
                        }
                    }

                }

                if((motor_info_map.size()!=motors_counter)|| (motor_info_map.empty()))
                {
                    QMessageBox msgBox;
                    msgBox.setText("Cannot find the SDO information requested, mechanical limits and actual position, velocity and torque for all motors"
                                ", please control the EtherCAT Slave setup and restart the GUI");
                    msgBox.exec();
                }
                else
                {
                    for ( auto &[slave_id, type, pos] : slave_info )
                    {
                        if(motor_info_map.count(slave_id)>0)
                        {
                            std::map<std::string,float> slaves_sdo_data=motor_info_map[slave_id];

                            EcGuiStart::joint_info_t joint_info_s;

                            joint_info_s.joint_name    ="joint_id_"+std::to_string(slave_id);
                            joint_info_s.actual_pos    =slaves_sdo_data.at("motor_pos");
                            joint_info_s.min_pos       =slaves_sdo_data.at("Min_pos");
                            joint_info_s.max_pos       =slaves_sdo_data.at("Max_pos");
                            if(joint_info_s.max_pos<joint_info_s.min_pos)
                            {
                            double aux_value=joint_info_s.min_pos;
                            joint_info_s.min_pos=joint_info_s.max_pos;
                            joint_info_s.max_pos=aux_value;
                            }
                            joint_info_s.actual_vel    =slaves_sdo_data.at("motor_vel");
                            joint_info_s.max_vel       =slaves_sdo_data.at("Max_vel");
                            joint_info_s.actual_torq   =slaves_sdo_data.at("torque");
                            joint_info_s.max_torq      =slaves_sdo_data.at("Max_tor");


                            joint_info_map[slave_id]=joint_info_s;
                            // parse the message taking the information requested. Save it into the joint_info_map.
                        }
                    }
                }
            }
        }
        
        if(!joint_info_map.empty())
        {

            // START the GUI
            EcGuiStart w(joint_info_map,ec_client_cfg,client);

            //w.setFixedSize(1600,1000);
            //w.show();

            ret_gui_start = a.exec();
        }
        else
        {
            if(client->is_client_alive())
            {
                client->stop_client();
            }
        }
        
        if ( t1.joinable() ) 
        {
            std::cout << "Client thread stopped" << std::endl;
            t1.join();
        }
    }
    
    return ret_gui_start;
}
