#include "ec_gui_start.h"
#include <QApplication>

#include <ros/ros.h>
#define WAIT_TIME_EC_CLIENT 60

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ec_gui",ros::init_options::NoSigintHandler);
    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
    QApplication a(argc, argv);

    std::vector< std::string > nodes;
    bool ec_client_node=false;
    int count=0;
    while(!ec_client_node)
    {
        if(ros::master::getNodes(nodes))
        {
            for (std::vector<std::string>::iterator it = nodes.begin() ; it != nodes.end(); ++it)
            {
                if(*it=="/ec_client")
                {
                    std::cout << "Found ec_client alive" << std::endl;
                    ec_client_node=true;
                }
            }
            if(!ec_client_node)
            {
                if(count==0)
                    std::cout << "... please start the ec_client node to use the gui" << std::endl;
                usleep(1000000); // wait 1s
                count++;
                if(count==WAIT_TIME_EC_CLIENT)
                {
                    std::cout << "... cannot reach the ec_client node" << std::endl;
                    return 1;
                }
            }
        }
    }

    EcGuiStart w;
    w.setFixedSize(773,524);
    w.show();


    return a.exec();
}
