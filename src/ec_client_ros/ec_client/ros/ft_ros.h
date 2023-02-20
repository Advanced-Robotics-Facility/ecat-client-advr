#ifndef __FT_ROS__
#define __FT_ROS__

#include "ec_client_pdo.h"

#include "ros/ros.h"
#include <ros/callback_queue.h>

#include "ec_msgs/FtPDO.h"

class FT_ROS
{
public:
    
    typedef std::shared_ptr<FT_ROS> Ptr;
 
    /**
    * @brief Constructor of FT ROS Class
    * It needs of shared pointer to the ROS Node Handle for the topics/services.
    * 
    * @param node p_node: ROS Node Handle.
    */
    FT_ROS(std::shared_ptr<ros::NodeHandle> node);
    

    /**
    * @brief Destructor of FT ROS Class
    * 
    */
    ~FT_ROS(){
    };
    
    
    /**
    * @brief Return the map of force torque sensors with EC Client PDO smart pointer, class to subscribe to the PDOs information.
    * 
    * @return std::map< std::__cxx11::string, EC_Client_PDO::Ptr >
    */
    std::map<std::string,EC_Client_PDO::Ptr> get_ec_client_pdo_map();
    
    /**
    * @brief Set the map of force torque sensors with EC Client PDO smart pointer, class to subscribe to the PDOs information.
    * 
    * @param ec_client_pdo_map p_ec_client_pdo_map: Map of slave name and smart pointer of EC Client PDO class.
    */
    void set_ec_client_pdo_map(std::map<std::string,EC_Client_PDO::Ptr> ec_client_pdo_map);
    
    /**
    * @brief Clear PDO force torque sensor message.
    * 
    */
    void clear_ft_pdo_message();
    
    /**
    * @brief Publish force torque sensor PDOs.
    * 
    * @return bool
    */
    bool pub_ft_pdo();
    
    /**
    * @brief Return the string of the Fault from a numeric value.
    * 
    * @param fault p_fault: Numeric value of the fault
    * @return std::__cxx11::string
    */
    std::string handle_fault(uint32_t fault);
    
    

    /**
    * @brief Set the list of ft slaves.
    * 
    * @param internal_list_fts p_internal_list_fts: List of ft slaves.
    */
    void set_internal_list_fts(std::vector<std::string> internal_list_fts);


    
private:
    
    std::shared_ptr<ros::NodeHandle>  _node;

    std::vector<std::string> _internal_list_fts;
    
    std::map<std::string,EC_Client_PDO::Ptr>_ec_client_pdo_map;
    ec_msgs::FtPDO _ft_pdo;
    
    ros::Publisher _pub_ft_pdo;
    
};

#endif
