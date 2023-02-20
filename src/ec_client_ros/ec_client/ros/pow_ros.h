#ifndef __POW_ROS__
#define __POW_ROS__

#include "ec_client_pdo.h"

#include "ros/ros.h"
#include <ros/callback_queue.h>

#include "ec_msgs/PowPDO.h"

class POW_ROS
{
public:
    
    typedef std::shared_ptr<POW_ROS> Ptr;
 
    /**
    * @brief Constructor of POW ROS Class
    * It needs of shared pointer to the ROS Node Handle for the topics/services.
    * 
    * @param node p_node: ROS Node Handle.
    */
    POW_ROS(std::shared_ptr<ros::NodeHandle> node);
    

    /**
    * @brief Destructor of POW ROS Class
    * 
    */
    ~POW_ROS(){
    };
    
    
    /**
    * @brief Return the map of power boards with EC Client PDO smart pointer, class to subscribe to the PDOs information.
    * 
    * @return std::map< std::__cxx11::string, EC_Client_PDO::Ptr >
    */
    std::map<std::string,EC_Client_PDO::Ptr> get_ec_client_pdo_map();
    
    /**
    * @brief Set the map of power boards with EC Client PDO smart pointer, class to subscribe to the PDOs information.
    * 
    * @param ec_client_pdo_map p_ec_client_pdo_map: Map of slave name and smart pointer of EC Client PDO class.
    */
    void set_ec_client_pdo_map(std::map<std::string,EC_Client_PDO::Ptr> ec_client_pdo_map);
    
    /**
    * @brief Clear PDO power board message.
    * 
    */
    void clear_pow_pdo_message();
    
    /**
    * @brief Publish power board PDOs.
    * 
    * @return bool
    */
    bool pub_pow_pdo();
    
    /**
    * @brief Return the string of the Fault from a numeric value.
    * 
    * @param fault p_fault: Numeric value of the fault
    * @return std::__cxx11::string
    */
    std::string handle_fault(uint32_t fault);
    
    
    /**
    * @brief Set the list of Power Board slaves.
    * 
    * @param internal_list_pows p_internal_list_pows: List of Power Board slaves.
    */
    void set_internal_list_pows(std::vector<std::string> internal_list_pows);


    
private:
    
    std::shared_ptr<ros::NodeHandle>  _node;

    std::vector<std::string> _internal_list_pows;
    
    std::map<std::string,EC_Client_PDO::Ptr>_ec_client_pdo_map;
    ec_msgs::PowPDO _pow_pdo;
    
    ros::Publisher _pub_pow_pdo;
    
};

#endif
