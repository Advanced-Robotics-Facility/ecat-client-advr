#ifndef __IMU_ROS__
#define __IMU_ROS__

#include "ec_client_pdo.h"

#include "ros/ros.h"
#include <ros/callback_queue.h>

#include "ec_msgs/ImuPDO.h"

class IMU_ROS
{
public:
    
    typedef std::shared_ptr<IMU_ROS> Ptr;
 
    /**
    * @brief Constructor of IMU ROS Class
    * It needs of shared pointer to the ROS Node Handle for the topics/services.
    * 
    * @param node p_node: ROS Node Handle.
    */
    IMU_ROS(std::shared_ptr<ros::NodeHandle> node);
    

    /**
    * @brief Destructor of IMU ROS Class
    * 
    */
    ~IMU_ROS(){
    };
    
    
    /**
    * @brief Return the map of imus with EC Client PDO smart pointer, class to subscribe to the PDOs information.
    * 
    * @return std::map< std::__cxx11::string, EC_Client_PDO::Ptr >
    */
    std::map<std::string,EC_Client_PDO::Ptr> get_ec_client_pdo_map();
    
    /**
    * @brief Set the map of imus with EC Client PDO smart pointer, class to subscribe to the PDOs information.
    * 
    * @param ec_client_pdo_map p_ec_client_pdo_map: Map of slave name and smart pointer of EC Client PDO class.
    */
    void set_ec_client_pdo_map(std::map<std::string,EC_Client_PDO::Ptr> ec_client_pdo_map);
    
    /**
    * @brief Clear PDO imu message.
    * 
    */
    void clear_imu_pdo_message();
    
    /**
    * @brief Publish imu PDOs.
    * 
    * @return bool
    */
    bool pub_imu_pdo();
    
    /**
    * @brief Return the string of the Fault from a numeric value.
    * 
    * @param fault p_fault: Numeric value of the fault
    * @return std::__cxx11::string
    */
    std::string handle_fault(uint32_t fault);
    
    
    /**
    * @brief Set the list of imu slaves.
    * 
    * @param internal_list_imus p_internal_list_imus: List of imu slaves.
    */
    void set_internal_list_imus(std::vector<std::string> internal_list_imus);


    
private:
    
    std::shared_ptr<ros::NodeHandle>  _node;

    std::vector<std::string> _internal_list_imus;
    
    std::map<std::string,EC_Client_PDO::Ptr>_ec_client_pdo_map;
    ec_msgs::ImuPDO _imu_pdo;
    
    ros::Publisher _pub_imu_pdo;
    
};

#endif
