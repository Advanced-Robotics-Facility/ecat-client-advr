#ifndef _ROBOT_VIZ_H_
#define _ROBOT_VIZ_H_

#include <XBotInterface/ModelInterface.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

class RobotViz
{

public:

    typedef Eigen::Vector4d color; //rgba

    typedef std::shared_ptr<RobotViz> Ptr;

    /**
     * @brief _reserved_color for the self collision
     */
    const color _reserved_color = (color() << 1.0, 0.0, 0.0, 1.0).finished();

    /**
     * @brief RobotViz
     * @param model model of the robot to publish as marker
     * @param topic_name topic of the published marker
     * @param nh to retrieve the namespace of the topic
     * @param rgba color of the robot published
     */
    RobotViz(const XBot::ModelInterface::ConstPtr model,
             const std::string& topic_name,
             ros::NodeHandle& nh,
             const color& rgba = (color() << 0.0, 1.0, 0.0, 0.5).finished());

    /**
     * @brief setPrefix
     * @param prefix added to header frame id of the marker (eg "planner/")
     */
    void setPrefix(const std::string& prefix);

    /**
     * @brief getPrefix
     * @return actual prefix
     */
    std::string getPrefix();

    /**
     * @brief setRGBA
     * @param rgba [Red, Green, Blue, Alpha]
     */
    void setRGBA(const color& rgba);

    /**
     * @brief publishMarkers of the robot with a time
     * @param time
     * @param red_links these links will be publisged with the _reserved_color
     */
    void publishMarkers(const ros::Time& time, const std::vector<std::string>& red_links);

private:

    XBot::ModelInterface::ConstPtr _model;
    ros::NodeHandle _nh;
    ros::Publisher ec_client_robot_pub;
    std::string _prefix;
    color _rgba;

    static Eigen::Affine3d toAffine3d(const urdf::Pose& p);

};

#endif

