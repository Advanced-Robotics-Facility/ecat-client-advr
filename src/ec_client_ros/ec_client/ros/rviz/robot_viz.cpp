#include "robot_viz.h"

RobotViz::RobotViz(const XBot::ModelInterface::ConstPtr model, const std::string & topic_name, ros::NodeHandle & nh, const RobotViz::color & rgba):
    _model(model),
    _nh(nh),
    _prefix("")
{
    ec_client_robot_pub = _nh.advertise<visualization_msgs::MarkerArray>( topic_name, 0, true);

    if(rgba != _reserved_color)
        _rgba = rgba;
}

void RobotViz::setPrefix(const std::string & prefix)
{
    _prefix = prefix;
}

std::string RobotViz::getPrefix()
{
    return _prefix;
}

void RobotViz::setRGBA(const color& rgba)
{
    _rgba = rgba;
}

void RobotViz::publishMarkers(const ros::Time & time, const std::vector<std::string> & red_links)
{

#if ROS_VERSION_MINOR <= 12
#define STATIC_POINTER_CAST boost::static_pointer_cast
#else
#define STATIC_POINTER_CAST std::static_pointer_cast
#endif

    visualization_msgs::MarkerArray markers;

    std::string bl = "base_link"; // _model->getFloatingBaseLink(bl);

    ros::Time t = time;

    std::vector<urdf::LinkSharedPtr> links;
    if(!_model)
        return;
    
    _model->getUrdf().getLinks(links);

    int id = 0;
    for(auto link : links)
    {
        if(link->collision)
        {
            visualization_msgs::Marker marker;
            
            marker.header.frame_id = _prefix+bl;
            marker.header.stamp = t;
            marker.ns = "visual";
            marker.id = id;

            marker.action = visualization_msgs::Marker::ADD;
            
            Eigen::Affine3d pose; 
            _model->getPose(link->name, bl, pose);
            pose = pose*toAffine3d(link->visual->origin);
            

            marker.pose.position.x = pose.translation()[0];
            marker.pose.position.y = pose.translation()[1];
            marker.pose.position.z = pose.translation()[2];
            
            Eigen::Quaterniond q(pose.linear());
            
            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.pose.orientation.w = q.w();

            if(std::find(red_links.begin(), red_links.end(), link->name) != red_links.end())
            {
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }
            else
            {
                marker.color.a = 0.5;
                marker.color.r = 0.5;
                marker.color.g = 0.5;
                marker.color.b = 0.5;
            }

            if(link->visual->geometry->type == urdf::Geometry::BOX)
            {
                marker.type = visualization_msgs::Marker::CUBE;

                auto mesh =
                        STATIC_POINTER_CAST<urdf::Box>(link->visual->geometry);

                marker.scale.x = mesh->dim.x;
                marker.scale.y = mesh->dim.y;
                marker.scale.z = mesh->dim.z;
            }
            else if(link->visual->geometry->type == urdf::Geometry::CYLINDER)
            {
                marker.type = visualization_msgs::Marker::CYLINDER;

                auto mesh =
                        STATIC_POINTER_CAST<urdf::Cylinder>(link->visual->geometry);

                marker.scale.x = marker.scale.y = 2.*mesh->radius;
                marker.scale.z = mesh->length;
            }
            else if(link->visual->geometry->type == urdf::Geometry::SPHERE)
            {
                marker.type = visualization_msgs::Marker::SPHERE;

                auto mesh =
                        STATIC_POINTER_CAST<urdf::Sphere>(link->visual->geometry);

                marker.scale.x = marker.scale.y = marker.scale.z = 2.*mesh->radius;
            }
            else if(link->visual->geometry->type == urdf::Geometry::MESH)
            {
                marker.type = visualization_msgs::Marker::MESH_RESOURCE;


                auto mesh =
                        STATIC_POINTER_CAST<urdf::Mesh>(link->visual->geometry);

                marker.mesh_resource = mesh->filename;
                marker.scale.x = mesh->scale.x;
                marker.scale.y = mesh->scale.y;
                marker.scale.z = mesh->scale.z;
            }
            markers.markers.push_back(marker);
            id++;
            
            marker.id = id;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            
            marker.text=link->name;
            
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.1;
            
            marker.color.a = 0.6;
            marker.color.r = 0;
            marker.color.g = 0;
            marker.color.b = 0;
            
            markers.markers.push_back(marker);
            id++;
            
        }
    }
    ec_client_robot_pub.publish(markers);
}

Eigen::Affine3d RobotViz::toAffine3d(const urdf::Pose & p)
{
    Eigen::Affine3d T;

    T.translation()[0] = p.position.x;
    T.translation()[1] = p.position.y;
    T.translation()[2] = p.position.z;

    T.linear() = Eigen::Matrix3d(Eigen::Quaterniond(p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z));
    return T;
}
