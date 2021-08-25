//
// Created by enver on 22.05.2021.
//
#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include<nav_msgs/Path.h>
#include <dynamic_reconfigure/server.h>
#include <artificial_potential_field_planner/APFPlannerConfig.h>
#include "artificial_potential_field_planner/apf_planner.h"


ros::NodeHandlePtr nh;
navigation::global_planner::APFPlanner apf_planner;
double forcemap_width = 10, forcemap_height = 10;
void callback(artificial_potential_field_planner::APFPlannerConfig &config, uint32_t level)
{
    forcemap_width = config.forcemap_width;
    forcemap_height = config.forcemap_height;
    apf_planner.setParameters(config.attraction_gain, config.repulsion_gain, config.radius);
}

nav_msgs::Path convert_Vector2D_arr_to_path(std::vector<navigation::types::Vector2D> vectList){
    nav_msgs::Path path;
    for(const auto& v: vectList){
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = v.x;
        pose.pose.position.y = v.y;
        pose.pose.orientation.w = 1;
        path.poses.push_back(pose);
    }
    return path;
}

visualization_msgs::Marker createForceArrowMarker(Vector2D& pose, Vector2D& force){
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "test";
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.pose.position.x = pose.x;
    arrow.pose.position.y = pose.y;
    arrow.pose.position.z = 1;
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( 0, 0, atan2(force.getUnitVector().y,force.getUnitVector().x)); //Only yaw information is needed
    myQuaternion = myQuaternion.normalize();
    arrow.pose.orientation.x = myQuaternion.x();
    arrow.pose.orientation.y = myQuaternion.y();
    arrow.pose.orientation.z = myQuaternion.z();
    arrow.pose.orientation.w = myQuaternion.w();

    arrow.scale.x=0.5;
    arrow.scale.y=0.05;
    arrow.scale.z = 0.05;

    arrow.color.g = 1.0f;
    arrow.color.a = 1.0;
    arrow.color.r = 0.0f;
    arrow.color.b = 0.0f;
    return arrow;

}
ros::Publisher marker_pub;
void drawForceMap(std::vector<std::pair<Vector2D , Vector2D>>& poseForcePairList){
    visualization_msgs::MarkerArray markerArray;
    int count = 1;
    for(auto& poseForcePair: poseForcePairList){
        visualization_msgs::Marker force_marker = createForceArrowMarker(poseForcePair.first,poseForcePair.second);
        force_marker.id = count++;
        markerArray.markers.push_back(force_marker);
    }
    marker_pub.publish(markerArray);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "apf_planner");
    nh = boost::make_shared<ros::NodeHandle>();

    

    ros::Publisher path_publisher = nh->advertise<nav_msgs::Path>("/path", 1000);
    marker_pub = nh->advertise<visualization_msgs::MarkerArray>("/arrows", 100000);

    ros::Subscriber sub_map = nh->subscribe<nav_msgs::OccupancyGrid>("/map", 10, [&] (const nav_msgs::OccupancyGrid::ConstPtr& map){apf_planner.setMap(*map);});
    ros::Subscriber sub_current_pose = nh->subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10, 
                                    [&] (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& currentPose)
                                        {
                                            navigation::types::Vector2D currentPosition(currentPose->pose.pose.position.x,currentPose->pose.pose.position.y);
                                            apf_planner.setCurrentPose(currentPosition);
                                            
                                        }
                                    );
    ros::Subscriber sub_goal_pose = nh->subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, 
                                    [&] (const geometry_msgs::PoseStamped::ConstPtr& goalPose)
                                        {
                                            navigation::types::Vector2D goalPosition(goalPose->pose.position.x,goalPose->pose.position.y);
                                            nav_msgs::Path path = convert_Vector2D_arr_to_path(apf_planner.plan(goalPosition));
                                            path.header.frame_id = "/test";
                                            path_publisher.publish(path);
                                            std::vector<std::pair<Vector2D, Vector2D>> forces = apf_planner.getForceMap(forcemap_height, forcemap_width);
                                            drawForceMap(forces);
                                        }
                                    );
    dynamic_reconfigure::Server<artificial_potential_field_planner::APFPlannerConfig> server;
    dynamic_reconfigure::Server<artificial_potential_field_planner::APFPlannerConfig>::CallbackType f;
 
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);
    ros::spin();
}

