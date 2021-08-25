#pragma once 

#include <base_planner/common/types/vector_2D.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace navigation::types;
namespace navigation{
    namespace global_planner{
        class BasePlanner{
            public: 
                virtual std::vector<Vector2D> plan(const Vector2D& goal_pose) = 0;
                virtual void setCurrentPose(const Vector2D& start_pose){
                    this->current_pose = start_pose;
                    std::cout<<"Current Pose:"<<this->current_pose.x<<"-"<<this->current_pose.y<<"\n";
                }
                virtual void setMap(nav_msgs::OccupancyGrid map){
                    this->map = map;
                }

            protected:
                nav_msgs::OccupancyGrid map;
                int resolution = 1;
                Vector2D goal_pose;
                Vector2D current_pose;
        };
    }
}