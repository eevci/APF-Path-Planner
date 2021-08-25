#pragma once
#include "gtest/gtest.h"
#include<ros/ros.h>
#include <artificial_potential_field_planner/apf_planner.h>


class test_apf_planner : public ::testing::Test{
public:
    test_apf_planner(){};
    ~test_apf_planner(){};
protected:
    std::shared_ptr<navigation::global_planner::APFPlanner> apf_planner;
    std::vector<Vector2D> resulted_plan;
    Vector2D current_pose;
    Vector2D goal_pose;
    void SetUp(){
        apf_planner = std::make_shared<navigation::global_planner::APFPlanner>();
        current_pose.x = 0;
        current_pose.y = 0;
        this->apf_planner->setCurrentPose(current_pose);

    }
    void TearDown(){
        //Publish resulted path
        std::cout<<"===Path to ("<<goal_pose.x<<","
        <<goal_pose.y<<")===\n";

        for(auto& point: this->resulted_plan){
            std::cout<<"("<<point.x<<","
                        <<point.y<<")\n";
        }
    }
    
};