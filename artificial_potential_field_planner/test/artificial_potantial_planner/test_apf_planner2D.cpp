#include "test_apf_planner2D.h"


TEST_F(test_apf_planner, emptyMapTest) {
    this->goal_pose.x = 50;
    this->goal_pose.y = 25;
    std::vector<int8_t> map_data(10000,0);
    nav_msgs::OccupancyGrid map;
    map.data = map_data;
    map.info.resolution = 1;
    map.info.width = 100;
    map.info.height = 100;
    this->apf_planner->setMap(map);
    this->resulted_plan = this->apf_planner->plan(this->goal_pose);
    EXPECT_NEAR(resulted_plan.back().x, goal_pose.x, map.info.resolution);
    EXPECT_NEAR(resulted_plan.back().y, goal_pose.y, map.info.resolution);
}

TEST_F(test_apf_planner, obstacleInTheMiddleOfMapTest) {
    this->goal_pose.x = 80;
    this->goal_pose.y = 80;
    std::vector<int8_t> map_data(10000,0);
    for(int i=30; i<70;i++ )
        map_data[49*100 + i]=100;
    nav_msgs::OccupancyGrid map;
    map.data = map_data;
    map.info.resolution = 1;
    map.info.width = 100;
    map.info.height = 100;
    this->apf_planner->setMap(map);
    this->resulted_plan = this->apf_planner->plan(this->goal_pose);
    EXPECT_NEAR(resulted_plan.back().x, goal_pose.x, map.info.resolution);
    EXPECT_NEAR(resulted_plan.back().y, goal_pose.y, map.info.resolution);
}