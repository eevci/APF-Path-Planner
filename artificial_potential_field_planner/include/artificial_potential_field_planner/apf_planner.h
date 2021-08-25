#pragma once

#include <base_planner/base_planner.h>

using namespace navigation::types;

namespace navigation{
    namespace global_planner{
        class APFPlanner :public BasePlanner{
        static constexpr uint16_t MAX_TRY_COUNT = 10000;

            public:
                std::vector<Vector2D> plan(const Vector2D& goal_pose);
                std::vector<std::pair<Vector2D, Vector2D>> getForceMap(const int height=10,const int width=10);
                void setParameters(double attraction_gain, double repulsion_gain, double radius);
            private:
                // fields
                double attraction_gain = 10;
                double repulsion_gain = 100;
                double radius = 10;
                
                // functions
                Vector2D calculateAttractionForce(Vector2D& pose);
                Vector2D calculateObstacleRepulsionForce(Vector2D& pose);
                bool isCloseEnough();
                bool isObstacle(const int& i, const int& j);
        };

    }
}