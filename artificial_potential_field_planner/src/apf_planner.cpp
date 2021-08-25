#include <artificial_potential_field_planner/apf_planner.h>
using namespace navigation::types;
namespace navigation{
    namespace global_planner{
        std::vector<Vector2D> APFPlanner::plan(const Vector2D& goal_pose)
        {
            std::vector<Vector2D> resulted_path;
            this->goal_pose = goal_pose;
            
            resulted_path.push_back(current_pose); 
            uint16_t tryCount = 0;

            while(!isCloseEnough() && tryCount < navigation::global_planner::APFPlanner::MAX_TRY_COUNT){
                Vector2D attraction_force = this->calculateAttractionForce(current_pose);
                Vector2D obstacle_force = this->calculateObstacleRepulsionForce(current_pose);
                Vector2D total_force = attraction_force.add(obstacle_force);
                current_pose = current_pose.add(total_force.getUnitVector().multiply(this->map.info.resolution));
                resulted_path.push_back(current_pose);
                tryCount++;
            }
            return resulted_path;
        }

        bool APFPlanner::isCloseEnough(){
            return goal_pose.substract(current_pose).getLength() < this->map.info.resolution;
        }

        Vector2D APFPlanner::calculateAttractionForce(Vector2D& pose){
            return goal_pose.substract(pose).multiply(this->attraction_gain);
        }

        Vector2D APFPlanner::calculateObstacleRepulsionForce(Vector2D& pose){
            float result = 0.0f;
            double r = this->radius*this->map.info.resolution;
            Vector2D potentialResult(0,0);

            for(int i = 0; i< this->map.info.height; i++){
                for(int j = 0; j< this->map.info.width; j++){
                    if(this->isObstacle(i,j)){ 
                        Vector2D obsVector(j*this->map.info.resolution, i*this->map.info.resolution);
                        Vector2D diffVector = pose.substract(obsVector); 
                        if(diffVector.getLength()<=r){
                            potentialResult = potentialResult.add(
                                diffVector.getUnitVector().multiply(this->repulsion_gain).
                                multiply((1.0 / diffVector.getLength() - 1.0 / r) / (diffVector.getLength() * diffVector.getLength())));
                        }
                    }
                }    
            }
            return potentialResult;
        }

        bool APFPlanner::isObstacle(const int& i, const int& j){
            return this->map.data[i*this->map.info.height + j];
        }

        std::vector<std::pair<Vector2D, Vector2D>> APFPlanner::getForceMap(const int height,const int width){
            std::vector<std::pair<Vector2D, Vector2D>> poseForcePairList;
            Vector2D pose;
             for(double i = current_pose.x-height/2.0; i<current_pose.x+height/2.0; i+=0.5){
                for(double j =  current_pose.y-width/2.0; j<current_pose.y+width/2.0; j+=0.5){
                    pose = Vector2D(i,j);
                    Vector2D total_force(0,0);
                    Vector2D attraction_force = this->calculateAttractionForce(pose);
                    Vector2D obstacle_force = this->calculateObstacleRepulsionForce(pose);
                    total_force = attraction_force.add(obstacle_force);
                    poseForcePairList.push_back({pose,total_force});
                }
            }
            return poseForcePairList;
        }

        void APFPlanner::setParameters(double attraction_gain, double repulsion_gain, double radius){
            this->attraction_gain = attraction_gain;
            this->repulsion_gain = repulsion_gain;
            this->radius = radius;
        }
    }
}
