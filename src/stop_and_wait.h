#ifndef TRUCK_TRAILER_RECOVERY_CPP
#define TRUCK_TRAILER_RECOVERY_CPP

#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/costmap_model.h>

namespace stop_and_wait{
    class StopAndWait : public nav_core::RecoveryBehavior{
       public:
           StopAndWait();
           ~StopAndWait();
           
           void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);
           void runBehavior();
        private:
            costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
            std::string name_;
            tf2_ros::Buffer* tf_;
            bool initialized_;
            double frequency_;
            //base_local_planner::CostmapModel* world_model_;
       
         
    };
  
};
#endif
