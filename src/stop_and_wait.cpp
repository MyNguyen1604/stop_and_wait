#include <pluginlib/class_list_macros.h>
#include "stop_and_wait.h"

PLUGINLIB_EXPORT_CLASS(stop_and_wait::StopAndWait, nav_core::RecoveryBehavior)

namespace stop_and_wait{
    //constructor
    StopAndWait::StopAndWait(): global_costmap_(NULL), local_costmap_(NULL), tf_(NULL), initialized_(false)/*, world_model_(NULL)*/ {}
    //destructor
    StopAndWait::~StopAndWait(){
         //delete world_model_;
         ROS_INFO("Clear all memory using by recovery behavior!");
    }
    
    //Initializing recovery behavior
    void StopAndWait::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
       if(!initialized_){
         name_ = name;
         tf_ = tf;
         global_costmap_ = global_costmap;
         local_costmap_ = local_costmap;
         
         ros::NodeHandle nh("~/"+name_);
         nh.param("frequency", frequency_, 5.0);
         //world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());
         initialized_ = true;
         ROS_INFO("Using stop and wait recovery");
        }
        else{
         ROS_ERROR("Not call initialize twice... Doing nothing!");
        }//end if
    }//end initialize() function
    
    //Run recovery behavior
    void StopAndWait::runBehavior(){
        //check sonething
        if(!initialized_){
         ROS_ERROR("Recovery behavior is not initialized... Please reinitialize!");
         return;
        }
        if(global_costmap_ == NULL || local_costmap_ == NULL){
         ROS_ERROR("Receive costmaps are null... Doing nothing!");
         return;
        }
        ROS_WARN("The robot perceives itself as stuck...");
        ROS_WARN("Stop and wait for obstacles removed...");
        
        ros::Rate r(frequency_);
        ros::NodeHandle n;
        ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("cmd_vel",10);
        const double WAIT_TIME = 20.0, CLOCK_SPEED = 1.0/frequency_, MOVE_TIME = 3.0;
        int count = 0;
        geometry_msgs::PoseStamped global_pose;
        local_costmap_->getRobotPose(global_pose);
        ROS_WARN("Robot pose (x, y): (%f, %f)",global_pose.pose.position.x, global_pose.pose.position.y);
        
        //Stop
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        pub_vel.publish(cmd_vel);
        
        ROS_WARN("Waiting for %f second to remove obstacles...", WAIT_TIME);
        //Waiting for a few second        
        //ros::Duration(WAIT_TIME).sleep();
         
        //Move backward a little bit
        while(n.ok() && (count < WAIT_TIME/CLOCK_SPEED + 1)){
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            pub_vel.publish(cmd_vel);
            count = count + 1;
            r.sleep();
        }
        
        count = 0;
        while(n.ok() && (count < MOVE_TIME/CLOCK_SPEED + 1)){
            cmd_vel.linear.x = -0.09;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            pub_vel.publish(cmd_vel);
            count = count + 1;
            r.sleep();
        }
        
        count = 0;
        while(n.ok() && (count < (MOVE_TIME/2*CLOCK_SPEED) + 1)){
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.2;
            pub_vel.publish(cmd_vel);
            count = count + 1;
            r.sleep();
        }
        count = 0;
        while(n.ok() && (count < (MOVE_TIME/CLOCK_SPEED) + 1)){
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = -0.2;
            pub_vel.publish(cmd_vel);
            count = count + 1;
            r.sleep();
        }
        count = 0;
        while(n.ok() && (count < MOVE_TIME/(2*CLOCK_SPEED) + 1)){
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.2;
            pub_vel.publish(cmd_vel);
            count = count + 1;
            r.sleep();
        }
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        pub_vel.publish(cmd_vel);
        ros::Duration(2.0).sleep();
        return;
    
    }

};
