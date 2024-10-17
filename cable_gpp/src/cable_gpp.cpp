#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <cable_gpp/cable_gpp.h>

PLUGINLIB_EXPORT_CLASS(cable_gpp_ns::cable_gpp, mbf_costmap_core::CostmapPlanner)

// using namespace std;

namespace cable_gpp_ns
{
    cable_gpp::cable_gpp()
    {
    }
    
    cable_gpp::cable_gpp(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        // initialize(name, costmap_ros);
        ROS_INFO_STREAM("Constructor");
    }

    void cable_gpp::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        ROS_INFO_STREAM("Initialize");
    }

    // void cable_gpp::initialize(std::string name, 
    //                         costmap_2d::Costmap2DROS *costmap_ros,
    //                         std::string global_frame)
    // {
        
    // }

    uint32_t cable_gpp::makePlan(const geometry_msgs::PoseStamped &start,
                        const geometry_msgs::PoseStamped &goal,
                        double tolerance,
                        std::vector<geometry_msgs::PoseStamped> &plan,
                        double &cost,
                        std::string &message)
    {
        ROS_INFO_STREAM("MAKEPLAN2");
        plan.push_back(start);
        for (int i=0; i<20; i++){
            geometry_msgs::PoseStamped new_goal = goal;
            tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

            new_goal.pose.position.x = -2.5 + (0.05 * i);
            new_goal.pose.position.y = -3.5 + (0.05 * i);
            ROS_INFO_STREAM("MAKEPLAN2");
            new_goal.pose.orientation.x = goal_quat.x();;
            new_goal.pose.orientation.y = goal_quat.y();
            new_goal.pose.orientation.z = goal_quat.z();
            new_goal.pose.orientation.w = goal_quat.w();

            plan.push_back(new_goal);
        }
        plan.push_back(goal);
        
        return true;
    }

    bool cable_gpp::cancel()
    {
        return false;
    }
}