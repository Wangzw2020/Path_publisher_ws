#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "driverless_actions/DoDriverlessTaskAction.h"   // Note: "Action" is appended
#include "std_msgs/Int8.h"
#include <path_msgs/path_msg.h>

class GlobalPathPlanning
{
public:
    typedef actionlib::SimpleActionClient<driverless_actions::DoDriverlessTaskAction> DoDriverlessTaskClient;
    
    GlobalPathPlanning();
    ~GlobalPathPlanning(){};
    
    bool init(ros::NodeHandle nh, ros::NodeHandle nh_private);
    bool start();
    
private:
    void taskDoneCallback(const actionlib::SimpleClientGoalState& state,
                          const driverless_actions::DoDriverlessTaskResultConstPtr& result);
    void taskFeedbackCallback(const driverless_actions::DoDriverlessTaskFeedbackConstPtr& feedback);
    void taskActivedCallback();
    
    void path_name_callback(const path_msgs::path_msg::ConstPtr& path_msg);

private:
    ros::NodeHandle nh_, nh_private_;
    bool is_ready_;

    int task_;
    int type_;
    
    float path_resolution_;
    float expect_speed_;
    std::string roadnet_file_;
    bool path_filp_;
    
    DoDriverlessTaskClient* ac_;
    
    std::string state_topic_;
    ros::Publisher path_tracking_state_pub_;
    std_msgs::Int8 path_tracking_state_; // 0 done 1 active 2 Feedback
    
    std::string path_name_topic_;
    ros::Subscriber path_file_sub_;
};
