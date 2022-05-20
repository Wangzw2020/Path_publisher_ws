#include <ros/ros.h>
#include "std_msgs/Int8.h"
#include <path_msgs/path_msg.h>

class PathPublisher
{
public:
    
    PathPublisher();
    ~PathPublisher(){};
    
    bool init(ros::NodeHandle nh, ros::NodeHandle nh_private);
	
	void path_state_callback(const std_msgs::Int8::ConstPtr& path_state);
	
private:
    ros::NodeHandle nh_, nh_private_;
    bool is_ready_;
    
    std::string state_topic_, path_name_topic_;
    ros::Publisher path_file_name_pub_;
	
	ros::Subscriber path_state_sub_;
    std_msgs::Int8 path_tracking_state_;
    
};
