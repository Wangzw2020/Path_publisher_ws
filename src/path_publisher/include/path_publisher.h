#include <ros/ros.h>
#include "std_msgs/Int8.h"
#include <path_msgs/path_msg.h>
#include <fstream>
#include <vector>

class PathPublisher
{
public:
    
    PathPublisher();
    ~PathPublisher(){};
    
    bool init(ros::NodeHandle nh, ros::NodeHandle nh_private);
	
	void path_state_callback(const std_msgs::Int8::ConstPtr& path_state);
	
	bool loadAllPathFile();
	bool pubNextPath();
	
private:
    ros::NodeHandle nh_, nh_private_;
    bool is_ready_;
    
    std::string task_list_file_;
    std::vector<std::string> all_path_;
    
    std::string state_topic_, path_topic_;
    ros::Publisher path_file_name_pub_;
	
	ros::Subscriber path_state_sub_;
    std_msgs::Int8 path_tracking_state_;
    
    int current_task_;
    int num_task_;
};
