#include <ros/ros.h>
#include "std_msgs/Int8.h"
#include <path_msgs/path_msg.h>
#include <obu_msgs/OBU_fusion.h>
#include <fstream>
#include <vector>

class PathPublisher
{
public:
    
    PathPublisher();
    ~PathPublisher(){};
    
    bool init(ros::NodeHandle nh, ros::NodeHandle nh_private);
	
	void path_state_callback(const std_msgs::Int8::ConstPtr& path_state);
	void obu_callback(const obu_msgs::OBU_fusion::ConstPtr& container);
	
	bool loadAllPathFile();
	bool pubNextPath();
	
private:
    ros::NodeHandle nh_, nh_private_;
    bool is_ready_;
    
    std::string task_list_file_;
    std::vector<std::string> all_path_;
    
    std::string state_topic_, path_topic_, obu_topic_;
    ros::Publisher path_file_name_pub_;
	
	ros::Subscriber path_state_sub_;
    std_msgs::Int8 path_tracking_state_;
    
    ros::Subscriber obu_sub_;
    bool obu_ok_;
    
    int current_task_;
    int num_task_;
    
    int passenger_position_;
    int move_pedestrian_position_;
    int light_state_;
    int destination_;
    int delivery_end_point_;
    
};
