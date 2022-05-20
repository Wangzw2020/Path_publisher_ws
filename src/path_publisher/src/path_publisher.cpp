#include <path_publisher.h>
#define __NAME__ "path_publisher"

PathPublisher::PathPublisher()
{

}

bool PathPublisher::init(ros::NodeHandle nh, ros::NodeHandle nh_private)
{
	nh_ = nh;
	nh_private_ = nh_private;
	
	nh_private_.param<std::string>("state_topic", state_topic_, " ");
	nh_private_.param<std::string>("path_name_topic", path_name_topic_, " ");
	
	path_file_name_pub_ = nh_.advertise<path_msgs::path_msg>(path_name_topic_, 1);
	
	path_state_sub_ = nh_.subscribe(state_topic_, 1, &PathPublisher::path_state_callback, this);
	
	is_ready_ = true;
	ROS_INFO("[%s] Path publisher is initialed!", __NAME__);
	return true;
}

void PathPublisher::path_state_callback(const std_msgs::Int8::ConstPtr& path_state)
{
	if (!is_ready_)
	{
		ROS_ERROR("[%s] System is not ready!", __NAME__);
		return;
	}
	int state;
	state = path_state->data;
	
	if(state == 0)		//done
	{
		std::cout << "state = 0  Task is done!" << std::endl;
		
		path_msgs::path_msg next_path;
		next_path.task = next_path.DRIVE_TASK;
		next_path.type = next_path.FILE_TYPE;
		next_path.path_resolution = 0.1;
		next_path.expect_speed = 15;
		next_path.roadnet_file = "a.txt";
		next_path.path_filp = false;
		path_file_name_pub_.publish(next_path);
		
		std::cout << "New task is sent!" << std::endl;
	}
	else if(state == 1)
	{
		std::cout << "state = 1  Task starts!" << std::endl;
	}
	else if(state == 2)
	{
		std::cout << "state = 2  Moving!" << std::endl;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_publisher");
	
	ros::NodeHandle nh, nh_private("~");
	
	PathPublisher publisher;
	publisher.init(nh, nh_private);
	
	ros::spin();
	
	return 0;
}
