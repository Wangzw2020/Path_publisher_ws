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
	nh_private_.param<std::string>("path_topic", path_topic_, " ");
	nh_private_.param<std::string>("task_list_file", task_list_file_, " ");
	nh_private_.param<int>("num_task", num_task_, 0);
	nh_private_.param<int>("current_task", current_task_, 0);
	nh_private_.param<int>("passenger_position", passenger_position_, 0);
	nh_private_.param<int>("move_pedestrian_position", move_pedestrian_position_, 0);
	nh_private_.param<int>("light_state", light_state_, 0);
	nh_private_.param<int>("destination", destination_, 0);

	std::cout << "num of task is " << num_task_ << '\t' << "current task is "<< current_task_ << std::endl;
	num_task_ = num_task_-1;
	
	if (!loadAllPathFile())
	{
		ROS_ERROR("[%s] Task file load failed!", __NAME__);
	}
	path_file_name_pub_ = nh_.advertise<path_msgs::path_msg>(path_topic_, 1);
	
	path_state_sub_ = nh_.subscribe(state_topic_, 1, &PathPublisher::path_state_callback, this);
	
	std::cout << "now state:  " << '\n' 
	<< "passenger_position:  " << passenger_position_ << '\n'
	<< "move_pedestrian_position:  " << move_pedestrian_position_ << '\n' 
	<< "light_state:  " << light_state_ << '\n' 
	<< "destination:  " << destination_ << std::endl; 
	
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
		std::cout << "state = 0  Task" << current_task_ << "is done!" << std::endl;
		
		if (current_task_ == num_task_)
		{
			ROS_INFO("[%s] All task is done! ", __NAME__);
		}
		
		if (!pubNextPath())
		{
			ROS_ERROR("[%s] compute next path failed!", __NAME__);
			return;
		}
		
		std::cout << "New task is sent!" << std::endl;
	}
	else if(state == 1)
	{
		std::cout << "state = 1  Task starts!" << std::endl;
	}
	else if(state == 2)
	{
		//std::cout << "state = 2  Moving!" << std::endl;
	}
}

void PathPublisher::obu_callback(const obu_msgs::OBU_fusion::ConstPtr& container)
{
	
	
	
}

bool PathPublisher::loadAllPathFile()
{
	std::ifstream path_file;
	path_file.open(task_list_file_.c_str());
	std::string line;
	if (!path_file)
	{
		ROS_ERROR("[%s] open task file failed!", __NAME__);
		return false;
	}
	while(getline(path_file,line))
	{
		all_path_.push_back(line);
	}
	path_file.close();

	std::cout << all_path_.size() << " path loaded!" << std::endl;
	for (size_t i=0; i<all_path_.size(); ++i)
	{
		std::cout << all_path_[i] << std::endl;
	}
	
	return true;
}

bool PathPublisher::pubNextPath()
{
	if (current_task_ == 0)
	{
		while(1)
		{
			if (light_state_ == 1)
				std::cout << "green light!" << std::endl;
			else
				std::cout << "red light!" << std::endl;
			
			if (light_state_ == 1)
				break;
			
			std::cout << "waiting green light signal!" << std::endl;
		}
		
		path_msgs::path_msg next_path;
		
		next_path.task = next_path.DRIVE_TASK;
		next_path.type = next_path.FILE_TYPE;
		next_path.path_resolution = 0.1;
		next_path.expect_speed = 15;
		next_path.path_filp = false;
		
		if (passenger_position_ == 1)
		{
			next_path.roadnet_file = all_path_[1];
		}
		else if (passenger_position_ == 2)
		{
			next_path.roadnet_file = all_path_[2];
		}
		else
		{
			ROS_ERROR("[%s] wrong passenger position!", __NAME__);
			return false;
		}
		
		path_file_name_pub_.publish(next_path);
	}
	else if (current_task_ == 1)
	{
		path_msgs::path_msg next_path;
		next_path.task = next_path.DRIVE_TASK;
		next_path.type = next_path.FILE_TYPE;
		next_path.path_resolution = 0.1;
		next_path.expect_speed = 15;
		
		next_path.roadnet_file = all_path_[4];
		next_path.path_filp = false;
	
		path_file_name_pub_.publish(next_path);
	}
	else if (current_task_ == 2)
	{
		path_msgs::path_msg next_path;
		next_path.task = next_path.DRIVE_TASK;
		next_path.type = next_path.FILE_TYPE;
		next_path.path_resolution = 0.1;
		next_path.expect_speed = 15;
		
		next_path.roadnet_file = all_path_[5];
		next_path.path_filp = false;
	
		path_file_name_pub_.publish(next_path);
	}
	else if (current_task_ == 3)
	{
		path_msgs::path_msg next_path;
		next_path.task = next_path.DRIVE_TASK;
		next_path.type = next_path.FILE_TYPE;
		next_path.path_resolution = 0.1;
		next_path.expect_speed = 15;
		next_path.path_filp = false;
		if (destination_ == 1)
			next_path.roadnet_file = all_path_[5];
		else if (destination_ == 2)
			next_path.roadnet_file = all_path_[6];
		else
		{
			ROS_ERROR("[%s] wrong destination position!", __NAME__);
			return false;
		}

		path_file_name_pub_.publish(next_path);
	}
	
//	path_msgs::path_msg next_path;
//	
//	next_path.task = next_path.DRIVE_TASK;
//	next_path.type = next_path.FILE_TYPE;
//	next_path.path_resolution = 0.1;
//	next_path.expect_speed = 15;
//	next_path.roadnet_file = "a.txt";
//	next_path.path_filp = false;
//	
//	path_file_name_pub_.publish(next_path);
	
	++current_task_;
	
	return true;
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
