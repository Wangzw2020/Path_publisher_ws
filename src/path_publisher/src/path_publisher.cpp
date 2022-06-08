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
	nh_private_.param<std::string>("obu_topic", obu_topic_, " ");
	nh_private_.param<std::string>("task_list_file", task_list_file_, " ");
	nh_private_.param<int>("num_task", num_task_, 0);
	nh_private_.param<int>("current_task", current_task_, 0);
	nh_private_.param<int>("passenger_position", passenger_position_, -1);
	nh_private_.param<int>("move_pedestrian_position", move_pedestrian_position_, -1);
	nh_private_.param<int>("delivery_end_point", delivery_end_point_, -1);
	nh_private_.param<int>("light_state", light_state_, -1);
	nh_private_.param<int>("destination", destination_, -1);

	std::cout << "num of task is " << num_task_ << '\t' << "current task is "<< current_task_ << std::endl;
	
	if (!loadAllPathFile())
	{
		ROS_ERROR("[%s] Task file load failed!", __NAME__);
	}
	
	path_file_name_pub_ = nh_.advertise<path_msgs::path_msg>(path_topic_, 1);
	
	path_state_sub_ = nh_.subscribe(state_topic_, 1, &PathPublisher::path_state_callback, this);
	obu_sub_ = nh_.subscribe(obu_topic_, 1, &PathPublisher::obu_callback, this);
	
	std::cout << "now state:  " << '\n' 
	<< "passenger_position:  " << passenger_position_ << '\n'
	<< "move_pedestrian_position:  " << move_pedestrian_position_ << '\n' 
	<< "light_state:  " << light_state_ << '\n' 
	<< "delivery end point: " << delivery_end_point_ << '\n'
	<< "destination:  " << destination_ << std::endl; 
	
	is_ready_ = true;
	ROS_INFO("[%s] Path publisher is initialized!", __NAME__);
	
	obu_ok_ = false;
	
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
			return;
		}
		
		if (!pubNextPath())
		{
			ROS_ERROR("[%s] compute next path failed!", __NAME__);
			return;
		}
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
    if (passenger_position_ != container->passenger_position)
    {
        passenger_position_ = container->passenger_position;
        std::cout << "passenger position updated: " << passenger_position_ << std::endl;
    }
	
	if (light_state_ != container->light_state)
    {
        light_state_ = container->light_state;
        std::cout << "light state updated: " << light_state_ << std::endl;
        if (!pubNextPath())
		{
			ROS_ERROR("[%s] no path published!", __NAME__);
		}
        obu_ok_ = true;
    }
    
    if (delivery_end_point_ != container->delivery_end_point)
    {
        delivery_end_point_ = container->delivery_end_point;
        std::cout << "delivery end point updated: " << delivery_end_point_ << std::endl;
    }
    
    if (destination_ != container->terminal_point)
    {
        destination_ = container->terminal_point;
        std::cout << "terminal point updated: " << destination_ << std::endl;
    }
	
//	std::cout << "now state:  " << '\n' 
//	<< "passenger_position:  " << passenger_position_ << '\n'
//	<< "move_pedestrian_position:  " << move_pedestrian_position_ << '\n' 
//	<< "light_state:  " << light_state_ << '\n' 
//	<< "delivery end point: " << delivery_end_point_ << '\n'
//	<< "destination:  " << destination_ << std::endl; 
	
	if (!obu_ok_)
    {
        ros::Duration(2).sleep();
        if (!pubNextPath())
		{
			ROS_ERROR("[%s] no path published!", __NAME__);
			return;
		}
        obu_ok_ = true;
	}
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
        path_msgs::path_msg next_path;
		
		next_path.task = next_path.DRIVE_TASK;
		next_path.type = next_path.FILE_TYPE;
		next_path.path_resolution = 0.1;
		next_path.expect_speed = 20;
		next_path.path_filp = false;
		next_path.roadnet_file = all_path_[0];
		path_file_name_pub_.publish(next_path);
		
		std::cout << "first path published!" << std::endl;
		++current_task_;
		std::cout << "New task is sent!" << std::endl;
    }
	else if (current_task_ == 1 && light_state_ == 1)
	{
		
		path_msgs::path_msg next_path;
		
		next_path.task = next_path.DRIVE_TASK;
		next_path.type = next_path.FILE_TYPE;
		next_path.path_resolution = 0.1;
		next_path.expect_speed = 25;
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
		++current_task_;
		std::cout << "current task: " << current_task_ << std::endl;
		std::cout << "New task is sent!" << std::endl;
	}
	else if (current_task_ == 2)
	{
		path_msgs::path_msg next_path;
		next_path.task = next_path.DRIVE_TASK;
		next_path.type = next_path.FILE_TYPE;
		next_path.path_resolution = 0.1;
		next_path.expect_speed = 10;
		next_path.path_filp = false;
		
		if (delivery_end_point_ == 1)
		    next_path.roadnet_file = all_path_[3];
		else if (delivery_end_point_ == 2)
		    next_path.roadnet_file = all_path_[4];
		else if (delivery_end_point_ == 3)
		    next_path.roadnet_file = all_path_[5];
		else if (delivery_end_point_ == 4)
		    next_path.roadnet_file = all_path_[6];
	
		path_file_name_pub_.publish(next_path);
		++current_task_;
		std::cout << "current task: " << current_task_ << std::endl;
		std::cout << "New task is sent!" << std::endl;
	}
	else if (current_task_ == 3)
	{
		ros::Duration(5).sleep();
		path_msgs::path_msg next_path;
		next_path.task = next_path.DRIVE_TASK;
		next_path.type = next_path.FILE_TYPE;
		next_path.path_resolution = 0.1;
		next_path.expect_speed = 20;
		next_path.path_filp = false;
		if (destination_ == 1)
			next_path.roadnet_file = all_path_[7];
		else if (destination_ == 2)
			next_path.roadnet_file = all_path_[8];
		else
		{
			ROS_ERROR("[%s] wrong destination position!", __NAME__);
			return false;
		}

		path_file_name_pub_.publish(next_path);
		++current_task_;
		std::cout << "current task: " << current_task_ << std::endl;
		std::cout << "New task is sent!" << std::endl;
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
