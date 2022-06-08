#include <path_planning/global_path_planning.h>
#define __NAME__ "global_path_planning"

GlobalPathPlanning::GlobalPathPlanning()
{
}

bool GlobalPathPlanning::init(ros::NodeHandle nh, ros::NodeHandle nh_private)
{
	nh_ = nh;
	nh_private_ = nh_private;
	
	nh_private_.param<int>("task", task_, 0); // 0 - 前进任务，1 - 后退任务
	nh_private_.param<int>("type", type_, 3); // 1 - POSE_TYPE，2 - PATH_TYPE， 3 - FILE_TYPE
	
	nh_private_.param<float>("path_resolution", path_resolution_, 0.1);
	nh_private_.param<float>("expect_speed", expect_speed_, 15); // km/h
	nh_private_.param<std::string>("roadnet_file", roadnet_file_, " "); // 绝对路径，以.txt结尾
	nh_private_.param<bool>("path_filp", path_filp_, false);
	
	// publish path state
	nh_private_.param<std::string>("state_topic", state_topic_, " ");
	path_tracking_state_pub_ = nh_.advertise<std_msgs::Int8>(state_topic_, 10);
	path_tracking_state_.data = -1;
	
	// subscribe this topic to know next path
	nh_private_.param<std::string>("path_name_topic", path_name_topic_, " ");
	path_file_sub_ = nh_.subscribe<path_msgs::path_msg>(path_name_topic_, 1, &GlobalPathPlanning::path_name_callback, this);
	
	ac_ = new DoDriverlessTaskClient("/do_driverless_task", true); // true -> don't need ros::spin()
	
	ROS_INFO("[%s] Wait for server.", __NAME__);
	ac_->waitForServer();
	ROS_INFO("[%s] Wait for server ok.", __NAME__);

	is_ready_ = true;
	
	return true;
}

bool GlobalPathPlanning::start()
{
    if(!is_ready_)
	{
		ROS_ERROR("[%s] System is not ready!", __NAME__);
		return false;
	}
	
	ROS_INFO("[%s] wait for path msg!", __NAME__);
//	driverless_actions::DoDriverlessTaskGoal goal;
//	
//	goal.task = task_;
//	goal.type = type_;
//	
//	goal.path_resolution = path_resolution_;
//	goal.expect_speed = expect_speed_;
//	goal.roadnet_file = roadnet_file_;
//	goal.path_filp = path_filp_;
//	
//    ac_->sendGoal(goal, boost::bind(&GlobalPathPlanning::taskDoneCallback,this,_1,_2),
//                        boost::bind(&GlobalPathPlanning::taskActivedCallback,this),
//                        boost::bind(&GlobalPathPlanning::taskFeedbackCallback,this,_1));
    
}

void GlobalPathPlanning::taskDoneCallback(const actionlib::SimpleClientGoalState& state,
                                          const driverless_actions::DoDriverlessTaskResultConstPtr& result)
{
    ROS_INFO("[%s] Task is done.", __NAME__);
    path_tracking_state_.data = 0;
    path_tracking_state_pub_.publish(path_tracking_state_);
}

void GlobalPathPlanning::taskFeedbackCallback(const driverless_actions::DoDriverlessTaskFeedbackConstPtr& feedback)
{
    ROS_INFO("[%s] Feedback.", __NAME__);
    path_tracking_state_.data = 2;
    path_tracking_state_pub_.publish(path_tracking_state_);
}

void GlobalPathPlanning::taskActivedCallback()
{
    ROS_INFO("[%s] Task is actived.", __NAME__);
    path_tracking_state_.data = 1;
    path_tracking_state_pub_.publish(path_tracking_state_);
}
    
void GlobalPathPlanning::path_name_callback(const path_msgs::path_msg::ConstPtr& path_msg)
{
	ROS_INFO("[%s] New path !", __NAME__);
	
	driverless_actions::DoDriverlessTaskGoal goal;
	
	goal.task = path_msg->task;
	goal.type = path_msg->type;
	
	goal.path_resolution = path_msg->path_resolution;
	goal.expect_speed = path_msg->expect_speed;
	goal.roadnet_file = path_msg->roadnet_file;
	goal.path_filp = path_msg->path_filp ;
	
    ac_->sendGoal(goal, boost::bind(&GlobalPathPlanning::taskDoneCallback,this,_1,_2),
                        boost::bind(&GlobalPathPlanning::taskActivedCallback,this),
                        boost::bind(&GlobalPathPlanning::taskFeedbackCallback,this,_1));
    
}
    
int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_path_planning");
    ros::AsyncSpinner spinner(2);
	spinner.start(); //非阻塞
	
	ros::NodeHandle nh, nh_private("~");
	
	GlobalPathPlanning planner;
	if(planner.init(nh, nh_private))
	{
	    planner.start();
	    ros::waitForShutdown();
	}
	
	return 0;
}
