#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "generate_traj/TrajAction.h"
#include "plan_and_run/demo_application.h"

class TrajAction
{
protected:

  ros::NodeHandle nh_; 
  // NodeHandle instance must be created before this line. 
  // Otherwise strange error occurs.
  actionlib::SimpleActionServer<generate_traj::TrajAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  generate_traj::TrajFeedback feedback_;
  generate_traj::TrajResult result_;
  // creating application from plan_and_run
  plan_and_run::DemoApplication application;
  plan_and_run::DescartesTrajectory traj;
  plan_and_run::DescartesTrajectory output_path;

public:

  TrajAction(std::string name) :
    as_(nh_, name, boost::bind(&TrajAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~TrajAction(void)
  {
  }

  // Define the callback function
  void executeCB(const generate_traj::TrajGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // Pre-set the feedback(add 0 to the feedback sequence here)
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);

    // Publish info to the console for the user
    // Test the goal
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %f with seeds %f, %f", action_name_.c_str(), goal->x[0], goal->y[0], goal->y[1]);

    // Start executing the action
    for(int i=1; i<=10; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      
      ROS_INFO("Working on it ......");

      // Upate the feedback msg
      feedback_.sequence.push_back(i);
      // publish the feedback
      as_.publishFeedback(feedback_);

      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }
    
    // Try to figure out a way 
    // avoid initialize every time in the callback function,
    // but for now, we can keep this for test the generate function
    ROS_INFO("Loading and initializing now!");
    // loading parameters
    application.loadParameters();
    // initializing ros components
    application.initRos();
    // initializing descartes
    application.initDescartes();

    ROS_INFO("All is well! Everyone is happy! You can start planning now!");
    // moving to home position
    application.moveHome();
    // generating trajectory
    application.generateTrajectory(traj);
    // planning robot path
    application.planPath(traj,output_path);
    // running robot path
    application.runPath(output_path);
    ros::Duration(10).sleep();
    

    if(success)
    {
      result_.result = 1;
      ROS_INFO("%s: Succeeded. Everyone is happy!", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }
};


int main(int argc, char** argv)
{
  // Setup the server
  ros::init(argc, argv, "traj_action_server");
  TrajAction traj_action("traj_action");
  // Info
  ROS_INFO("Waiting for the client ......");
  ros::spin();
  return 0;
}