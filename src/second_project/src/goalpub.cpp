#include <ros/ros.h>
// generic action libs
#include <actionlib/server/simple_action_server.h>
// generated from .action file
#include <second_project/SlamAction.h>


class SlamAction
{
private:
  ros::NodeHandle nh_;
  // FibonacciAction pointer to itself???
  actionlib::SimpleActionServer<second_project::SlamAction> as_; // action lib server
  std::string action_name_;
  // create messages that are used to published feedback/result
  // just custom messags at the end

  second_project::SlamActionFeedback feedback_;
  second_project::SlamActionResult result_;

public:
  // executeCB = actual callback, this is just a dispatcher of sort
  SlamAction(std::string name) :
    as_(nh_, name, boost::bind(&SlamAction::executeCB, this, _1), false), // autostart false
    action_name_(name)
  {
    as_.start(); // we start it manually
  }

  ~SlamAction(void)
  {
  }



  void executeCB(const second_project::SlamActionGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1); //simulate compute time
    bool success = true;

    // clear and set first two values
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);
    // feeback is just an array in .action


    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // start executing the action
    for(int i=1; i<=goal->order; i++)
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
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, we simulate compute time
      r.sleep();
    }

    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "goalserver");

  SlamAction points("gothere");
  ros::spin();

  return 0;
}
