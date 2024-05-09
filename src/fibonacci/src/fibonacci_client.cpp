#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");

  // create the action client, true/false for spin() automatically called
  actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("fibonacci", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time, n.b: blocking

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  actionlib_tutorials::FibonacciGoal goal;
  int order = 10;
  double duration = 1.0; // how long for the timeout
  ros::param::get("order",order);
  ros::param::get("duration",duration);
  goal.order = order;
  ac.sendGoal(goal);

  //wait for the action to return, like for a service. wait "duration" , otherwise raise timeout bool
  bool finished_before_timeout = ac.waitForResult(ros::Duration(duration));


  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    // retrieve result from action server
    actionlib_tutorials::FibonacciResultConstPtr  result = ac.getResult();
    for (int i=0; i<result->sequence.size();i++){
       ROS_INFO ("%d ", result->sequence[i]);
    }

  }
  else{
  	ROS_INFO("Action did not finish before the time out.");
	  ac.cancelGoal (); // waitForResult doesn't cancel the goal when it timeouts, it leaves the server running and doesn't care
  }
    
  //exit
  return 0;
}
