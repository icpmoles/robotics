#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseFeedback.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>


using namespace std;


struct simplePose {
    double X, Y, theta;
};

vector<simplePose> getList(std::string path){
    
  vector<simplePose> list;
  
  ROS_INFO("GOALPUB: from launch file expecting a CSV in: %s",path.c_str());

  string line;
  ifstream csvfile;
  csvfile.open (path); 

  if (csvfile.is_open())
  {
    ROS_INFO("GOALPUB: CSV opened ");
    while ( getline (csvfile,line) ) // reading  line
    {
     ROS_INFO("GOALPUB: line: %s ", line.c_str());
     
      stringstream linestream(line); 
       simplePose e;
      string xvalue;
      string yvalue;
      string tvalue;
      getline (linestream,xvalue,',');
      getline (linestream,yvalue,',');
      getline (linestream,tvalue,',');
      e.X = std::stof (xvalue);
      e.Y = std::stof (yvalue);
      e.theta = std::stof (tvalue);
      list.push_back(e);
    }
    csvfile.close();
  }

  else  {ROS_INFO("GOALPUB: unable to open file %i %i%i%i%i ",csvfile.rdstate(),csvfile.good(),csvfile.eof(),csvfile.fail(),csvfile.bad());
  
  
  }

  ROS_INFO("GOALPUB: CSV fully parsed %i elements", static_cast<int>(list.size()));

  csvfile.close();
  return list;
  
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// standard notation placeholders for donecb when being called as callback
void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO("GOALPUB: Finished in state [%s]", state.toString().c_str());
}

void activeCb() {
    ROS_INFO("GOALPUB: Goal just went active");
}


void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& result) {
    ROS_INFO("GOALPUB: Got Feedback of length ");
}


int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh_private("~");

  
	std::string wp_path;
	nh_private.getParam("waypoint_path", wp_path); //get l
  std::vector<simplePose> goal_list = getList(wp_path);
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  
  // simplePose goal_list[]  =
  // {{ 2.0 , 0.0 , 0.1},
  // { -1.0 , 0.0 , -3.14}
  // } ;
  
  // int goal_size = sizeof(goal_list)/sizeof(simplePose);


  int goal_size = size(goal_list);
  move_base_msgs::MoveBaseGoal goal;
  
  goal.target_pose.header.frame_id = "map";
  //wait for the ros server to come up
  int i = 0;
  while (ros::ok() && i<goal_size){
   //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("GOALPUB: Waiting for the move_base action server to come up");
  }


      //we'll send a goal to the robot to move 1 meter forward
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = goal_list[i].X;
      goal.target_pose.pose.position.y = goal_list[i].Y;

      tf2::Quaternion myQuaternion;
      myQuaternion.setRPY( 0, 0, goal_list[i].theta );
      myQuaternion.normalize();
      goal.target_pose.pose.orientation.x = myQuaternion.getX();
      goal.target_pose.pose.orientation.y = myQuaternion.getY();
      goal.target_pose.pose.orientation.z = myQuaternion.getZ();
      goal.target_pose.pose.orientation.w = myQuaternion.getW();

      ROS_INFO("GOALPUB: Sending goal %i / %i ",i+1,goal_size);
      ac.sendGoal(goal,&doneCb , &activeCb);//, &feedbackCb);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        i++;
        ROS_INFO("GOALPUB: Hooray, the base reached the goal");
      }
      else if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
        ROS_INFO("GOALPUB: The goal was aborted, moving to the next goal");
        i++;
      }
      else
        ROS_INFO("GOALPUB: something else happened to the goal, retrying(?)");

}

ROS_INFO("GOALPUB: We reached all the %i goals sucessfully",goal_size);
return 0;
}
