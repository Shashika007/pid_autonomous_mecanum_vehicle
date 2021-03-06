#include <ros/ros.h>
#include <aiv_controller_pid/AIVAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>


float r = 0.077;
float lx = 0.188;
float ly = 0.135;
//class containing the client
class ControllerClient{

  public:

    ControllerClient(std::string name):

	    //Set up the client. It's publishing to topic "pid_control", and is set to auto-spin
	    ac("pid_control", true),

	    //Stores the name
	    action_name(name)
	    {
	      //Get connection to a server
	      ROS_INFO("%s Waiting For Server...", action_name.c_str());

	      //Wait for the connection to be valid
	      ac.waitForServer();

	      ROS_INFO("%s Got a Server...", action_name.c_str());

	      goalsub = n.subscribe("/cmd_vel", 100, &ControllerClient::GoalCallback, this);
      }

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state, const aiv_controller_pid::AIVResultConstPtr& result)
{
	ROS_INFO("Finished in state [%s]", state.toString().c_str());

	ROS_INFO("Result: %i", result->ok);
};

// Called once when the goal becomes active
void activeCb()
{
	ROS_INFO("Goal just went active...");
};

// Called every time feedback is received for the goal
void feedbackCb(const aiv_controller_pid::AIVFeedbackConstPtr& feedback)
{
	ROS_INFO("Got Feedback of Progress to Goal: position: %f", feedback->right_wheel1_vel);
};

void GoalCallback(const geometry_msgs::Twist& twist)
{
	float gain = 1/r;
	goal.right_wheel1_vel =gain*(twist.linear.x - twist.linear.y + twist.angular.z*(lx+ly));
	goal.left_wheel2_vel =gain*(twist.linear.x + twist.linear.y - twist.angular.z*(lx+ly));
	goal.left_wheel3_vel =gain*(twist.linear.x - twist.linear.y - twist.angular.z*(lx+ly));
	goal.right_wheel4_vel = gain*(twist.linear.x + twist.linear.y + twist.angular.z*(lx+ly));
	ac.sendGoal(goal, boost::bind(&ControllerClient::doneCb, this, _1, _2),
		 boost::bind(&ControllerClient::activeCb, this),
		 boost::bind(&ControllerClient::feedbackCb, this, _1));
};

private:
	actionlib::SimpleActionClient<aiv_controller_pid::AIVAction> ac;
	std::string action_name;
	aiv_controller_pid::AIVGoal goal;
	ros::Subscriber goalsub;
	ros::NodeHandle n;
};

int main (int argc, char **argv)
{
	ros::init(argc, argv, "pid_client");

	// create the action client
	// true causes the client to spin its own thread
	ControllerClient client(ros::this_node::getName());

	ros::spin();

	//exit
	return 0;
}
