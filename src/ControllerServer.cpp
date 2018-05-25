#include <ros/ros.h>
#include <aiv_controller_pid/AIVAction.h>
#include <actionlib/server/simple_action_server.h>

#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/JointState.h"
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <aiv_controller_pid/controllerServerConfig.h>


#define PI 3.14159265
// pid values for right wheel1
float r1_kp;
float r1_ki;
float r1_kd;
//pid values for left wheel2
float l2_kp;
float l2_ki;
float l2_kd;
//pid values for left wheel3
float l3_kp;
float l3_ki;
float l3_kd;
//pid values for right wheel4
float r4_kp;
float r4_ki;
float r4_kd;
//pid values for odometry x
float x_kp;
float x_ki;
float x_kd;
//pid values for odometry y
float y_kp;
float y_ki;
float y_kd;
// pid values for odometry th
float th_kp;
float th_ki;
float th_kd;
//Class for containing the server
class ControllerServer{
public:

	ControllerServer(std::string name):
	as(n, "pid_control", boost::bind(&ControllerServer::executeCB, this, _1), false),
	action_name(name)
	{
		as.registerPreemptCallback(boost::bind(&ControllerServer::preemptCB, this));

		//Start the server
		as.start();

		//Subscriber current velocity of motors
	//	positionservosub = n2.subscribe("/right_wheel1_velocity", 1, &ControllerServer::SensorCallBackR1, this);
		right_wheel1_enc_sub = n2.subscribe("/right_wheel1_velocity", 1, &ControllerServer::SensorCallBackR1, this);
		left_wheel2_enc_sub = n2.subscribe("/left_wheel2_velocity", 1, &ControllerServer::SensorCallBackL2, this);
		left_wheel3_enc_sub = n2.subscribe("/left_wheel3_velocity", 1, &ControllerServer::SensorCallBackL3, this);
		right_wheel4_enc_sub = n2.subscribe("/right_wheel4_velocity", 1, &ControllerServer::SensorCallBackR4, this);

		//Publisher setpoint, current velocity and error of control
		error_control_R1pub = n2.advertise<geometry_msgs::Vector3>("/control/error/rightwheel1", 1);
		error_control_L2pub = n2.advertise<geometry_msgs::Vector3>("/control/error/leftwheel2", 1);
		error_control_L3pub = n2.advertise<geometry_msgs::Vector3>("/control/error/leftwheel3", 1);
		error_control_R4pub = n2.advertise<geometry_msgs::Vector3>("/control/error/rightwheel4", 1);

		//Publisher PID output in velocity
		velocityR1pub = n2.advertise<std_msgs::Float32>("/motor/right1vel", 1);
		velocityL2pub = n2.advertise<std_msgs::Float32>("/motor/left2vel", 1);
		velocityL3pub = n2.advertise<std_msgs::Float32>("/motor/left3vel", 1);
		velocityR4pub = n2.advertise<std_msgs::Float32>("/motor/right4vel", 1);



		//Max e Min Output PID Controller
		float max = PI;
		float min = -PI;

		//Initializing PID Controller
		Initialize(min,max);
  }


void preemptCB()
{
	ROS_INFO("%s got preempted!", action_name.c_str());
	result.ok = 0;
	as.setPreempted(result, "I got Preempted!");
}

//Callback for processing a goal
void executeCB(const aiv_controller_pid::AIVGoalConstPtr& goal)
{
  prevTime = ros::Time::now();

	//If the server has been killed, don't process
	if(!as.isActive()||as.isPreemptRequested()) return;

	//Run the processing at 100Hz
	ros::Rate rate(100);

	//Setup some local variables
	bool success = true;

	//Loop control
	while(1)
	{
		std_msgs::Float32 msg_vel1;
		std_msgs::Float32 msg_vel2;
		std_msgs::Float32 msg_vel3;
		std_msgs::Float32 msg_vel4;

		//PID Controller
		msg_vel1.data = PIDController(goal->right_wheel1_vel, velocity_encoderR1,r1_kp,r1_ki,r1_kd);
		msg_vel2.data = PIDController(goal->left_wheel2_vel, velocity_encoderL2,l2_kp,l2_kp,l2_kd);
		msg_vel3.data = PIDController(goal->left_wheel3_vel, velocity_encoderL3,l3_kp,l3_ki,l3_kd);
		msg_vel4.data = PIDController(goal->right_wheel4_vel, velocity_encoderR4,r4_kp,r4_ki,r4_kd);


		//Publishing PID output in velocity
		velocityR1pub.publish(msg_vel1);
		velocityL2pub.publish(msg_vel2);
		velocityL3pub.publish(msg_vel3);
		velocityR4pub.publish(msg_vel4);

		//Auxiliary Message
		geometry_msgs::Vector3 msg_errorR1;
		geometry_msgs::Vector3 msg_errorL2;
		geometry_msgs::Vector3 msg_errorL3;
		geometry_msgs::Vector3 msg_errorR4;

		msg_errorR1.x = goal->right_wheel1_vel;
		msg_errorR1.y = velocity_encoderR1;
		msg_errorR1.z = goal->right_wheel1_vel - velocity_encoderR1;

		msg_errorL2.x = goal->right_wheel1_vel;
		msg_errorL2.y = velocity_encoderR1;
		msg_errorL2.z = goal->right_wheel1_vel - velocity_encoderR1;

		msg_errorL3.x = goal->right_wheel1_vel;
		msg_errorL3.y = velocity_encoderR1;
		msg_errorL3.z = goal->right_wheel1_vel - velocity_encoderR1;

		msg_errorR4.x = goal->right_wheel1_vel;
		msg_errorR4.y = velocity_encoderR1;
		msg_errorR4.z = goal->right_wheel1_vel - velocity_encoderR1;

		//Publishing setpoint, feedback and error control
		error_control_R1pub.publish(msg_errorR1);
		error_control_L2pub.publish(msg_errorL2);
		error_control_L3pub.publish(msg_errorL3);
		error_control_R4pub.publish(msg_errorR4);

		feedback.right_wheel1_vel = velocity_encoderR1;
		feedback.left_wheel2_vel = velocity_encoderL2;
		feedback.left_wheel3_vel = velocity_encoderL3;
		feedback.right_wheel4_vel = velocity_encoderR4;

                //Publish feedback to action client
                as.publishFeedback(feedback);

		//Check for ROS kill
		if(!ros::ok())
		{
			success = false;
			ROS_INFO("%s Shutting Down", action_name.c_str());
			break;
		}

		//If the server has been killed/preempted, stop processing
		if(!as.isActive()||as.isPreemptRequested()) return;

		//Sleep for rate time
		rate.sleep();
	}

	//Publish the result if the goal wasn't preempted
	if(success)
	{
		result.ok = 1;
		as.setSucceeded(result);
	}
	else
	{
		result.ok = 0;
		as.setAborted(result,"I Failed!");
	}
}


void Initialize( float min, float max)
{
  setOutputLimits(min, max);
  lastError = 0;
  errSum = 0;
}

void setOutputLimits(float min, float max)
{
	if (min > max) return;

	minLimit = min;
	maxLimit = max;
}

float PIDController(float setpoint, float PV, float kp, float ki, float kd)
{
	ros::Time now = ros::Time::now();
	ros::Duration change = now - prevTime;

	float error = setpoint - PV;

	float dErr = error - lastError;

	errSum += error*change.toSec();
	errSum = std::min(errSum, maxLimit);
	errSum = std::max(errSum, minLimit);

  dErr = (error - lastError)/change.toSec();

	//Do the full calculation
	float output = (kp*error) + (ki*errSum) + (kd*dErr);

	//Clamp output to bounds
	output = std::min(output, maxLimit);
	output = std::max(output, minLimit);

	//Required values for next round
	lastError = error;

	return output;
}

void SensorCallBackR1(const std_msgs::Float32& msg)
{
	velocity_encoderR1 = msg.data;
}

void SensorCallBackL2(const std_msgs::Float32& msg)
{
	velocity_encoderL2 = msg.data;
}

void SensorCallBackL3(const std_msgs::Float32& msg)
{
	velocity_encoderL3 = msg.data;
}

void SensorCallBackR4(const std_msgs::Float32& msg)
{
	position_encoder = msg.data;
}

protected:
	ros::NodeHandle n;
	ros::NodeHandle n2;

	//Subscriber
	ros::Subscriber right_wheel1_enc_sub;
	ros::Subscriber left_wheel2_enc_sub;
	ros::Subscriber left_wheel3_enc_sub;
	ros::Subscriber right_wheel4_enc_sub;

	//Publishers
	ros::Publisher velocityR1pub;
	ros::Publisher velocityL2pub;
	ros::Publisher velocityL3pub;
	ros::Publisher velocityR4pub;

	ros::Publisher error_control_R1pub;
  ros::Publisher error_control_L2pub;
	ros::Publisher error_control_L3pub;
	ros::Publisher error_control_R4pub;




	//Actionlib variables
	actionlib::SimpleActionServer<aiv_controller_pid::AIVAction> as;
	aiv_controller_pid::AIVFeedback feedback;
	aiv_controller_pid::AIVResult result;
	std::string action_name;

	//Control variables
	float position_encoder;
	float velocity_encoderR1;
	float velocity_encoderL2;
	float velocity_encoderL3;
	float velocity_encoderR4;
	float errSum;
	float lastError;
	float minLimit, maxLimit;
	ros::Time prevTime;

};

void callback(aiv_controller_pid::controllerServerConfig &config,float level){
  ROS_INFO("New Values r1: [%f] = [%f] = [%f]", config.r1_kp, config.r1_ki, config.r1_kd);
	ROS_INFO("New Values r4: [%f] = [%f] = [%f]", config.r4_kp, config.r4_ki, config.r4_kd);
	// load right1 pid values
  r1_kp = config.r1_kp;
  r1_ki = config.r1_ki;
  r1_kd = config.r1_kd;
	// load left2 pid values
	l2_kp = config.l2_kp;
	l2_ki = config.l2_ki;
	l2_kd = config.l2_kd;
	// load left3 pid values
	l3_kp = config.l3_kp;
	l3_ki = config.l3_ki;
	l3_kd = config.l3_kd;
	// load right4 pid values
  r4_kp = config.r4_kp;
	r4_ki = config.r4_ki;
	r4_kd = config.r4_kd;
	// load odometry x pid Values
	x_kp = config.x_kp;
	x_ki = config.x_ki;
	x_kd = config.x_kd;
	// load odometry y pid Values
	y_kp = config.y_kp;
	y_ki = config.y_ki;
	y_kd = config.y_kd;
	// load odometry th pid Values
	th_kp = config.th_kp;
	th_ki = config.th_ki;
	th_kd = config.th_kd;


}



//Used by ROS to actually create the node. Could theoretically spawn more than one server
int main(int argc, char** argv)
{
	ros::init(argc, argv, "pid_server");

 	//Just a check to make sure the usage was correct
	if(argc != 1)
	{
		ROS_INFO("Usage: pid_server");
		return 1;
	}

	dynamic_reconfigure::Server<aiv_controller_pid::controllerServerConfig> server1;
        dynamic_reconfigure::Server<aiv_controller_pid::controllerServerConfig>::CallbackType f;
        f = boost::bind(&callback, _1, _2);
        server1.setCallback(f);
	//Spawn the server
	   ControllerServer server(ros::this_node::getName());

	ros::spin();

	return 0;
}
