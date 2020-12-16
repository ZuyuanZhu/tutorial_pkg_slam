#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

//1.Use correct indentation for every line to make your code more readable.
//2.Comment the code 

tf::Transform transform;//Transformation message
tf::Quaternion q;//Quaternion for storing rotation data

class Stopper {
public:
	//Tunable parameters, you can define yours as you need. You may want to use an individual head file for all variables and definitions.
	constexpr const static double FORWARD_SPEED_LOW = 0.1;
	constexpr const static double FORWARD_SPEED_MIDDLE = 0.2;
	constexpr const static double FORWARD_SPEED_HIGH = 0.4;
	constexpr const static double FORWARD_SPEED_SHIGH = 0.6;
	constexpr const static double SPEED_STOP = 0;
	constexpr const static double TURN_LEFT_SPEED_HIGH = 1.0;
	constexpr const static double TURN_LEFT_SPEED_LOW = 0.3;
	constexpr const static double TURN_RIGHT_SPEED_HIGH = -2.4;  
	constexpr const static double TURN_RIGHT_SPEED_LOW = -0.6;
	constexpr const static double TURN_RIGHT_SPEED_MIDDLE = -1.2;
	Stopper();	
	//functions for controlling the robot.
	void startMoving();
	void moveForward(double forwardSpeed);
	void moveStop();   
	void moveRight(double turn_right_speed = TURN_RIGHT_SPEED_HIGH);
	void pose_callback(const geometry_msgs::PoseStampedPtr &pose);
private:
    //%%% Put all the publisher and subscriber declarations here:
	ros::NodeHandle	node;  //create a ROS node
	ros::Publisher commandPub;	//create a Publisher that publishes to the robot's velocity command topic
	ros::Subscriber pose_sub; //create a Subscriber that subscribes to the pose
};

Stopper::Stopper(){
	//%%% Put all the subscriber and publisher definitions here:
	commandPub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10); //Advertise a new publisher for the simulated robot's velocity command topic at 10Hz	
	pose_sub = node.subscribe("/pose", 10, &Stopper::pose_callback, this);
}

void Stopper::pose_callback(const geometry_msgs::PoseStampedPtr &pose)
{
   static tf::TransformBroadcaster br;
   q.setX(pose->pose.orientation.x);
   q.setY(pose->pose.orientation.y);
   q.setZ(pose->pose.orientation.z);
   q.setW(pose->pose.orientation.w);

   transform.setOrigin(tf::Vector3(pose->pose.position.x, pose->pose.position.y, 0.0));
   transform.setRotation(q);

   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
}


//send a velocity command
void Stopper::moveForward(double forwardSpeed){
	geometry_msgs::Twist msg; // The default constructor will set all commands to 0
	msg.linear.x = forwardSpeed; //Drive forward at a given speed along x-axis. The robot points up the x-axis.
	commandPub.publish(msg);  // publish the velocity command to the "cmd_vel" topic
}

void Stopper::moveStop(){
	geometry_msgs::Twist msg; 
	msg.linear.x = SPEED_STOP;
	msg.angular.z = SPEED_STOP;
	commandPub.publish(msg);
}

void Stopper::moveRight(double turn_right_speed){
	geometry_msgs::Twist msg; 
	msg.angular.z = turn_right_speed;
	commandPub.publish(msg);
}

void Stopper::startMoving(){
	ros::Rate rate(20);   //Define rate for repeatable operations, frequency = N/4, i.e., 5. 
	ROS_INFO("Start moving");
	// keep spinning loop until user presses Ctrl+C
	while (ros::ok()){ //Check if ROS is working. E.g. if ROS master is stopped or there was sent signal to stop the system, ros::ok() will return false.
		moveForward(FORWARD_SPEED_LOW);//Do not keep sending moveForward command to the robot when your robot tries to turn right or left. Or, your robot may ignore the turning command.
		//ROS_INFO_STREAM("Robot speed: " << FORWARD_SPEED_LOW);		
		ros::spinOnce(); //Allow ROS to process incoming messages, 5Hz for simulator; 10 Hz for real robot.
		rate.sleep(); //rate.sleep() in the while loop controls the update frequency of the while loop. Here the frequency is N/4=20/4=5. This frequency will overwrite the subscriber's and publisher's frequency.  
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "stopper");   // Initiate new ROS node named "stopper"
	Stopper stopper; 	// Create new stopper object  
	stopper.startMoving();     // Start the movement
	return 0;
}
