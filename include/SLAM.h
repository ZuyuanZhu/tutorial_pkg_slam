#include <ros/ros.h>
#include <geometry_msgs/PostStamped.h>
#include <tf/transform_broadcaster.h>

//Transformation message:
tf::Transform transform;
//Quaternion for storing rotation data:
tf::Quaternion q;
//Function for handling incoming /PostStamped messages, extract quaternion parameters from message and put it to quaternion structure, put translation parameters into transform message, put quaternion structure into transform message, publish transform:
void pose_callback(const geometry_msgs::PoseStampedPtr &pose){
	static tf::TransformBroadcaster br; //Publisher for transformation:
	q.setX(pose->pose.orientation.x);
	q.setY(pose->pose.orientation.y);
	q.setZ(pose->pose.orientation.z);
	q.setW(pose->pose.orientation.w);
	
	transform.setOrigin(tf::Vector3(pose->pose.position.x, pose->pose.position.y, 0.0));
	transform.setRotation(q);
	
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
}

int main(int argc, char **argv){
	ros::init(argc, argv, "drive_controller");
	ros::NodeHandle n("~");
	ros::Subscriber pose_pub=n.subscriber("/pose", 1, pose_callback);
	ros::Rate loop_rate(100);
	while (ros::ok()){
		ros::spineOnce();
		loop_rate.sleep();
	}
}


