#ifndef FARADAY_STATE_PUBLISHER_H
#define FARADAY_STATE_PUBLISHER_H

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace faraday_state_publisher
{
class FaradayStatePublisher
{
public:
	FaradayStatePublisher(const std::string& joints_topic,
						  const std::vector<std::string>& joint_names);
	FaradayStatePublisher() {printf("hahahaha\n");}						  

	void updateJointPosition(const sensor_msgs::JointStateConstPtr& joints);

protected:
	bool extractJoints(const sensor_msgs::JointState& msg,double* actuators) const;

private:
	ros::NodeHandle nh_;
	tf::TransformBroadcaster tf_broadcaster_;
	ros::Subscriber joint_sub_;
	std::vector<std::string> joint_names_;

	// mm radian
	double last_pose[5]={0.0, 0.0, 390.0, 0.0, 0.0};
};

//limb1-4 link1 initial pose in the base_link
extern const tf::Vector3 limb1_link1_position;
extern const tf::Vector3 limb1_link1_orientation;

extern const tf::Vector3 limb2_link1_position;
extern const tf::Vector3 limb2_link1_orientation;

extern const tf::Vector3 limb3_link1_position;
extern const tf::Vector3 limb3_link1_orientation;

extern const tf::Vector3 limb4_link1_position;
extern const tf::Vector3 limb4_link1_orientation;

//limb5 link1-4 initial pose in the base_link
// extern const tf::Vector3 limb5_link1_orientation;
extern const tf::Matrix3x3 limb5_link1_rotation;
extern const tf::Vector3 limb5_link1_position;

// extern const tf::Vector3 limb5_link2_orientation;
extern const tf::Matrix3x3 limb5_link2_rotation;
extern const tf::Vector3 limb5_link2_position;

// extern const tf::Vector3 limb5_link3_orientation;
extern const tf::Matrix3x3 limb5_link3_rotation;

extern const double limb5_link1_y;
extern const double limb5_link2_y;
extern const double limb5_link3_y;
extern const double limb5_link3_z;


} //end namespace faraday_state_publisher


#endif //end FARADAY_STATE_PUBLISHER_H