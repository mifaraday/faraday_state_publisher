#include "faraday_state_publisher/faraday_state_publisher.h"

static std::vector<std::string> getDefaultFaradayJointNames()
{
    std::vector<std::string> names;
    names.reserve(5);
    names.push_back("joint_1");
    names.push_back("joint_2");
    names.push_back("joint_3");
    names.push_back("joint_4");
    names.push_back("joint_5");

    return names;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"faraday_state_publisher");
    ros::NodeHandle nh;

    std::vector<std::string> joint_names;
    joint_names.reserve(5);
    if(!nh.getParam("controller_joint_names",joint_names))
    {
        joint_names=getDefaultFaradayJointNames();
        ROS_INFO("Faraday State Publisher: loading default joint names [joint_2 ... joint_5]");
    }
    else
    {
        ROS_INFO("Faraday State Publisher: loaded joint names from param server");
    }

    // std::cout<<"I am here1"<<std::endl;
    // std::cout<<"joint_names size is: "<<joint_names.size()<<std::endl;
    
    // faraday_state_publisher::FaradayStatePublisher pub;
    faraday_state_publisher::FaradayStatePublisher pub("joint_states",joint_names);
    // std::cout<<"I am here2"<<std::endl;
    ros::spin();

    return 0;
}
