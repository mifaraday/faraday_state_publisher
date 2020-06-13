#include "faraday_state_publisher/faraday_state_publisher.h"
#include "faraday_kinematics/faraday_kinematic_defs.h"
#include "faraday_kinematics/faraday_kinematics.h"
#include "tf/transform_listener.h"

namespace fsp=faraday_state_publisher;

static tf::Vector3 toTF(const Eigen::Vector3d& v)
{
    //mm to m
    return tf::Vector3(v(0)/1000.0,v(1)/1000.0,v(2)/1000.0);
}

//calculate limb1-4 link1-3 transform
static void calculateLink1to3Transform(const Eigen::Vector3d& start,
                                       const Eigen::Vector3d& stop,
                                       const tf::Vector3 position,
                                       const tf::Vector3 orientation,
                                       tf::Transform& link1_tf,
                                       tf::Transform& link2_tf,
                                       tf::Transform& link3_tf)
{
    //calculate lin1 transform
    tf::Vector3 link1_origin;

    link1_origin.setX(position[0]);
    link1_origin.setY(position[1]);
    link1_origin.setZ(start[2]/1000.0);

    link1_tf.setOrigin(link1_origin);

    tf::Matrix3x3 link1_rotation;

    link1_rotation.setEulerYPR(orientation[2],orientation[1],orientation[0]);  

    link1_tf.setBasis(link1_rotation);

    tf::Vector3 X1=link1_rotation.getColumn(0);
    tf::Vector3 Z2=X1;   
    tf::Vector3 Z3=toTF(stop)-toTF(start);
    Z3.normalize();
    tf::Vector3 X2=Z3.cross(Z2);
    X2.normalize();
    tf::Vector3 X3=X2;

    //calculate link2 transform
    tf::Vector3 link2_origin;

    link2_origin.setX(start(0)/1000.0);
    link2_origin.setY(start(1)/1000.0);
    link2_origin.setZ(start(2)/1000.0);

    link2_tf.setOrigin(link2_origin);

    tf::Vector3 Y2=Z2.cross(X2);
    
    /*std::cout<<"X2 is "<<X2[0]<<" "<<X2[1]<<" "<<X2[2]<<std::endl;
    std::cout<<"Y2 is "<<Y2[0]<<" "<<Y2[1]<<" "<<Y2[2]<<std::endl;
    std::cout<<"Z2 is "<<Z2[0]<<" "<<Z2[1]<<" "<<Z2[2]<<std::endl;*/

    tf::Matrix3x3 link2_rotation(
        X2.getX(), Y2.getX(), Z2.getX(),
        X2.getY(), Y2.getY(), Z2.getY(),
        X2.getZ(), Y2.getZ(), Z2.getZ());

    link2_tf.setBasis(link2_rotation);

    //calculate link3_transform
    tf::Vector3 link3_origin;
    link3_origin=link2_origin;

    link3_tf.setOrigin(link3_origin);

    tf::Vector3 Y3=Z3.cross(X3);

    tf::Matrix3x3 link3_rotation(
        X3.getX(), Y3.getX(), Z3.getX(),
        X3.getY(), Y3.getY(), Z3.getY(),
        X3.getZ(), Y3.getZ(), Z3.getZ());

    link3_tf.setBasis(link3_rotation);
}                                       


//calculate limb1-2 link4-5 transform  
static void calculateLimb1to2Link4to5Transform(
                                       const Eigen::Vector3d& start,
                                       const Eigen::Vector3d& stop,
                                       const tf::Vector3 m_x_axis,
                                       tf::Transform& link4_tf,
                                       tf::Transform& link5_tf)
{
    tf::Vector3 Z5=m_x_axis;
    tf::Vector3 Z4=toTF(stop)-toTF(start);
    Z4.normalize();
    tf::Vector3 X4=Z5.cross(Z4);
    X4.normalize();
    tf::Vector3 X5=-X4;

    //calculate limb1-2 link4 transform
    tf::Vector3 link4_origin;

    link4_origin.setX(stop(0)/1000.0);
    link4_origin.setY(stop(1)/1000.0);
    link4_origin.setZ(stop(2)/1000.0);

    link4_tf.setOrigin(link4_origin);

    tf::Vector3 Y4=Z4.cross(X4);

    tf::Matrix3x3 link4_rotation(
        X4.getX(), Y4.getX(), Z4.getX(),
        X4.getY(), Y4.getY(), Z4.getY(),
        X4.getZ(), Y4.getZ(), Z4.getZ());

    link4_tf.setBasis(link4_rotation);

    //calculate limb1-2 link5 transform
    tf::Vector3 link5_origin=link4_origin;

    link5_tf.setOrigin(link5_origin);

    tf::Vector3 Y5=Z5.cross(X5);

    tf::Matrix3x3 link5_rotation(
        X5.getX(), Y5.getX(), Z5.getX(),
        X5.getY(), Y5.getY(), Z5.getY(),
        X5.getZ(), Y5.getZ(), Z5.getZ());

    link5_tf.setBasis(link5_rotation);
}

//calculate limb3-4 link4-5 transform  
static void calculateLimb3to4Link4to5Transform(
                                       const Eigen::Vector3d& start,
                                       const Eigen::Vector3d& stop,
                                       const tf::Vector3 m_x_axis,
                                       tf::Transform& link4_tf,
                                       tf::Transform& link5_tf)
{
    tf::Vector3 Z5=-m_x_axis;
    tf::Vector3 Z4=toTF(stop)-toTF(start);
    Z4.normalize();
    tf::Vector3 X4=Z4.cross(Z5);
    X4.normalize();
    tf::Vector3 X5=X4;

    //calculate limb3-4 link4 transform
    tf::Vector3 link4_origin;

    link4_origin.setX(stop(0)/1000.0);
    link4_origin.setY(stop(1)/1000.0);
    link4_origin.setZ(stop(2)/1000.0);

    link4_tf.setOrigin(link4_origin);

    tf::Vector3 Y4=Z4.cross(X4);

    tf::Matrix3x3 link4_rotation(
        X4.getX(), Y4.getX(), Z4.getX(),
        X4.getY(), Y4.getY(), Z4.getY(),
        X4.getZ(), Y4.getZ(), Z4.getZ());

    link4_tf.setBasis(link4_rotation);

    //calculate limb3-4 link5 transform
    tf::Vector3 link5_origin=link4_origin;

    link5_tf.setOrigin(link5_origin);

    tf::Vector3 Y5=Z5.cross(X5);

    tf::Matrix3x3 link5_rotation(
        X5.getX(), Y5.getX(), Z5.getX(),
        X5.getY(), Y5.getY(), Z5.getY(),
        X5.getZ(), Y5.getZ(), Z5.getZ());

    link5_tf.setBasis(link5_rotation);
}  

//calculate limb5 link1-4 transform  
static void calculateLimb5Transform(const Eigen::Vector3d& S5,
                                    const tf::Vector3 m_x_axis,
                                    tf::Transform& link1_tf,
                                    tf::Transform& link2_tf,
                                    tf::Transform& link3_tf,
                                    tf::Transform& link4_tf)  
{
    tf::Vector3 S5_Base=toTF(S5);

    //calculate link1 transform
    tf::Vector3 link1_origin;

    link1_origin.setX(fsp::limb5_link1_position[0]);
    link1_origin.setY(S5_Base[1]-fsp::limb5_link1_y);
    link1_origin.setZ(fsp::limb5_link1_position[2]);

    link1_tf.setOrigin(link1_origin);

    link1_tf.setBasis(fsp::limb5_link1_rotation);

    //calculate link2 transform
    tf::Vector3 link2_origin;

    link2_origin.setX(S5_Base[0]);
    link2_origin.setY(S5_Base[1]-fsp::limb5_link2_y);
    link2_origin.setZ(fsp::limb5_link2_position[2]);

    link2_tf.setOrigin(link2_origin);

    link2_tf.setBasis(fsp::limb5_link2_rotation);

    //calculate link3 transform
    tf::Vector3 link3_origin;

    link3_origin.setX(S5_Base[0]);
    link3_origin.setY(S5_Base[1]-fsp::limb5_link3_y);
    link3_origin.setZ(S5_Base[2]-fsp::limb5_link3_z);

    link3_tf.setOrigin(link3_origin);

    link3_tf.setBasis(fsp::limb5_link3_rotation);

    //calculate link4 transform
    tf::Vector3 link4_origin=S5_Base;

    link4_tf.setOrigin(link4_origin);

    tf::Vector3 X4=fsp::limb5_link3_rotation.getColumn(0);
    tf::Vector3 Z4=m_x_axis;
    tf::Vector3 Y4;
    Y4=Z4.cross(X4);

    tf::Matrix3x3 link4_rotation(
        X4.getX(), Y4.getX(), Z4.getX(),
        X4.getY(), Y4.getY(), Z4.getY(),
        X4.getZ(), Y4.getZ(), Z4.getZ());

    link4_tf.setBasis(link4_rotation);
}                                                                

fsp::FaradayStatePublisher::FaradayStatePublisher(
                          const std::string& joints_topic,
						  const std::vector<std::string>& joint_names)
    : joint_names_(joint_names)                          
{
    // joint_names_.resize(5);
    // joint_names_=joint_names;
    // printf("hahahaha2\n");
    // std::cout<<"I am here3"<<std::endl;

    ROS_ASSERT(joint_names_.size()==5);

    joint_sub_=nh_.subscribe<sensor_msgs::JointState>(joints_topic,1,
        boost::bind(&fsp::FaradayStatePublisher::updateJointPosition,
                   this,_1));
}


bool fsp::FaradayStatePublisher::extractJoints(const sensor_msgs::JointState& msg,
                                      double* actuators) const
{                
    // ROS_INFO_STREAM("msg.name.size is "<<msg.name.size());                    
    int indexes[5]={-1, -1, -1, -1, -1};
    for (int i = 0; i < (int)msg.name.size(); i++)
    {
        if(msg.name[i]==joint_names_[0])  indexes[0]=i;
        else if(msg.name[i]==joint_names_[1]) indexes[1]=i;
        else if(msg.name[i]==joint_names_[2]) indexes[2]=i;
        else if(msg.name[i]==joint_names_[3]) indexes[3]=i;
        else if(msg.name[i]==joint_names_[4]) indexes[4]=i;
    }

    //check for failure
    for(int i=0;i<5;++i)
        if(indexes[i]<0)
            return false;

    //Otherwise copy values, and continue on
    for(int i=0;i<5;++i)
        actuators[i]=msg.position[indexes[i]];

    return true;
}
     

void fsp::FaradayStatePublisher::updateJointPosition(const sensor_msgs::JointStateConstPtr& joints)
{
    double actuators[5];

    if(!extractJoints(*joints,actuators))
    {
        ROS_ERROR("JointState did not contain all of the robot joint names");
        return;
    }

    double pose[5];

    //using joint states, calculate forward kinematics of faraday
    if(!faraday_kinematics::forward_kinematics(actuators,pose,last_pose))
    {
        ROS_ERROR("Could not calculate FK for given pose");
        return;
    }

    /*std::cout<<"joint position is "<<actuators[0]<<" "<<actuators[1]<<" "
            <<actuators[2]<<" "<<actuators[3]<<" "<<actuators[4]<<std::endl;
    std::cout<<"current pose is "<<pose[0]<<" "<<pose[1]<<" "<<pose[2]<<" "
            <<pose[3]<<" "<<pose[4]<<std::endl;
    std::cout<<std::endl;*/

    for(int i=0;i<3;i++)
        last_pose[i]=pose[i];

    //move_platform transform
    tf::Transform move_platform_tf;
    tf::Vector3 move_platform_origin;

    move_platform_origin.setX(pose[0]/1000.0);
    move_platform_origin.setY(pose[1]/1000.0);
    move_platform_origin.setZ(pose[2]/1000.0);

    move_platform_tf.setOrigin(move_platform_origin);

    tf::Matrix3x3 move_platform_rotation;

    move_platform_rotation.setEulerYPR(0.0, pose[4], pose[3]);

    move_platform_tf.setBasis(move_platform_rotation);

    //using joints and　FK, calculate intermediate points on the robot
    //pts: A, B, C
    faraday_kinematics::IntermediatePoints pts;
    if(!faraday_kinematics::calcIntermediatePoints(actuators,pose,pts))
    {
        ROS_ERROR("Could not calculate intermediate point for given pose");
        return;
    }

    /*std::cout<<"pts.A"<<std::endl;
    std::cout<<"U1\t U2\t U3\t U4"<<std::endl<<pts.A<<std::endl<<std::endl;
    std::cout<<"pts.B"<<std::endl;
    std::cout<<"S1\t S2\t S3\t S4"<<std::endl<<pts.B<<std::endl<<std::endl;
    std::cout<<"pts.C"<<std::endl;
    std::cout<<"S5"<<std::endl<<pts.C<<std::endl<<std::endl;*/

    //using intermediate points, calculate transform to each link in model
    tf::Transform link1_tf,link2_tf,link3_tf,link4_tf,link5_tf;

    // limb1
    calculateLink1to3Transform(pts.A.col(0),pts.B.col(0),
                               limb1_link1_position,
                               limb1_link1_orientation,
                               link1_tf,link2_tf,link3_tf);
    tf_broadcaster_.sendTransform(tf::StampedTransform(link1_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb1_link1"));
    tf_broadcaster_.sendTransform(tf::StampedTransform(link2_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb1_link2")); 
    tf_broadcaster_.sendTransform(tf::StampedTransform(link3_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb1_link3")); 
    calculateLimb1to2Link4to5Transform(pts.A.col(0),pts.B.col(0),
                                       move_platform_rotation.getColumn(0),
                                       link4_tf,link5_tf);
    tf_broadcaster_.sendTransform(tf::StampedTransform(link4_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb1_link4"));
    tf_broadcaster_.sendTransform(tf::StampedTransform(link5_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb1_link5"));

    // limb2
    calculateLink1to3Transform(pts.A.col(1),pts.B.col(1),
                               limb2_link1_position,
                               limb2_link1_orientation,
                               link1_tf,link2_tf,link3_tf);
    tf_broadcaster_.sendTransform(tf::StampedTransform(link1_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb2_link1"));
    tf_broadcaster_.sendTransform(tf::StampedTransform(link2_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb2_link2")); 
    tf_broadcaster_.sendTransform(tf::StampedTransform(link3_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb2_link3")); 
    calculateLimb1to2Link4to5Transform(pts.A.col(1),pts.B.col(1),
                                       move_platform_rotation.getColumn(0),
                                       link4_tf,link5_tf);
    tf_broadcaster_.sendTransform(tf::StampedTransform(link4_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb2_link4"));
    tf_broadcaster_.sendTransform(tf::StampedTransform(link5_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb2_link5"));

    // limb3
    calculateLink1to3Transform(pts.A.col(2),pts.B.col(2),
                               limb3_link1_position,
                               limb3_link1_orientation,
                               link1_tf,link2_tf,link3_tf);
    tf_broadcaster_.sendTransform(tf::StampedTransform(link1_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb3_link1"));
    tf_broadcaster_.sendTransform(tf::StampedTransform(link2_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb3_link2")); 
    tf_broadcaster_.sendTransform(tf::StampedTransform(link3_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb3_link3")); 
    calculateLimb3to4Link4to5Transform(pts.A.col(2),pts.B.col(2),
                                       move_platform_rotation.getColumn(0),
                                       link4_tf,link5_tf);
    tf_broadcaster_.sendTransform(tf::StampedTransform(link4_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb3_link4"));
    tf_broadcaster_.sendTransform(tf::StampedTransform(link5_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb3_link5"));                                           

    // limb4
    calculateLink1to3Transform(pts.A.col(3),pts.B.col(3),
                               limb4_link1_position,
                               limb4_link1_orientation,
                               link1_tf,link2_tf,link3_tf);
    tf_broadcaster_.sendTransform(tf::StampedTransform(link1_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb4_link1"));
    tf_broadcaster_.sendTransform(tf::StampedTransform(link2_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb4_link2")); 
    tf_broadcaster_.sendTransform(tf::StampedTransform(link3_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb4_link3"));
    calculateLimb3to4Link4to5Transform(pts.A.col(3),pts.B.col(3),
                                       move_platform_rotation.getColumn(0),
                                       link4_tf,link5_tf);
    tf_broadcaster_.sendTransform(tf::StampedTransform(link4_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb4_link4"));
    tf_broadcaster_.sendTransform(tf::StampedTransform(link5_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb4_link5"));

    // limb5
    calculateLimb5Transform(pts.C,move_platform_rotation.getColumn(0),
                            link1_tf,link2_tf,link3_tf,link4_tf);
    tf_broadcaster_.sendTransform(tf::StampedTransform(link1_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb5_link1"));
    tf_broadcaster_.sendTransform(tf::StampedTransform(link2_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb5_link2")); 
    tf_broadcaster_.sendTransform(tf::StampedTransform(link3_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb5_link3"));
    tf_broadcaster_.sendTransform(tf::StampedTransform(link4_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "limb5_link4"));

    // move_platform
    tf_broadcaster_.sendTransform(tf::StampedTransform(move_platform_tf,
                                                     joints->header.stamp,
                                                     "base_link",
                                                     "move_platform"));                                                                                                                                      

}
