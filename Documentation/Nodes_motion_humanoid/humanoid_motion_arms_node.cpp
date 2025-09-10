#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <unistd.h>

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle n;
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  tf::TransformBroadcaster broadcaster;
  ros::Rate loop_rate(100);

  const double degree = M_PI/180;
  // robot state
  double angle=0;
  double rot4 = 0;
  // message declarations
  geometry_msgs::TransformStamped odom_trans;
  sensor_msgs::JointState joint_state;

  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "trunk";

  //odom_trans.header.stamp = ros::Time::now();
  odom_trans.transform.translation.x = 0.0;//cos(angle)*2;
  odom_trans.transform.translation.y = 0.0;//sin(angle)*2;
  odom_trans.transform.translation.z = 0.0;//.7;
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

  //std::cout << FilaName << std::endl;

  joint_state.name.resize(6);
  joint_state.position.resize(6);

  joint_state.name[0] ="left_joint_shoulder_pitch";
  joint_state.name[1] ="left_joint_shoulder_roll";
  joint_state.name[2] ="left_joint_elbow_pitch";

  joint_state.name[3] ="right_joint_shoulder_pitch";
  joint_state.name[4] ="right_joint_shoulder_roll";
  joint_state.name[5] ="right_joint_elbow_pitch";

  std::cout << "<---main loop: while(ros_ok())--->" << std::endl;
  std::cout<< std::endl;

  while (ros::ok()) 
    {

      joint_state.header.stamp = ros::Time::now();

      joint_state.position[0] = 0;//rot4*degree;
      joint_state.position[1] = 0;//rot4*degree;
      joint_state.position[2] = 0;//rot4*degree;

      joint_state.position[3] = 0;//rot4*degree;
      joint_state.position[4] = 0;//rot4*degree;
      joint_state.position[5] = 0;//rot4*degree;

      // update transform
      // (moving in a circle with radius=2)
      odom_trans.header.stamp = ros::Time::now();

      //send the joint state and transform
      joint_pub.publish(joint_state);
      broadcaster.sendTransform(odom_trans);
      
      rot4 += 0.2;

      if (rot4 > 90) 
        {
          rot4 = 0;
        }

      // This will adjust as needed per iteration
      ros::spinOnce();
      loop_rate.sleep();
    }

  return 0;
  }
