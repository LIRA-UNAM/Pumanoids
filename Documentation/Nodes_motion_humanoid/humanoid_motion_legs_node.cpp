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
  //odom_trans.transform.translation.x = 0.0;//cos(angle)*2;
  //odom_trans.transform.translation.y = 0.0;//sin(angle)*2;
  //odom_trans.transform.translation.z = 0.0;//.7;
  //odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

  //std::cout << FilaName << std::endl;

  joint_state.name.resize(12);
  joint_state.position.resize(12);

  joint_state.name[0] ="left_joint_leg_yaw";
  joint_state.name[1] ="left_joint_leg_pitch";
  joint_state.name[2] ="left_joint_leg_roll";
  joint_state.name[3] ="left_joint_knee_pitch";
  joint_state.name[4] ="left_joint_ankle_pitch";
  joint_state.name[5] ="left_joint_ankle_roll";

  joint_state.name[6] ="right_joint_leg_yaw";
  joint_state.name[7] ="right_joint_leg_pitch";
  joint_state.name[8] ="right_joint_leg_roll";
  joint_state.name[9] ="right_joint_knee_pitch";
  joint_state.name[10] ="right_joint_ankle_pitch";
  joint_state.name[11] ="right_joint_ankle_roll";

  std::cout << "<---main loop: while(ros_ok())--->" << std::endl;
  std::cout<< std::endl;

  while (ros::ok()) 
    {

      joint_state.header.stamp = ros::Time::now();

      joint_state.position[0] = rot4*degree;
      joint_state.position[1] = rot4*degree;
      joint_state.position[2] = rot4*degree;
      joint_state.position[3] = -rot4*degree;
      joint_state.position[4] = rot4*degree;
      joint_state.position[5] = rot4*degree;
      joint_state.position[6] = rot4*degree;
      joint_state.position[7] = -rot4*degree;
      joint_state.position[8] = rot4*degree;
      joint_state.position[9] = -rot4*degree;
      joint_state.position[10] = -rot4*degree;
      joint_state.position[11] = -rot4*degree;

      // update transform
      // (moving in a circle with radius=2)

      odom_trans.transform.translation.x = cos(angle)*2;
      odom_trans.transform.translation.y = sin(angle)*2;
      odom_trans.transform.translation.z = 0.7;
      odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
      odom_trans.header.stamp = ros::Time::now();

      //send the joint state and transform
      joint_pub.publish(joint_state);
      broadcaster.sendTransform(odom_trans);
      
      rot4 += 0.02;

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

