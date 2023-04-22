#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <string.h>

// ROS
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include "geometry_msgs/Point.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// FSW
#include <ff_msgs/EkfState.h>
#include <ff_msgs/ControlState.h>

using namespace Eigen;

namespace fake_setpoint {

std::string TOPIC_SETPOINT = "/queen/gnc/ctl/setpoint/pd";
std::string PARAM_CONTROL = "/queen/td/controller";

class FakeSetpoint {
 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_setpoint_;

  // Forward declaring full pose messages for test cases
  ff_msgs::ControlState setpoint_init;
  ff_msgs::ControlState setpoint_stepx;
  ff_msgs::ControlState setpoint_stepy;
  ff_msgs::ControlState setpoint_stepz;

  int t_test = 20; // Time between each new step command

  // Default pose: [10.9 -8.05 4.9], [0.7071068 0 0 0.7071068]
  // Default position point:
  geometry_msgs::Point xyz_init_msg;
  double xyz_init [3] = { 10.9, -8.05, 4.9 };

  // Attitude step test cases
  double rpy_init  [3] = { 0       , 0       , 1.570796 };
  double rpy_stepx [3] = { 0.174533, 0       , 1.570796 }; // +10deg x-axis rotation (inertial, -y body)
  double rpy_stepy [3] = { 0       , 0.174533, 1.570796 }; // +10deg y-axis rotation (inertial, +x body)
  double rpy_stepz [3] = { 0       , 0       , 1.745330 }; // +10deg z-axis rotation (inertial, +z body)

  tf2::Quaternion q_init_tf;
  tf2::Quaternion q_stepx_tf;
  tf2::Quaternion q_stepy_tf;
  tf2::Quaternion q_stepz_tf;

  geometry_msgs::Quaternion q_init_msg;
  geometry_msgs::Quaternion q_stepx_msg;
  geometry_msgs::Quaternion q_stepy_msg;
  geometry_msgs::Quaternion q_stepz_msg;

 public:
  void setpoint_main() {
    pub_setpoint_ = nh_.advertise<ff_msgs::ControlState>(TOPIC_SETPOINT, 1, true);
    ros::param::set(PARAM_CONTROL, "pd");

    // Fix the initial position in a geometry_msgs::Point, to pass into each Pose
    xyz_init_msg.x = xyz_init[0];
    xyz_init_msg.y = xyz_init[1];
    xyz_init_msg.z = xyz_init[2];

    // Compute quaternion for each case
    // Quaternion notation: [w, x, y, z]
    q_init_tf.setRPY( rpy_init[0], rpy_init[1], rpy_init[2] );
    q_stepx_tf.setRPY( rpy_stepx[0], rpy_stepx[1], rpy_stepx[2] );
    q_stepy_tf.setRPY( rpy_stepy[0], rpy_stepy[1], rpy_stepy[2] );
    q_stepz_tf.setRPY( rpy_stepz[0], rpy_stepz[1], rpy_stepz[2] );

    // Convert tf2 messages to geometry messages, for Astrobee compatability
    q_init_msg  = tf2::toMsg(q_init_tf);
    q_stepx_msg = tf2::toMsg(q_stepx_tf);
    q_stepy_msg = tf2::toMsg(q_stepy_tf);
    q_stepz_msg = tf2::toMsg(q_stepz_tf);

    // Set the initial position coordinates for all test cases
    setpoint_init.pose.position  = xyz_init_msg;
    setpoint_stepx.pose.position = xyz_init_msg;
    setpoint_stepy.pose.position = xyz_init_msg;
    setpoint_stepz.pose.position = xyz_init_msg;

    // Set the quaternion geometry message for each associated test case
    setpoint_init.pose.orientation  = q_init_msg;
    setpoint_stepx.pose.orientation = q_stepx_msg;
    setpoint_stepy.pose.orientation = q_stepy_msg;
    setpoint_stepz.pose.orientation = q_stepz_msg;

    if (ros::ok()) {
      // Run tests with t_test sec offset
      sleep(3); // An initial lag to set up the rosbag

      pub_setpoint_.publish(setpoint_init);  sleep(5); // set attitude to spawn values

      pub_setpoint_.publish(setpoint_stepx); sleep(t_test); // x-axis rotation unit test
      pub_setpoint_.publish(setpoint_init);  sleep(t_test);

      pub_setpoint_.publish(setpoint_stepy); sleep(t_test); // y-axis rotation unit test
      pub_setpoint_.publish(setpoint_init);  sleep(t_test);

      pub_setpoint_.publish(setpoint_stepz); sleep(t_test); // z-axis rotation unit test
      pub_setpoint_.publish(setpoint_init);  sleep(t_test);

      // ros::spinOnce();
    }
    else {
      std::cout << "[FAKE SETPOINT] ROS not working properly...looping back." << std::endl;
      ros::spinOnce();
    }
  }  // setpoint_main()  
};  // class FakeSetpoint()
}  // namespace fake_setpoint

int main(int argc, char** argv) {
  ros::init(argc, argv, "fake_setpoint");
  fake_setpoint::FakeSetpoint my_setpoint;
  my_setpoint.setpoint_main();
  return 1;
}
