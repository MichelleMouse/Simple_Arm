#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <std_msgs/Float64.h>

class Publishers
{
public:
  Publishers(ros::NodeHandle* nh) : m_nh(*nh)
  {
    m_joint1_pub = m_nh.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
    m_joint2_pub = m_nh.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);
  }

  void publish(ros::Publisher &pb, std_msgs::Float64 &msg)
  {
    pb.publish(msg);
  }

  bool handle_safe_move_request(simple_arm::GoToPosition::Request& req, simple_arm::GoToPosition::Response& res);
  std::vector<float> clamp_at_boundaries(float requested_j1, float requested_j2);

private:
  ros::Publisher m_joint1_pub, m_joint2_pub;
  ros::NodeHandle m_nh;
};

//Global joint publisher variables
// ros::Publisher joint1_pub, joint2_pub;

//Checks and clamps the joint angles to a safe zone
std::vector<float> Publishers::clamp_at_boundaries(float requested_j1, float requested_j2)
{
  //Defines clamped joint angles and assign them to the requested ones
  float clamped_j1 = requested_j1;
  float clamped_j2 = requested_j2;

  float min_j1, max_j1, min_j2, max_j2;
  ros::NodeHandle n2;

  std::string node_name = ros::this_node::getName();    //Gets node name

  //Gets joints min and max parameters
  n2.getParam(node_name + "/min_joint_1_angle", min_j1);
  n2.getParam(node_name + "/max_joint_1_angle", max_j1);
  n2.getParam(node_name + "/min_joint_2_angle", min_j2);
  n2.getParam(node_name + "/max_joint_2_angle", max_j2);

  //Checks if joint 1 falls in the safe zone, otherwise clamp it
  if (requested_j1 < min_j1 || requested_j1 > max_j1) {
    clamped_j1 = std::min(std::max(requested_j1, min_j1), max_j1);
    ROS_WARN("j1 is out of bounds, valid range (%1.2f,%1.2f), clamping to: %1.2f", min_j1, max_j1, clamped_j1);
  }

  //Checks if joint 2 falls in the safe zone, otherwise clamp it
  if (requested_j2 < min_j2 || requested_j2 > max_j2) {
    clamped_j2 = std::min(std::max(requested_j2, min_j2), max_j2);
    ROS_WARN("j2 is out of bounds, valid range (%1.2f,%1.2f), clamping to: %1.2f", min_j2, max_j2, clamped_j2);
  }

  //Stores clamped joint angles in a clamped_data vector
  std::vector<float> clamped_data = { clamped_j1, clamped_j2 };

  return clamped_data;
}

//Executes whenever a safe_move service is requested
bool Publishers::handle_safe_move_request(simple_arm::GoToPosition::Request& req, simple_arm::GoToPosition::Response& res)
{

  ROS_INFO("GoToPositionRequest received - j1:%1.2f, j2:%1.2f", (float)req.joint_1, (float)req.joint_2);

  //Checks if requested joint angles are in the safe zone, otherwise clamp them
  std::vector<float> joints_angles = clamp_at_boundaries(req.joint_1, req.joint_2);

  //Publishes clamped joint angles to the arm
  std_msgs::Float64 joint1_angle, joint2_angle;
  joint1_angle.data = joints_angles[0];
  joint2_angle.data = joints_angles[1];
  // joint1_pub.publish(joint1_angle);
  // joint2_pub.publish(joint2_angle);
  m_joint1_pub.publish(joint1_angle);
  m_joint2_pub.publish(joint2_angle);

  //Waits 3 seconds for arm to settle
  ros::Duration(3).sleep();

  //Returns a response message
  res.msg_feedback = "Joint angles set - j1: " + std::to_string(joints_angles[0]) + " , j2: " + std::to_string(joints_angles[1]);
  ROS_INFO_STREAM(res.msg_feedback);

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_mover");
  ros::NodeHandle n;

  Publishers pubs(&n);

  // joint1_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
  // joint2_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);

  //Defines a safe_move service with a handle_safe_move_request callback function
  ros::ServiceServer service = n.advertiseService("/arm_mover/safe_move", &Publishers::handle_safe_move_request, (Publishers*) &pubs);
  ROS_INFO("Ready to send joint commands");

  //Handles ROS communication events
  ros::spin();

  return 0;
}
