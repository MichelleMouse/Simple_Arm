#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_mover");

  ros::NodeHandle nh;

  //Initialises publishers
  ros::Publisher joint1_pub = nh.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
  ros::Publisher joint2_pub = nh.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);


  ros::Rate loop_rate(10);

  int start_time, elapsed;
  while (not start_time) {
    start_time = ros::Time::now().toSec();
  }

  while (ros::ok()) {
    elapsed = ros::Time::now().toSec() - start_time;

    //Sets the arm joint angles
    std_msgs::Float64 joint1_angle, joint2_angle;
    joint1_angle.data = sin(2 * M_PI * 0.1 * elapsed) * (M_PI / 2);
    joint2_angle.data = sin(2 * M_PI * 0.1 * elapsed) * (M_PI / 2);

    //Publishes the arm joint angles
    joint1_pub.publish(joint1_angle);
    joint2_pub.publish(joint2_angle);

    //Sleeps for the time remaining until 10 Hz is reached
    loop_rate.sleep();
  }

  return 0;
}
