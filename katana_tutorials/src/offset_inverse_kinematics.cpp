/*
 * follow_joint_trajectory_client.cpp
 *
 *  Created on: 06.11.2011
 *      Author: martin
 */

#include <katana_tutorials/follow_joint_trajectory_client.h>
#include<iostream>
#include<cmath>


double Theta_3(double a2,double a3,double x,double y,double z,double d){
//        double theta = 0;
//        theta = std::acos((pow(a2,2) + std::pow(a3,2) - std::pow(x,2) - std::pow(z,2))/(2*a2*a3));
//        return theta;
        double theta = 0,D = 0;
        D = (std::pow(x,2) + std::pow(y,2) + std::pow(z,2) - std::pow(d,2) - std::pow(a2,2) - std::pow(a3,2))/(2*a2*a3);
        theta = std::atan2(-std::sqrt(1 - std::pow(D,2)),D);// need to be config
        return theta;

}

double Theta_2(double a2,double a3,double x,double y,double z,double d,double theta3){
//        double theta = 0 ;
//      
//        theta = std::acos((std::pow(a2,2) - std::pow(a3,2) + std::pow(x,2) + std::pow(z,2))/(2*a2*std::sqrt(pow(x,2) + std::pow(z,2))));
//        return theta;
        double theta = 0,s3 = 0,c3 = 0;
        s3 = std::sin(theta3);
        c3 = std::cos(theta3);
        theta = std::atan2(z,std::sqrt(std::pow(x,2) + std::pow(y,2) - std::pow(d,2))) - std::atan2(a3*s3,a2 + (a3*c3));
        return theta;

}


double Theta_1(double x,double y,double d){
        double theta = 0;
	if(d != 0){
            theta = std::atan2(y,x) + atan2(-(std::sqrt(std::pow(x,2) + std::pow(y,2) - std::pow(d,2))),d);
	}else{
	    theta = std::atan2(y,x);
	}
        return theta;
}


namespace katana_tutorials
{

FollowJointTrajectoryClient::FollowJointTrajectoryClient() :
    traj_client_("/katana_arm_controller/follow_joint_trajectory", true), got_joint_state_(false), spinner_(1)
{
  joint_names_.push_back("katana_motor1_pan_joint");
  joint_names_.push_back("katana_motor2_lift_joint");
  joint_names_.push_back("katana_motor3_lift_joint");
  joint_names_.push_back("katana_motor4_lift_joint");
  joint_names_.push_back("katana_motor5_wrist_roll_joint");

  joint_state_sub_ = nh_.subscribe("/joint_states", 1, &FollowJointTrajectoryClient::jointStateCB, this);
  spinner_.start();

  // wait for action server to come up
  while (!traj_client_.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the follow_joint_trajectory server");
  }
}

FollowJointTrajectoryClient::~FollowJointTrajectoryClient()
{
}

void FollowJointTrajectoryClient::jointStateCB(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> ordered_js;

  ordered_js.resize(joint_names_.size());

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    bool found = false;
    for (size_t j = 0; j < msg->name.size(); ++j)
    {
      if (joint_names_[i] == msg->name[j])
      {
        ordered_js[i] = msg->position[j];
        found = true;
        break;
      }
    }
    if (!found)
      return;
  }

  ROS_INFO_ONCE("Got joint state!");
  current_joint_state_ = ordered_js;
  got_joint_state_ = true;
}

//! Sends the command to start a given trajectory
void FollowJointTrajectoryClient::startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
{
  // When to start the trajectory: 1s from now
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  traj_client_.sendGoal(goal);
}

control_msgs::FollowJointTrajectoryGoal FollowJointTrajectoryClient::makeArmUpTrajectory()
{
 
  double a2 = 35,a3 = 25,x,y,z,d1 = 0;
  double theta1 = 0, theta2 = 0 , theta3 = 0,finalPointAngle = 0, p = 0;
  std::cin >> x >> y >> z;

  p = std::sqrt(std::pow(x,2) + std::pow(y,2));
  
  if(x > a2 + a3 || y > a2 + a3 || z > a2 + a3 || p > a2+ a3 || std::sqrt(std::pow(p,2) + std::pow(z,2)) > a2 + a3){
     x = 60;
     y = 0;
     z = 0;

     std::cout << "Invalid coordinate,Please fill in new coordinate" <<  std::endl;
   
  }
  theta3 = Theta_3(a2,a3,x,y,z,d1);
  theta2 = Theta_2(a2,a3,x,y,z,d1,theta3);
  if(x != 0 || y != 0){
     theta1 = Theta_1(x,y,d1);

  }else{
     theta1 = current_joint_state_[0];

  }

  const size_t NUM_TRAJ_POINTS = 3;
  const size_t NUM_JOINTS = 5;

  // positions after calibration
  std::vector<double> calibration_positions(NUM_JOINTS);
  calibration_positions[0] = theta1;
  calibration_positions[1] = theta2;
  calibration_positions[2] = 0.0;
  calibration_positions[3] = -theta3;
  calibration_positions[4] = 0.0;

  // arm pointing straight up
  std::vector<double> straight_up_positions(NUM_JOINTS);
  straight_up_positions[0] = 0.0;
  straight_up_positions[1] = 2.57;
  straight_up_positions[2] = 0.0;
  straight_up_positions[3] = 0.0;
  straight_up_positions[4] = 0.0;

  trajectory_msgs::JointTrajectory trajectory;

  for (ros::Rate r = ros::Rate(10); !got_joint_state_; r.sleep())
  {
    ROS_DEBUG("waiting for joint state...");

    if (!ros::ok())
      exit(-1);
  }

  // First, the joint names, which apply to all waypoints
  trajectory.joint_names = joint_names_;

  trajectory.points.resize(NUM_TRAJ_POINTS);

  // trajectory point:
  int ind = 0;
  trajectory.points[ind].time_from_start = ros::Duration(5 * ind);
  trajectory.points[ind].positions = current_joint_state_;

  // trajectory point:
  ind++;
  trajectory.points[ind].time_from_start = ros::Duration(5 * ind);
  trajectory.points[ind].positions = calibration_positions;

  // trajectory point:
  ind++;
  trajectory.points[ind].time_from_start = ros::Duration(5 * ind);
  trajectory.points[ind].positions.resize(NUM_JOINTS);
  trajectory.points[ind].positions = calibration_positions;

  //  // all Velocities 0
  //  for (size_t i = 0; i < NUM_TRAJ_POINTS; ++i)
  //  {
  //    trajectory.points[i].velocities.resize(NUM_JOINTS);
  //    for (size_t j = 0; j < NUM_JOINTS; ++j)
  //    {
  //      trajectory.points[i].velocities[j] = 0.0;
  //    }
  //  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  return goal;
}

//! Returns the current state of the action
actionlib::SimpleClientGoalState FollowJointTrajectoryClient::getState()
{
  return traj_client_.getState();
}

} /* namespace katana_tutorials */

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "follow_joint_trajectory_client");

  katana_tutorials::FollowJointTrajectoryClient arm;
  // Start the trajectory
  arm.startTrajectory(arm.makeArmUpTrajectory());
  // Wait for trajectory completion
  while (!arm.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }
}
