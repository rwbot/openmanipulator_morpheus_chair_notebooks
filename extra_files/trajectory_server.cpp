#include "open_manipulator_core/trajectory_server.hpp"
#include <array>
#include <numeric>
#include <vector>

DynamixelTrajectoryAction::DynamixelTrajectoryAction(const std::string &name)
    : as_(nh_, name,
          boost::bind(&DynamixelTrajectoryAction::executeCB, this, _1), false),
      action_name_(name) {
  jointPub =
      nh_.advertise<sensor_msgs::JointState>("goal_dynamixel_position", 1);
  jointSub = nh_.subscribe<sensor_msgs::JointState>(
      "joint_states", 1,
      boost::bind(&DynamixelTrajectoryAction::onJointState, this, _1));
  as_.start();
}

void DynamixelTrajectoryAction::onJointState(
    const sensor_msgs::JointState::ConstPtr &joints) {
  currentState = *joints;
}

std::vector<double>
DynamixelTrajectoryAction::calculatePoint(ros::Duration curTime) {
  while (curTime > points[idx + 1].time_from_start)
    idx++;
  double percent = (curTime.toSec() - points[idx].time_from_start.toSec()) /
                   (points[idx + 1].time_from_start.toSec() -
                    points[idx].time_from_start.toSec()); // Inverse Lerp
  ROS_INFO_STREAM("CurTime: " << curTime << "\nTime(" << idx
                              << "): " << points[idx].time_from_start.toSec()
                              << "\nTime(" << idx + 1 << "): "
                              << points[idx + 1].time_from_start.toSec()
                              << "\nPrecent: " << percent << "\n");
  std::vector<double> retval;
  for (int i = 0; i < points[idx].positions.size(); i++) {
    retval.push_back(points[idx].positions[i] +
                     (points[idx + 1].positions[i] - points[idx].positions[i]) *
                         percent); // Lerp
  }
  return retval;
}

void DynamixelTrajectoryAction::executeCB(
    const control_msgs::FollowJointTrajectoryGoalConstPtr &goal) {
  idx = 0;
  ROS_INFO_STREAM("Starting path...");
  bool success = true;
  feedback_.joint_names = currentState.name;
  points = goal->trajectory.points;
  ros::Rate rate(250); // 250hz 4ms set in dynamixel workbench

  targetState = currentState;
  targetState.header.seq = 0;
  targetState.name = currentState.name;

  ros::Time startTime = ros::Time::now();
  ros::Time endTime = startTime + points[points.size() - 1].time_from_start;
  ros::Time now = startTime;
  while (idx < points.size() - 2 && ros::ok()) {
    now = ros::Time::now();
    ros::Duration nowFromStart = now - startTime;
    std::vector<double> curPointPositions = calculatePoint(nowFromStart);

    targetState.header.seq++;
    targetState.position.clear();
    for (size_t i = 0; i < curPointPositions.size(); i++) {
      targetState.position.push_back(curPointPositions[i]);
    }
    targetState.position.push_back(currentState.position[6]);

    if (as_.isPreemptRequested()) {
      as_.setPreempted();
      success = false;
      break;
    }

    targetState.header.stamp = ros::Time::now();
    jointPub.publish(targetState);

    feedback_.desired = points[idx];
    feedback_.actual.positions = targetState.position;
    feedback_.actual.velocities = targetState.velocity;
    feedback_.actual.effort = targetState.effort;

    nowFromStart = ros::Time::now() - startTime;
    feedback_.actual.time_from_start = nowFromStart;

    feedback_.error.positions =
        targetState.position - feedback_.actual.positions;
    feedback_.error.velocities =
        targetState.velocity - feedback_.actual.velocities;
    feedback_.error.effort = targetState.effort - feedback_.actual.effort;

    as_.publishFeedback(feedback_);
    bool limit_exceeded = false;
    for (size_t jointIdx = 0; jointIdx < feedback_.error.positions.size();
         jointIdx++) {
      if (feedback_.error.positions[jointIdx] > max_position_error) {
        limit_exceeded = true;
        break;
      }
    }

    if (limit_exceeded) {
      ROS_ERROR_STREAM("Too high position error");
      result_.error_code = result_.PATH_TOLERANCE_VIOLATED;
      as_.setAborted(result_);
      success = false;
      break;
    }
    rate.sleep();
  }

  targetState.header.seq++;
  targetState.position.clear();
  for (size_t i = 0; i < points[points.size() - 1].positions.size(); i++) {
    targetState.position.push_back(points[points.size() - 1].positions[i]);
  }
  targetState.position.push_back(currentState.position[6]);

  targetState.header.stamp = ros::Time::now();
  jointPub.publish(targetState);

  if (success) {
    result_.error_code = result_.SUCCESSFUL;
    ROS_INFO_STREAM("Current trajectory complete");
    as_.setSucceeded(result_);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_action_server");

  ROS_INFO("Starting action server!");
  DynamixelTrajectoryAction action("arm/follow_joint_trajectory");

  ROS_INFO("Spinning...");
  ros::spin();

  return 0;
}
