/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2019, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TRAJECTORY_PREVIEW_TRAJECTORY_PREVIEW_IMPL_H
#define TRAJECTORY_PREVIEW_TRAJECTORY_PREVIEW_IMPL_H

#include "robot_trajectory.h"
#include <QObject>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace
{
// utility function
double trajectoryDuration(const trajectory_preview::RobotTrajectory& traj)
{
  return traj.getWayPointDurationFromStart(traj.getWayPointCount() - 1);
}

const static double target_fps = 15.0;

}  // namespace

namespace trajectory_preview
{
class TrajectoryPreviewImpl : public QObject
{
  Q_OBJECT
public:
  TrajectoryPreviewImpl() : QObject(), scale_(1.0)
  {
  }

  void initialize(const std::string& input_traj_topic, const std::string& output_state_topic)
  {
    ros::NodeHandle nh;

    std::string urdf_xml_string, srdf_xml_string;
    if (!nh.getParam("robot_description", urdf_xml_string) ||
        !nh.getParam("robot_description_semantic", srdf_xml_string))
    {
      ROS_ERROR_STREAM("Failed to get URDF and/or SRDF");
      return;
    }

    traj_sub_ = nh.subscribe(input_traj_topic, 1, &TrajectoryPreviewImpl::onNewTrajectory, this);
    state_pub_ = nh.advertise<sensor_msgs::JointState>(output_state_topic, 1, true);
    timer_ = nh.createTimer(ros::Rate(target_fps), &TrajectoryPreviewImpl::onAnimateCallback, this, false, false);
  }

  void onNewTrajectory(const trajectory_msgs::JointTrajectoryConstPtr& msg)
  {
    stopDisplay();

    // Prepare the new trajectory message
    display_traj_.reset(new RobotTrajectory(*msg));

    trajectory_duration_ = trajectoryDuration(*display_traj_);

    // Let listeners know that a new trajectory is ready
    emit newTrajectory(trajectory_duration_);

    // Reset state associated with trajectory playback
    current_time_ = 0.0;

    startDisplay();
  }

  void onAnimateCallback(const ros::TimerEvent& e)
  {
    if (!display_traj_)
      return;

    auto step = (e.current_real - e.last_real).toSec() * scale_;
    if (e.last_real == ros::Time())  // Last time is zero if the timer gets restarted
    {
      step = 0;
    }

    // compute current time
    current_time_ = current_time_ + step;

    if (current_time_ < trajectory_duration_)
    {
      publishStateAtTime(current_time_);
      emit update(current_time_ / trajectory_duration_);
    }
    else
    {
      // Make sure we always publish the last state even if we've gone over the
      // end of the trajectory
      publishStateAtTime(trajectory_duration_);
      emit update(1.0);
      emit trajectoryFinished();
      stopDisplay();
    }
  }

  void publishStateAtTime(double t)
  {
    if (!display_traj_)
      return;

    sensor_msgs::JointState state_msg;

    // Compute the interpolated state
    display_traj_->getStateAtDurationFromStart(t, state_msg);

    // Display
    state_pub_.publish(state_msg);
  }

  void startDisplay()
  {
    if (!display_traj_)
      return;

    if (current_time_ > 0.999 * trajectory_duration_)
    {
      // reset timer
      current_time_ = 0.0;
    }
    timer_.start();
  }

  void stopDisplay()
  {
    timer_.stop();
  }

  void setScale(double new_scale)
  {
    scale_ = new_scale;
  }

  void setCurrentTime(int index, int num_indexes)
  {
    if (!display_traj_)
      return;

    const auto total_time = trajectory_duration_;
    current_time_ = total_time * static_cast<double>(index) / static_cast<double>(num_indexes);
    publishStateAtTime(current_time_);
  }

  double currentTrajectoryDuration() const
  {
    return trajectory_duration_;
  }

Q_SIGNALS:
  void newTrajectory(double total_time);
  void trajectoryFinished();
  void update(double ratio);

private:
  ros::Subscriber traj_sub_;
  ros::Publisher state_pub_;
  ros::Timer timer_;

  double trajectory_duration_;
  double current_time_;
  double scale_;

  RobotTrajectoryPtr display_traj_;
};

}  // namespace trajectory_preview

#endif  // TRAJECTORY_PREVIEW_TRAJECTORY_PREVIEW_IMPL_H
