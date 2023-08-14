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
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace
{
// utility function
double trajectoryDuration(const trajectory_preview::RobotTrajectory& traj)
{
  return traj.getWayPointDurationFromStart(traj.getWayPointCount() - 1);
}

const static double timer_period = 1.0 / 30.0;

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

  void initialize(rclcpp::Node::SharedPtr node, const std::string& input_traj_topic,
                  const std::string& output_state_topic)
  {
    node_ = node;
    traj_sub_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        input_traj_topic, 1, std::bind(&TrajectoryPreviewImpl::onNewTrajectory, this, std::placeholders::_1));
    state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(output_state_topic, rclcpp::SystemDefaultsQoS());
  }

  void onNewTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
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

  void onAnimateCallback()
  {
    if (!display_traj_)
      return;

    // compute current time
    current_time_ = current_time_ + (timer_period * scale_);

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

    sensor_msgs::msg::JointState state_msg;

    // Compute the interpolated state
    display_traj_->getStateAtDurationFromStart(t, state_msg);

    // Display
    state_pub_->publish(state_msg);
    emit update(t / trajectory_duration_);
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

    // Start the timer
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(long((timer_period / scale_) * 1000.0)),
                                      std::bind(&TrajectoryPreviewImpl::onAnimateCallback, this));
  }

  void stopDisplay()
  {
    if (timer_)
      timer_->cancel();
  }

  void setScale(double new_scale)
  {
    scale_ = new_scale;
    if (timer_)
    {
      timer_->cancel();
      startDisplay();
    }
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
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double trajectory_duration_;
  double current_time_;
  double scale_;

  RobotTrajectoryPtr display_traj_;
};

}  // namespace trajectory_preview

#endif  // TRAJECTORY_PREVIEW_TRAJECTORY_PREVIEW_IMPL_H
