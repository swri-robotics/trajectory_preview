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
#ifndef TRAJECTORY_PREVIEW_TRAJECTORY_PREVIEW_WIDGET_H
#define TRAJECTORY_PREVIEW_TRAJECTORY_PREVIEW_WIDGET_H

#include <QWidget>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rclcpp/node.hpp>

namespace Ui
{
class TrajectoryPreview;
}

namespace trajectory_preview
{
class TrajectoryPreviewImpl;

class TrajectoryPreviewWidget : public QWidget
{
  Q_OBJECT
public:
  TrajectoryPreviewWidget(QWidget* parent = 0);
  ~TrajectoryPreviewWidget();

  /**
   * @brief Creates a ROS subscriber to a trajectory topic
   * and a publisher for the state of the robot
   * @param input_traj_topic
   * @param output_state_topic
   */
  void initializeROS(rclcpp::Node::SharedPtr node, const std::string& input_traj_topic, const std::string& output_state_topic);

protected Q_SLOTS:
  // User Interactions
  void onPlayPauseButton();
  void onScaleChanged(double new_scale);
  void onSliderChanged(int new_position);

  // Automatic ROS interactions
  void onNewTrajectory(double total_duration);
  void onTrajectoryFinished();
  void onUpdate(double ratio);

private:
  TrajectoryPreviewImpl* impl_;
  Ui::TrajectoryPreview* ui_;

  int num_ticks_;
};

}  // namespace trajectory_preview

#endif  // TRAJECTORY_PREVIEW_TRAJECTORY_PREVIEW_WIDGET_H
