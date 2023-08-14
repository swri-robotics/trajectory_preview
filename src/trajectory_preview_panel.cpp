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
#include "trajectory_preview/trajectory_preview_panel.h"
#include "trajectory_preview/trajectory_preview_widget.h"

#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <rclcpp/node.hpp>

namespace trajectory_preview
{
void TrajectoryPreviewPanel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Declare and get the input trajectory
  node->declare_parameter<std::string>("trajectory_topic", "trajectory");
  auto input_topic = node->get_parameter("trajectory_topic").get_value<std::string>();

  node->declare_parameter<std::string>("display_topic", "trajectory/joint_states");
  auto output_topic = node->get_parameter("display_topic").get_value<std::string>();

  widget_ = new TrajectoryPreviewWidget();
  widget_->initializeROS(node, input_topic, output_topic);

  // Add the widget to the panel layout
  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->addWidget(widget_);
  setLayout(layout);
}

}  // namespace trajectory_preview

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(trajectory_preview::TrajectoryPreviewPanel, rviz_common::Panel)
