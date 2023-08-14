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
#ifndef TRAJECTORY_PREVIEW_PANEL_H
#define TRAJECTORY_PREVIEW_PANEL_H

#include <rviz_common/panel.hpp>

namespace trajectory_preview
{
class TrajectoryPreviewWidget;

class TrajectoryPreviewPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  using rviz_common::Panel::Panel;

  void onInitialize() override;

private:
  TrajectoryPreviewWidget* widget_;
};

}  // namespace trajectory_preview

#endif  // TRAJECTORY_PREVIEW_PANEL_H
