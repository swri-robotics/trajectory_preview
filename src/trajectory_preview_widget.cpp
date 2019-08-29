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
#include "trajectory_preview/trajectory_preview_widget.h"
#include "trajectory_preview_impl.h"
#include "ui_trajectory_preview_widget.h"

namespace trajectory_preview {

TrajectoryPreviewWidget::TrajectoryPreviewWidget(QWidget *parent)
    : QWidget(parent), impl_(nullptr) {
  ui_ = new Ui::TrajectoryPreview();
  ui_->setupUi(this);

  num_ticks_ = ui_->timeSlider->maximum() - ui_->timeSlider->minimum();

  // QT Events
  connect(ui_->playButton, SIGNAL(clicked()), this, SLOT(onPlayPauseButton()));
  connect(ui_->scaleSpinBox, SIGNAL(valueChanged(double)), this,
          SLOT(onScaleChanged(double)));
  connect(ui_->timeSlider, SIGNAL(sliderMoved(int)), this,
          SLOT(onSliderChanged(int)));

  // Create ROS stuff
  impl_ = new TrajectoryPreviewImpl();
  connect(impl_, SIGNAL(newTrajectory(double)), this,
          SLOT(onNewTrajectory(double)));
  connect(impl_, SIGNAL(update(double)), this, SLOT(onUpdate(double)));
  connect(impl_, SIGNAL(trajectoryFinished()), this,
          SLOT(onTrajectoryFinished()));
}

TrajectoryPreviewWidget::~TrajectoryPreviewWidget() { delete impl_; }

void TrajectoryPreviewWidget::onPlayPauseButton() {
  if (ui_->playButton->isChecked()) {
    impl_->startDisplay();
    ui_->playButton->setText(tr("Pause"));
  } else {
    impl_->stopDisplay();
    ui_->playButton->setText(tr("Play"));
  }
}

void TrajectoryPreviewWidget::onScaleChanged(double new_scale) {
  impl_->setScale(new_scale);
}

void TrajectoryPreviewWidget::onSliderChanged(int new_position) {
  impl_->setCurrentTime(new_position, num_ticks_);
}

void TrajectoryPreviewWidget::onNewTrajectory(double total_duration) {
  ui_->timeSlider->setValue(ui_->timeSlider->minimum());
  ui_->playButton->setChecked(true);
  ui_->playButton->setText(tr("Pause"));
  ui_->totalLabel->setText(QString::number(total_duration, 'g', 4));
}

void TrajectoryPreviewWidget::onTrajectoryFinished() {
  ui_->playButton->setChecked(false);
  ui_->playButton->setText(tr("Play"));
}

void TrajectoryPreviewWidget::onUpdate(double ratio) {
  int tick = static_cast<int>(ratio * num_ticks_ + 0.5);
  ui_->timeSlider->setValue(tick);
  ui_->currentLabel->setText(
      QString::number(ratio * impl_->currentTrajectoryDuration(), 'g', 4));
}

void TrajectoryPreviewWidget::initializeROS(
    const std::string &input_traj_topic,
    const std::string &output_state_topic) {
  impl_->initialize(input_traj_topic, output_state_topic);
}

void TrajectoryPreviewWidget::setTrajectory(
    const trajectory_msgs::JointTrajectory &trajectory) {
  impl_->onNewTrajectory(
      boost::make_shared<trajectory_msgs::JointTrajectory>(trajectory));
}

} // namespace trajectory_preview
