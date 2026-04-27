// Copyright 2026 TurtleBot3 PPO benchmark authors.
//
// Licensed under the Apache License, Version 2.0 (the "License").

#include "turtlebot3_gazebo/pedestrian_plugin.hpp"

#include <cmath>

#include <gazebo/common/Console.hh>

namespace gazebo
{

void PedestrianPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;

  if (_sdf->HasElement("speed")) {
    speed_ = _sdf->Get<double>("speed");
  }
  if (_sdf->HasElement("z")) {
    z_ = _sdf->Get<double>("z");
  }

  if (!_sdf->HasElement("waypoints")) {
    gzerr << "PedestrianPlugin: <waypoints> element required\n";
    return;
  }

  auto wp_elem = _sdf->GetElement("waypoints");
  if (!wp_elem->HasElement("point")) {
    gzerr << "PedestrianPlugin: <waypoints> needs at least one <point>\n";
    return;
  }
  for (auto p = wp_elem->GetElement("point"); p; p = p->GetNextElement("point")) {
    auto v = p->Get<ignition::math::Vector3d>();
    waypoints_.emplace_back(v.X(), v.Y());
  }
  if (waypoints_.size() < 2) {
    gzerr << "PedestrianPlugin: need at least 2 waypoints (got "
          << waypoints_.size() << ")\n";
    return;
  }

  // Cumulative arc length, with closing segment back to waypoint 0 so the
  // path is a closed loop.
  cum_lengths_.reserve(waypoints_.size() + 1);
  cum_lengths_.push_back(0.0);
  for (size_t i = 1; i < waypoints_.size(); ++i) {
    cum_lengths_.push_back(
      cum_lengths_.back() + waypoints_[i].Distance(waypoints_[i - 1]));
  }
  cum_lengths_.push_back(
    cum_lengths_.back() + waypoints_.back().Distance(waypoints_[0]));
  total_length_ = cum_lengths_.back();

  // Disable physics interaction — we drive the pose directly.
  model_->SetGravityMode(false);
  model_->SetStatic(false);

  start_time_ = model_->GetWorld()->SimTime();
  update_conn_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&PedestrianPlugin::OnUpdate, this, std::placeholders::_1));

  gzmsg << "PedestrianPlugin loaded for " << model_->GetName()
        << " (loop length " << total_length_ << " m, speed " << speed_
        << " m/s, " << waypoints_.size() << " waypoints)\n";
}

void PedestrianPlugin::OnUpdate(const common::UpdateInfo & _info)
{
  if (waypoints_.size() < 2 || total_length_ <= 0.0) {
    return;
  }

  const double t = (_info.simTime - start_time_).Double();
  const double s = std::fmod(std::max(0.0, t) * speed_, total_length_);

  // Locate segment such that cum_lengths_[i] <= s < cum_lengths_[i+1].
  size_t i = 0;
  while (i + 1 < cum_lengths_.size() && cum_lengths_[i + 1] <= s) {
    ++i;
  }
  const size_t b = (i + 1) % waypoints_.size();
  const auto & pa = waypoints_[i % waypoints_.size()];
  const auto & pb = waypoints_[b];
  const double seg_len = cum_lengths_[i + 1] - cum_lengths_[i];
  const double frac = (seg_len > 1e-6) ? (s - cum_lengths_[i]) / seg_len : 0.0;

  const auto pos = pa + (pb - pa) * frac;
  const double yaw = std::atan2(pb.Y() - pa.Y(), pb.X() - pa.X());

  model_->SetWorldPose(
    ignition::math::Pose3d(pos.X(), pos.Y(), z_, 0.0, 0.0, yaw));
  model_->SetLinearVel(ignition::math::Vector3d::Zero);
  model_->SetAngularVel(ignition::math::Vector3d::Zero);
}

}  // namespace gazebo
