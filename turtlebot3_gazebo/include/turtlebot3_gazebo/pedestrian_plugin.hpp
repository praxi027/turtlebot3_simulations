// Copyright 2026 TurtleBot3 PPO benchmark authors.
//
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef TURTLEBOT3_GAZEBO__PEDESTRIAN_PLUGIN_HPP_
#define TURTLEBOT3_GAZEBO__PEDESTRIAN_PLUGIN_HPP_

#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector2.hh>

namespace gazebo
{

// Drives a model along a closed loop of (x, y) waypoints at constant speed.
// Used for pedestrian-style obstacles in the warehouse_v3 benchmark — the
// model is a regular <model> with a collision body, so the lidar can see it.
class PedestrianPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  void OnUpdate(const common::UpdateInfo & _info);

  physics::ModelPtr model_;
  event::ConnectionPtr update_conn_;
  std::vector<ignition::math::Vector2d> waypoints_;
  std::vector<double> cum_lengths_;   // length up to start of segment i
  double total_length_ = 0.0;
  double speed_ = 0.6;                 // m/s
  double z_ = 0.0;                     // base height
  common::Time start_time_;
};

GZ_REGISTER_MODEL_PLUGIN(PedestrianPlugin)
}  // namespace gazebo

#endif  // TURTLEBOT3_GAZEBO__PEDESTRIAN_PLUGIN_HPP_
