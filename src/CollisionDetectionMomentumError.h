/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <Eigen/src/Core/Matrix.h>
#include "LpfThreshold.h"

#include <RBDyn/Coriolis.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>

namespace mc_plugin
{

struct CollisionDetectionMomentumError : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController & controller) override;

  void after(mc_control::MCGlobalController & controller) override;

  void computeGammaAndMomentum(mc_control::MCGlobalController & controller);

  void addGui(mc_control::MCGlobalController & controller);
  void addPlot(mc_control::MCGlobalController & controller);
  void addLog(mc_control::MCGlobalController & controller);

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~CollisionDetectionMomentumError() override;

private:

  int jointNumber;
  double dt;
  double counter = 0.0;

  Eigen::VectorXd momentum; // M qd
  Eigen::VectorXd gamma; // tau -g +c^t qd
  Eigen::VectorXd momentum_hat;
  Eigen::VectorXd momentum_hat_dot;
  Eigen::VectorXd tau_ext_hat;
  Eigen::VectorXd tau_ext_hat_dot;
  Eigen::VectorXd tau;
  Eigen::VectorXd momentum_error;

  double alpha_1 = 40.0;
  double alpha_2 = 400.0;

  rbd::Coriolis * coriolis;
  rbd::ForwardDynamics forwardDynamics;

  LpfThreshold lpf_threshold_;
  double threshold_offset_ = 3.0;
  double threshold_filtering_ = 0.005;
  Eigen::VectorXd momentum_error_high_;
  Eigen::VectorXd momentum_error_low_;

  int jointShown = 0;
  bool activate_plot_ = false;
  bool plot_added_ = false;
  bool collision_stop_activated_ = false;
  bool obstacle_detected_ = false;
  bool activate_verbose = false;

};

} // namespace mc_plugin
