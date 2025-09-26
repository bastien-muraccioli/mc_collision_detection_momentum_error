#include "CollisionDetectionMomentumError.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

CollisionDetectionMomentumError::~CollisionDetectionMomentumError() = default;

void CollisionDetectionMomentumError::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{

  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);

  auto & robot = ctl.robot(ctl.robots()[0].name());
  auto & realRobot = ctl.realRobot(ctl.robots()[0].name());
  auto & rjo = robot.refJointOrder();

  dt = ctl.timestep();

  jointNumber = ctl.robot(ctl.robots()[0].name()).refJointOrder().size();

  // Make sure to have obstacle detection
  if(!ctl.controller().datastore().has("Obstacle detected"))
  {
    ctl.controller().datastore().make<bool>("Obstacle detected", false);
  }

  momentum = Eigen::VectorXd::Zero(jointNumber);
  gamma = Eigen::VectorXd::Zero(jointNumber);
  momentum_hat = Eigen::VectorXd::Zero(jointNumber);
  momentum_hat_dot = Eigen::VectorXd::Zero(jointNumber);
  tau_ext_hat = Eigen::VectorXd::Zero(jointNumber);
  tau_ext_hat_dot = Eigen::VectorXd::Zero(jointNumber);
  tau = Eigen::VectorXd::Zero(jointNumber);

  Eigen::VectorXd qdot(jointNumber);
  for(size_t i = 0; i < jointNumber; i++)
  {
    qdot[i] = robot.alpha()[robot.jointIndexByName(rjo[i])][0];
  }

  coriolis = new rbd::Coriolis(robot.mb());
  forwardDynamics = rbd::ForwardDynamics(robot.mb());

  forwardDynamics.computeC(realRobot.mb(), realRobot.mbc());
  forwardDynamics.computeH(robot.mb(), robot.mbc());
  auto inertiaMatrix = forwardDynamics.H() - forwardDynamics.HIr();
  momentum = inertiaMatrix * qdot;

  tau = Eigen::VectorXd::Map(realRobot.jointTorques().data(), realRobot.jointTorques().size());

  auto coriolisMatrix = coriolis->coriolis(realRobot.mb(), realRobot.mbc());
  auto coriolisGravityTerm = forwardDynamics.C(); //C*qdot + g
  gamma = tau + (coriolisMatrix + coriolisMatrix.transpose()) * qdot - coriolisGravityTerm;

  momentum_hat = momentum;
  momentum_error = momentum - momentum_hat;
  momentum_error_high_ = Eigen::VectorXd::Zero(jointNumber);
  momentum_error_low_ = Eigen::VectorXd::Zero(jointNumber);

  lpf_threshold_.setValues(threshold_offset_, threshold_filtering_, jointNumber);

  addGui(ctl);
  addLog(ctl);

  mc_rtc::log::info("CollisionDetectionMomentumError::init called with configuration:\n{}", config.dump(true, true));
}

void CollisionDetectionMomentumError::computeGammaAndMomentum(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);

  if(ctl.robot().encoderVelocities().empty())
  {
    return;
  }

  auto & robot = ctl.robot();
  auto & realRobot = ctl.realRobot(ctl.robots()[0].name());

  Eigen::VectorXd qdot(jointNumber);
  rbd::paramToVector(realRobot.alpha(), qdot);
  forwardDynamics.computeC(realRobot.mb(), realRobot.mbc());
  forwardDynamics.computeH(robot.mb(), robot.mbc());
  auto inertiaMatrix = forwardDynamics.H() - forwardDynamics.HIr();
  momentum = inertiaMatrix * qdot;

  tau = Eigen::VectorXd::Map(realRobot.jointTorques().data(), realRobot.jointTorques().size());

  auto coriolisMatrix = coriolis->coriolis(realRobot.mb(), realRobot.mbc());
  auto coriolisGravityTerm = forwardDynamics.C(); //C*qdot + g
  gamma = tau + (coriolisMatrix + coriolisMatrix.transpose()) * qdot - coriolisGravityTerm;
}

void CollisionDetectionMomentumError::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("CollisionDetectionMomentumError::reset called");
}

void CollisionDetectionMomentumError::before(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  counter += dt;

  if(activate_plot_ && !plot_added_)
  {
    addPlot(controller);
    plot_added_ = true;
  }

  computeGammaAndMomentum(ctl);
  momentum_error = momentum - momentum_hat;
  momentum_hat_dot = gamma + tau_ext_hat + alpha_1*(momentum_error);
  tau_ext_hat_dot = alpha_2*(momentum_error);
  momentum_hat += momentum_hat_dot*dt;
  tau_ext_hat += tau_ext_hat_dot*dt;

  momentum_error_high_ = lpf_threshold_.adaptiveThreshold(tau_ext_hat_dot, true);
  momentum_error_low_ = lpf_threshold_.adaptiveThreshold(tau_ext_hat_dot, false);

  obstacle_detected_ = false;
  for (int i = 0; i < jointNumber; i++)
  {
    if (tau_ext_hat_dot[i] > momentum_error_high_[i] || tau_ext_hat_dot[i] < momentum_error_low_[i])
    {
      obstacle_detected_ = true;
      if(activate_verbose) mc_rtc::log::info("[Collision Detection Momentum Error] Obstacle detected on joint {}", i);
      if(collision_stop_activated_)
      {
        ctl.controller().datastore().get<bool>("Obstacle detected") = obstacle_detected_;
      }
        break;
    }
  }
  // mc_rtc::log::info("CollisionDetectionMomentumError::before");
}

void CollisionDetectionMomentumError::after(mc_control::MCGlobalController & controller)
{
  // mc_rtc::log::info("CollisionDetectionMomentumError::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration CollisionDetectionMomentumError::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = true;
  return out;
}

void CollisionDetectionMomentumError::addPlot(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & gui = *ctl.controller().gui();

  gui.addPlot(
      "CollisionDetectionMomentumError_momentum",
      mc_rtc::gui::plot::X(
          "t", [this]() { return counter; }),
      mc_rtc::gui::plot::Y(
          "momentum(t)", [this]() { return momentum[jointShown]; }, mc_rtc::gui::Color::Red),
      mc_rtc::gui::plot::Y(
          "momentum_hat(t)", [this]() { return momentum_hat[jointShown]; }, mc_rtc::gui::Color::Green)
      );

  gui.addPlot(
      "CollisionDetectionMomentumError_tau_ext_hat",
      mc_rtc::gui::plot::X(
          "t", [this]() { return counter; }),
      mc_rtc::gui::plot::Y(
          "tau_ext_hat(t)", [this]() { return tau_ext_hat[jointShown]; }, mc_rtc::gui::Color::Red)
      );

  gui.addPlot(
      "CollisionDetectionMomentumError_momentum_dot",
      mc_rtc::gui::plot::X(
          "t", [this]() { return counter; }),
      mc_rtc::gui::plot::Y(
          "momentum_dot(t)", [this]() { return momentum_hat_dot[jointShown]; }, mc_rtc::gui::Color::Red)
      );

  gui.addPlot(
    "CollisionDetectionMomentumError_momentum_error",
    mc_rtc::gui::plot::X(
        "t", [this]() { return counter; }),
        mc_rtc::gui::plot::Y(
        "momentum_error_high(t)", [this]() { return momentum_error_high_[jointShown]; }, mc_rtc::gui::Color::Gray),
        mc_rtc::gui::plot::Y(
        "momentum_error_low(t)", [this]() { return momentum_error_low_[jointShown]; }, mc_rtc::gui::Color::Gray),
    mc_rtc::gui::plot::Y(
        "momentum_error(t)", [this]() { return tau_ext_hat_dot[jointShown]; }, mc_rtc::gui::Color::Red)
    );
}

void CollisionDetectionMomentumError::addLog(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  ctl.controller().logger().addLogEntry("CollisionDetectionMomentumError_momentum", [this]() { return momentum; });
  ctl.controller().logger().addLogEntry("CollisionDetectionMomentumError_momentum_hat", [this]() { return momentum_hat; });
  ctl.controller().logger().addLogEntry("CollisionDetectionMomentumError_momentum_hat_dot", [this]() { return momentum_hat_dot; });
  ctl.controller().logger().addLogEntry("CollisionDetectionMomentumError_tau_ext_hat", [this]() { return tau_ext_hat; });
  ctl.controller().logger().addLogEntry("CollisionDetectionMomentumError_tau_ext_hat_dot", [this]() { return tau_ext_hat_dot; });
  ctl.controller().logger().addLogEntry("CollisionDetectionMomentumError_gamma", [this]() { return gamma; });
  ctl.controller().logger().addLogEntry("CollisionDetectionMomentumError_momentum_error", [this]() { return momentum_error; });
  ctl.controller().logger().addLogEntry("CollisionDetectionMomentumError_momentum_error_high", [this]() { return momentum_error_high_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionMomentumError_momentum_error_low", [this]() { return momentum_error_low_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionMomentumError_obstacleDetected", [this]() { return obstacle_detected_; });
}

void CollisionDetectionMomentumError::addGui(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & gui = *ctl.controller().gui();

  ctl.controller().gui()->addElement({"Plugins", "CollisionDetectionMomentumError"},
    mc_rtc::gui::Button("Add plot", [this]() { return activate_plot_ = true; }),
    // Add checkbox to activate the collision stop
    mc_rtc::gui::Checkbox("Collision stop", collision_stop_activated_),
    mc_rtc::gui::Checkbox("Verbose", activate_verbose), 
    // Add Threshold offset input
    mc_rtc::gui::NumberInput("Threshold offset", [this](){return this->threshold_offset_;},
        [this](double offset)
      { 
        threshold_offset_ = offset;
        lpf_threshold_.setOffset(threshold_offset_); 
      }),
    // Add Threshold filtering input
    mc_rtc::gui::NumberInput("Threshold filtering", [this](){return this->threshold_filtering_;},
        [this](double filtering)
      { 
        threshold_filtering_ = filtering;
        lpf_threshold_.setFiltering(threshold_filtering_); 
      })                                                                         
    );

  gui.addElement({"Plugins", "CollisionDetectionMomentumError"},
      mc_rtc::gui::NumberInput(
          "alpha_1", [this]() { return alpha_1; },
          [this](double alpha)
          {
            this->alpha_1 = alpha;
          }));

  gui.addElement({"Plugins", "CollisionDetectionMomentumError"},
      mc_rtc::gui::NumberInput(
          "alpha_2", [this]() { return alpha_2; },
          [this](double alpha)
          {
            this->alpha_2 = alpha;
          }));

  gui.addElement({"Plugins", "CollisionDetectionMomentumError"},
      mc_rtc::gui::IntegerInput(
          "jointShown", [this]() { return jointShown; },
          [this](int joint)
          {
            this->jointShown = joint;
          }));
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("CollisionDetectionMomentumError", mc_plugin::CollisionDetectionMomentumError)
