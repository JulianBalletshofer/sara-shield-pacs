#include "safety_shield/planning_utils.h"

namespace safety_shield {

std::vector<Motion> trajectoryPlanningRuckig(ruckig::InputParameter<ruckig::DynamicDOFs> planner_settings,
                                             const Motion& start_motion, const int nb_joints,
                                             const std::vector<double>& v_max_allowed,
                                             const std::vector<double>& a_max_allowed,
                                             const std::vector<double>& j_max_allowed, const double sample_time,
                                             const double start_time) {
  // Create Ruckig trajectory planner
  ruckig::Ruckig<ruckig::DynamicDOFs> planner(nb_joints, sample_time);
  ruckig::OutputParameter<ruckig::DynamicDOFs> output(nb_joints);
  planner_settings.current_position = start_motion.getAngle();
  planner_settings.current_velocity = start_motion.getVelocity();
  planner_settings.current_acceleration = start_motion.getAcceleration();
  planner_settings.max_velocity = v_max_allowed;
  planner_settings.max_acceleration = a_max_allowed;
  planner_settings.max_jerk = j_max_allowed;
  // Validate input parameters
  try {
    planner.validate_input(planner_settings, false, true);
  } catch (const std::exception& e) {
    throw std::runtime_error(std::string("Ruckig input validation failed: ") + e.what());
  }
  // Calculate the trajectory
  auto result = planner.calculate(planner_settings, output.trajectory);
  if (result != ruckig::Result::Working) {
    throw std::runtime_error("Ruckig trajectory planning failed with result: " +
                             std::to_string(static_cast<int>(result)));
  }
  // Convert the output trajectory to a vector of Motion objects
  std::vector<Motion> planned_trajectory = convertTrajectory(output, nb_joints, sample_time, start_time);
  return planned_trajectory;
}

std::vector<Motion> computeGoalTrajectory(const Motion& start_motion, const Motion& goal_motion,
                                          const std::vector<double>& v_max_allowed,
                                          const std::vector<double>& a_max_allowed,
                                          const std::vector<double>& j_max_allowed, const double sample_time,
                                          const double start_time) {
  int nb_joints = start_motion.getAngle().size();
  // Create Ruckig planner settings for goal motion
  ruckig::InputParameter<ruckig::DynamicDOFs> planner_settings(nb_joints);
  planner_settings.target_position = goal_motion.getAngle();
  planner_settings.target_velocity = goal_motion.getVelocity();
  planner_settings.target_acceleration = goal_motion.getAcceleration();
  std::vector<Motion> planned_trajectory = trajectoryPlanningRuckig(
      planner_settings, start_motion, nb_joints, v_max_allowed, a_max_allowed, j_max_allowed, sample_time, start_time);
  return planned_trajectory;
}

std::vector<Motion> computeStoppingTrajectory(const Motion& start_motion, const std::vector<double>& v_max_allowed,
                                              const std::vector<double>& a_max_allowed,
                                              const std::vector<double>& j_max_allowed, const double sample_time,
                                              const double start_time) {
  // Create Ruckig settings for failsafe trajectory using velocity control
  int nb_joints = start_motion.getAngle().size();
  ruckig::InputParameter<ruckig::DynamicDOFs> planner_settings(nb_joints);
  planner_settings.control_interface = ruckig::ControlInterface::Velocity;
  // planner_settings.synchronization  = ruckig::Synchronization::Time; // dicuss whether we want this
  planner_settings.target_velocity = std::vector<double>(nb_joints, 0.0);
  planner_settings.target_acceleration = std::vector<double>(nb_joints, 0.0);
  // Convert the output trajectory to a vector of Motion objects
  std::vector<Motion> failsafe_trajectory = trajectoryPlanningRuckig(
      planner_settings, start_motion, nb_joints, v_max_allowed, a_max_allowed, j_max_allowed, sample_time, start_time);
  return failsafe_trajectory;
}

std::vector<Motion> convertTrajectory(ruckig::OutputParameter<ruckig::DynamicDOFs>& output, const int nb_joint,
                                      const double sample_time, const double start_time) {
  // Convert the output trajectory to a vector of Motion objects
  const double total_time = output.trajectory.get_duration();
  size_t num_samples = static_cast<size_t>(std::ceil(total_time / sample_time)) + 1;  // since time 0 also considered

  std::vector<Motion> planned_trajectory(num_samples);
  std::vector<double> q(nb_joint);
  std::vector<double> dq(nb_joint);
  std::vector<double> ddq(nb_joint);
  std::vector<double> dddq(nb_joint);

  size_t new_section = 0;
  double t_ruckig = 0.0;
  double t_sample = start_time;  // if we compute stopping trajectory, we want to start at first sample time

  for (size_t i = 0; i < num_samples; i++) {
    output.trajectory.at_time(t_ruckig, q, dq, ddq, dddq, new_section);
    planned_trajectory[i] = Motion(t_sample, q, dq, ddq, dddq, t_sample);
    t_sample += sample_time;
    t_ruckig += sample_time;
  }
  return planned_trajectory;
}
}  // namespace safety_shield