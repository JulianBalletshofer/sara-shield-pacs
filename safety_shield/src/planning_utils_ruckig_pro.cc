#include "safety_shield/planning_utils_ruckig_pro.h"

namespace safety_shield {

std::vector<Motion> trajectoryPlanningFromWaypoints(
    const Motion& start_motion,
    std::vector<std::vector<double>> waypoints,
    const std::vector<double>& v_max_allowed,
    const std::vector<double>& a_max_allowed,
    const std::vector<double>& j_max_allowed,
    const double sample_time,
    const double start_time) {

  const size_t DOFs = start_motion.getAngle().size();
  if (DOFs == 0) return {};

  if (waypoints.empty()) return {};

  // last waypoint is target, the rest are intermediates
  std::vector<double> target_position = waypoints.back();
  waypoints.pop_back();

  const size_t max_waypoints = std::max<size_t>(waypoints.size() + 1, 1);

  ruckig::Ruckig<ruckig::DynamicDOFs> planner(DOFs, sample_time, max_waypoints);
  ruckig::InputParameter<ruckig::DynamicDOFs> input(DOFs);
  ruckig::OutputParameter<ruckig::DynamicDOFs> output(DOFs);

  // Limits
  input.max_velocity     = v_max_allowed;
  input.max_acceleration = a_max_allowed;
  input.max_jerk         = j_max_allowed;

  // Start
  input.current_position     = start_motion.getAngle();
  input.current_velocity     = start_motion.getVelocity();
  input.current_acceleration = start_motion.getAcceleration();

  // Target
  input.target_position     = target_position;
  input.target_velocity     = std::vector<double>(DOFs, 0.0);
  input.target_acceleration = std::vector<double>(DOFs, 0.0);

  // Intermediate waypoints
  input.intermediate_positions = waypoints;

  input.interrupt_calculation_duration = 5000;

  const auto result = planner.calculate(input, output.trajectory);
  if (result != ruckig::Result::Working && result != ruckig::Result::Finished) {
    spdlog::error("Trajectory generation failed with result: {}", static_cast<int>(result));
    return {};
  }

  const double total_time = output.trajectory.get_duration();
  const size_t num_samples = static_cast<size_t>(std::ceil(total_time / sample_time)) + 1;

  std::vector<Motion> planned_trajectory;
  planned_trajectory.reserve(num_samples);

  std::vector<double> q(DOFs), dq(DOFs), ddq(DOFs), dddq(DOFs);
  size_t new_section = 0;
  double t_ruckig = 0.0;
  double t_sample = start_time;

  for (size_t i = 0; i < num_samples; ++i) {
    output.trajectory.at_time(t_ruckig, q, dq, ddq, dddq, new_section);

    planned_trajectory.emplace_back(
        t_sample, q, dq, ddq, dddq, t_sample);

    t_sample += sample_time;
    t_ruckig += sample_time;
  }

  return planned_trajectory;
}
}  // namespace safety_shield