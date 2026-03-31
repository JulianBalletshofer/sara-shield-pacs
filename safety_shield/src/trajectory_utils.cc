#include "safety_shield/trajectory_utils.h"

#include "spdlog/spdlog.h"
#include "spdlog/fmt/fmt.h"

namespace safety_shield {

std::vector<double> calcTimePointsForEquidistantIntervals(double begin, double end, double reachability_set_duration) {
  std::vector<double> time_points;
  double current_time = begin;
  while (current_time < end) {
    time_points.push_back(current_time);
    // test bugfix of adding the last time point twice - only issue when using non path consistent
    if (std::abs(current_time - end) < 1e-7) {
      return time_points;
    }
    current_time += reachability_set_duration;
  }
  time_points.push_back(end);
  return time_points;
}

std::vector<Motion> getMotionsOfAllTimeStepsFromPath(const LongTermTraj& ltt, const Path& path, const std::vector<double>& time_points) {
  Path path_copy = path;
  std::vector<Motion> motion_samples;
  motion_samples.push_back(ltt.interpolate(path_copy.getPosition(), path_copy.getVelocity(), path_copy.getAcceleration(), path_copy.getJerk()));
  std::string path_pos_log;
  for (int i = 0; i < time_points.size() - 1; i++) {
    // increment path and push motion to list
    double time_interval_duration = time_points[i + 1] - time_points[i];
    path_copy.increment(time_interval_duration);
    path_pos_log += fmt::format("{:.4f} ", path_copy.getPosition());
    Motion motion = ltt.interpolate(path_copy.getPosition(), path_copy.getVelocity(), path_copy.getAcceleration(), path_copy.getJerk());
    // interpolate ignores time of motion
    motion.setTime(time_points[i+1]);
    motion_samples.push_back(motion);
  }
  std::string q0_log;
  for (const auto& m : motion_samples) q0_log += fmt::format("{:.4f} ", m.getAngle()[0]);
  spdlog::info("getMotionsOfAllTimeStepsFromPath path positions: [{}], q[0]: [{}]", path_pos_log, q0_log);
  return motion_samples;
}

std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>> getInertiaMatricesOfAllTimeStepsFromPath(
  const LongTermTraj& ltt, const Path& path, const std::vector<double>& time_points
) {
  Path path_copy = path;
  std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>> inertia_matrices;
  inertia_matrices.push_back(ltt.getInertiaMatrices(ltt.getLowerIndex(path_copy.getPosition())));
  for (int i = 0; i < time_points.size() - 1; i++) {
    // increment path and push motion to list
    double time_interval_duration = time_points[i + 1] - time_points[i];
    path_copy.increment(time_interval_duration);
    inertia_matrices.push_back(ltt.getInertiaMatrices(ltt.getLowerIndex(path_copy.getPosition())));
  }
  return inertia_matrices;
}
}