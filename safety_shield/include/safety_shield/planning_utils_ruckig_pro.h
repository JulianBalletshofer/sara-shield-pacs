// -*- lsst-c++ -*/
/**
 * @file planning_utils_ruckig_pro.h
 * @brief Defines utility functions for planning of trajectories
 * @version 0.1
 * @copyright This file is part of SaRA-Shield.
 * SaRA-Shield is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 * SaRA-Shield is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with SaRA-Shield.
 * If not, see <https://www.gnu.org/licenses/>.
 */

#include <Eigen/Dense>
#include <ruckig/ruckig.hpp>
#include <ruckig/trackig.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "safety_shield/motion.h"

#ifndef PLANNING_UTILS_RUCKIG_PRO_H
#define PLANNING_UTILS_RUCKIG_PRO_H

namespace safety_shield {

/**
 * @brief Computes a trajectory from the given motion to a set of waypoints using ruckig.
 * @param start_motion The starting motion of the trajectory.
 * @param waypoints The waypoints to which the trajectory should be planned.
 * @param v_max_allowed The maximum allowed velocities for each joint.
 * @param a_max_allowed The maximum allowed accelerations for each joint.
 * @param j_max_allowed The maximum allowed jerks for each joint.
 * @param sample_time The sample time of the trajectory.
 * @param start_time The time at which the trajectory starts.
 * @returns planned trajectory as a vector of motions.
 */
std::vector<Motion> trajectoryPlanningFromWaypoints(
  const Motion& start_motion, 
  std::vector<std::vector<double>> waypoints,
  const std::vector<double>& v_max_allowed,
  const std::vector<double>& a_max_allowed,
  const std::vector<double>& j_max_allowed, 
  const double sample_time,
  const double start_time);
  
/**
 * Helper function to check waypoint calculation. 
 */
template <size_t DOFs>
inline bool is_close_to_waypoint(const std::array<double, DOFs>& position,
                          const std::array<double, DOFs>& waypoint,
                          double tolerance = 1e-2) {
    for (size_t i = 0; i < 7; ++i) {
        if (std::fabs(position[i] - waypoint[i]) > tolerance) {
            return false;
        }
    }
    return true;
}

}  // namespace safety_shield
#endif  // PLANNING_UTILS_RUCKIG_PRO_H