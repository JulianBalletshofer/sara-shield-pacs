// -*- lsst-c++ -*/
/**
 * @file planning_utils.h
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
#include <stdexcept>
#include <string>
#include <vector>

#include "safety_shield/motion.h"

#ifndef PLANNING_UTILS_H
#define PLANNING_UTILS_H

namespace safety_shield {

/**
 * @brief Computes a trajectory from the given motion to a goal motion or to a full stop based on the input conditions
 * using ruckig.
 * @param planner_settings The input parameters for the ruckig planner.
 * @param start_motion The starting motion of the trajectory.
 * @param nb_joints The number of joints in the robot.
 * @param v_max_allowed The maximum allowed velocities for each joint.
 * @param a_max_allowed The maximum allowed accelerations for each joint.
 * @param j_max_allowed The maximum allowed jerks for each joint.
 * @param sample_time The sample time of the trajectory.
 * @param start_time The time at which the trajectory starts.
 * @returns planned trajectory as a vector of motions.
 */
std::vector<Motion> trajectoryPlanningRuckig(ruckig::InputParameter<ruckig::DynamicDOFs> planner_settings,
                                             const Motion& start_motion, const int nb_joints,
                                             const std::vector<double>& v_max_allowed,
                                             const std::vector<double>& a_max_allowed,
                                             const std::vector<double>& j_max_allowed, const double sample_time,
                                             const double start_time = 0.0);

/**
 * @brief Computes a trajectory from the given motion to the goal motion using ruckig.
 * @param start_motion The starting motion of the trajectory.
 * @param goal_motion The goal motion of the trajectory.
 * @param v_max_allowed The maximum allowed velocities for each joint.
 * @param a_max_allowed The maximum allowed accelerations for each joint.
 * @param j_max_allowed The maximum allowed jerks for each joint.
 * @param sample_time The sample time of the trajectory.
 * @param start_time The time at which the trajectory starts.
 * @returns planned trajectory as a vector of motions.
 */
std::vector<Motion> computeGoalTrajectory(const Motion& start_motion, const Motion& goal_motion,
                                          const std::vector<double>& v_max_allowed,
                                          const std::vector<double>& a_max_allowed,
                                          const std::vector<double>& j_max_allowed, const double sample_time,
                                          const double start_time = 0.0);

/**
 * @brief Computes a failsafe trajectory from the given motion to a full stop of the robot using ruckig.
 * @details This function computes a failsafe trajectory that is not path-consistent using velocity control.
 * @param start_motion The starting motion of the trajectory.
 * @param v_max_allowed The maximum allowed velocities for each joint.
 * @param a_max_allowed The maximum allowed accelerations for each joint.
 * @param j_max_allowed The maximum allowed jerks for each joint.
 * @param sample_time The sample time of the trajectory.
 * @param start_time The time at which the trajectory starts.
 * @returns failsafe trajectory as a vector of motions.
 */
std::vector<Motion> computeStoppingTrajectory(const Motion& start_motion, const std::vector<double>& v_max_allowed,
                                              const std::vector<double>& a_max_allowed,
                                              const std::vector<double>& j_max_allowed, const double sample_time,
                                              const double start_time = 0.0);

/**
 * @brief builds trajectory of motions from the computed trajectory .
 * @details This function converts the computed trajectory into a vector of motions starting with time start_time.
 * @param output The ruckig object containing the computed trajectory.
 * @param nb_joint The number of joints in the robot.
 * @param sample_time The sample time of the trajectory.
 * @param start_time The time at which the trajectory starts.
 * @returns trajectory as a vector of motions.
 */
std::vector<Motion> convertTrajectory(ruckig::OutputParameter<ruckig::DynamicDOFs>& output, const int nb_joint,
                                      const double sample_time, const double start_time = 0.0);

}  // namespace safety_shield

#endif  // PLANNING_UTILS_H