// -*- lsst-c++ -*/
/**
 * @file verification_utils.h
 * @brief Defines utility functions for verification
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
#include <stdexcept>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include "safety_shield/motion.h"
#include "safety_shield/robot_reach.h"
#include "reach_lib.hpp"
#include "spdlog/spdlog.h"

#ifndef VERIFICATION_UTILS_H
#define VERIFICATION_UTILS_H

namespace safety_shield {

/**
 * @brief Check two capsules for collision
 *
 * @param[in] cap1 Capsule 1
 * @param[in] cap2 Capsule 2
 *
 * @returns true if capsules collide, false else
 */
inline bool capsuleCollisionCheck(const reach_lib::Capsule& cap1, const reach_lib::Capsule& cap2) {
  return reach_lib::intersections::capsule_capsule_intersection(cap1, cap2);
}

/**
 * @brief Check a set of robot capsules if they collide with a set of human capsules
 *
 * @param[in] robot_capsules The robot capsules
 * @param[in] human_capsules The human occupancy capsules
 *
 * @returns Whether a collision between any two capsules of the robot and human set occured
 */
bool robotHumanCollision(const std::vector<reach_lib::Capsule>& robot_capsules,
                          const std::vector<reach_lib::Capsule>& human_capsules);

/**
 * @brief For a given human capsule, find all robot capsules in contact and return their indices.
 * 
 * @param human_capsule Human capsule to check for contact with robot capsules.
 * @param robot_capsules List of robot capsules.
 * @returns List of indices of robot capsules in contact with the given human capsule.
 */
std::vector<int> findHumanRobotContact(const reach_lib::Capsule& human_capsule,
    const std::vector<reach_lib::Capsule>& robot_capsules);

/**
 * @brief For a given set of human capsule, find all robot capsules in contact and return their indices.
 * 
 * @param human_capsule Human capsule to check for contact with robot capsules.
 * @param robot_capsules List of robot capsules.
 * @returns Map that maps a list of robot link indices to the human capsule they are in contact with.
 *        The key is the human capsule index and the value is a list of robot link indices.
 */
std::map<int, std::vector<int>> findAllHumanRobotContacts(const std::vector<reach_lib::Capsule>& human_capsule,
    const std::vector<reach_lib::Capsule>& robot_capsules);

/**
 * @brief For a given human capsule, find all environment elements in contact and return their indices.
 * 
 * @param human_capsule Human capsule to check for contact with environment elements.
 * @param environment_elements List of environment elements.
 * @returns List of indices of environment elements in contact with the given human capsule.
 */
std::vector<int> findHumanEnvironmentContacts(const reach_lib::Capsule& human_capsule,
    const std::vector<reach_lib::AABB>& environment_elements);

/**
 * @brief Build two maps, which map robot and environment elements to their human capsule in contact.
 * 
 * @param[in] robot_capsules List of robot capsules.
 * @param[in] human_capsules List of human capsules.
 * @param[in] environment_elements List of environment elements.
 * @param[out] robot_collision_map Maps robot capsule indices to human capsule indices in contact.
 * @param[out] environment_collision_map Maps environment element indices to human capsule indices in contact.
 */
void buildContactMaps(const std::vector<reach_lib::Capsule>& robot_capsules, 
    const std::vector<reach_lib::Capsule>& human_capsules,
    const std::vector<reach_lib::AABB>& environment_elements,
    std::unordered_map<int, std::unordered_set<int>>& robot_collision_map,
    std::unordered_map<int, std::unordered_set<int>>& environment_collision_map);

/**
 * @brief Recursively build the graph of human body parts in contact with each other.
 * 
 * @param[in] current Index of the current capsule/body part.
 * @param[in] human_capsules List of human capsules.
 * @param[in] unclampable_body_part_map List of pairs of human body parts that cannot cause clamping because they are part of the same body chain.
 * @param[in, out] visited_body_parts Set of body parts already visited.
 * @param[out] human_contact_graph List of elements of the new combined body part.
 */
void buildHumanContactGraph(
    int current,
    const std::vector<reach_lib::Capsule>& human_capsules,
    const std::unordered_map<int, std::set<int>>& unclampable_body_part_map,
    std::unordered_set<int>& visited_body_parts,
    std::unordered_set<int>& human_contact_graph);

/**
 * @brief Check if a pair of links are unclampable according to the unclampable map.
 * 
 * @param link1 Link 1 
 * @param link2 Link 2
 * @param unclampable_enclosures_map List of pairs of robot links that cannot cause clamping. 
 * @return true if the two links can never cause clamping.
 * @return false if the two links can cause clamping.
 */
inline bool linkPairUnclampable(int link1, int link2, 
    const std::unordered_map<int, std::set<int>>& unclampable_enclosures_map) {
  return ((unclampable_enclosures_map.find(link1) != unclampable_enclosures_map.end() &&
      unclampable_enclosures_map.at(link1).find(link2) != unclampable_enclosures_map.at(link1).end()) ||
      (unclampable_enclosures_map.find(link2) != unclampable_enclosures_map.end() &&
      unclampable_enclosures_map.at(link2).find(link1) != unclampable_enclosures_map.at(link2).end()));
};

/**
 * @brief Create a capsule with a larger radius than the given capsule.
 * 
 * @param cap original capsule
 * @param additional_radius additional radius to add to the original capsule
 * @return reach_lib::Capsule expanded capsule
 */
inline reach_lib::Capsule createExpandedCapsule(reach_lib::Capsule cap, double additional_radius) {
  cap.r_ += additional_radius;
  return cap;
};

/**
 * @brief Check if a self-constrained collision (SCC) could occur with the given human capsule.
 * 
 * @param robot_collisions Robot capsule indices in contact with the given human capsule.
 * @param unclampable_enclosures_map List of pairs of robot links that cannot cause clamping.
 * @param robot_capsules List of robot capsules.
 * @param d_human Human diameter.
 * @return true if SCC could occur
 * @return false if SCC cannot occur
 */
bool selfConstrainedCollisionCheck(const std::vector<int>& robot_collisions,
    const std::unordered_map<int, std::set<int>>& unclampable_enclosures_map,
    const std::vector<reach_lib::Capsule>& robot_capsules,
    double d_human);

/**
 * @brief Calculate the normal vectors of the AABB surface pointing towards the robot capsule.
 * 
 * @param[in] robot_capsule Robot capsule.
 * @param[in] environment_element Environment element as axis-aligned bounding box.
 * @param[out] normals Normal vectors of the AABB surface pointing towards the robot capsule.
 * @return true calculation successful
 * @return false calculation failed
 */
bool calculateNormalVectors(const reach_lib::Capsule& robot_capsule,
    const reach_lib::AABB& environment_element,
    std::vector<Eigen::Vector3d>& normals);

/**
 * @brief Check if the given robot capsule is moving towards the given element in one time step.
 * 
 * @param velocity_capsule The velocity of the two points defining the robot capsule.
 * @param normal Normal vector of the element's surface pointing towards the robot capsule.
 * @param radius Radius of the robot capsule.
 * @param velocity_error Maximal error in velocity calculation.
 * @return true if the robot capsule could be moving towards the element.
 * @return false if the robot capsule is not moving towards the element.
 */
bool capsuleMovingTowardsElement(const RobotReach::CapsuleVelocity velocity_capsule,
    const Eigen::Vector3d& normal,
    double radius,
    double velocity_error);

/**
 * @brief Check if the given robot capsule is moving towards the given element over a sequence of time steps.
 * 
 * @param robot_capsule_velocities_start Pointer to the start of the list of robot capsule velocities.
 * @param robot_capsule_velocities_end Pointer to the end of the list of robot capsule velocities.
 * @param robot_capsules List of robot capsules.
 * @param capsule_index Index of the robot capsule to check.
 * @param normal Normal vector of the element's surface pointing towards the robot capsule.
 * @param velocity_error Maximal error in velocity calculation.
 * @return true 
 * @return false 
 */
bool capsuleTrajectoryMovingTowardsElement(
    const std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator& robot_capsule_velocities_start,
    const std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator& robot_capsule_velocities_end,
    const std::vector<reach_lib::Capsule>& robot_capsules,
    int capsule_index,
    const Eigen::Vector3d& normal,
    double velocity_error);

/**
 * @brief Check if an environmentally constrained collision (ECC) could occur with the given human capsule.
 * 
 * @param robot_collisions Robot capsule indices in contact with the given human capsule. 
 * @param robot_capsules List of robot capsules.
 * @param environment_collisions Environment element indices in contact with the given human capsule.
 * @param environment_elements List of environment elements.
 * @param d_human Human diameter.
 * @param robot_capsule_velocities_start Pointer to the start of the list of robot capsule velocities.
 * @param robot_capsule_velocities_end Pointer to the end of the list of robot capsule velocities.
 * @param velocity_errors upper bound of the velocity error for each joint
 * @return true if ECC could occur
 * @return false if ECC cannot occur
 */
bool environmentallyConstrainedCollisionCheck(const std::vector<int>& robot_collisions,
    const std::vector<reach_lib::Capsule>& robot_capsules,
    const std::vector<int>& environment_collisions,
    const std::vector<reach_lib::AABB>& environment_elements,
    double d_human,
    const std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator& robot_capsule_velocities_start,
    const std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator& robot_capsule_velocities_end,
    std::vector<double> velocity_errors);

/**
 * @brief Separate all robot-human collisions into constrained and unconstrained collisions.
 * 
 * @param[in] robot_capsules Reachable capsules of the robot
 * @param[in] human_capsules List of human reachable set capsules.
 * @param[in] environment_elements List of environment elements
 * @param[in] human_radii List of radii of human body parts/extremities.
 * @param[in] unclampable_body_part_map List of pairs of human body parts that cannot cause clamping because they are part of the same body chain.
 * @param[in] unclampable_enclosures_map List of pairs of robot links that cannot cause clamping.
 * @param[in] robot_capsule_velocities_it Iterator pointing to the beginning of the list of robot capsule velocities for this short-term plan.
 * @param[in] robot_capsule_velocities_end Iterator pointing to the end of the list of robot capsule velocities for this short-term plan.
 * @param[in] velocity_errors upper bound of the velocity error for each joint
 * @param[out] unconstrained_collisions List of unconstrained collisions. List of std::pair<human_capsule_index, robot_capsule_index>
 * @param[out] constrained_collisions List of constrained collisions. List of std::pair<human_capsule_index, robot_capsule_index>
 */
void separateConstrainedCollisions(
  const std::vector<reach_lib::Capsule>& robot_capsules, 
  const std::vector<reach_lib::Capsule>& human_capsules,
  const std::vector<reach_lib::AABB>& environment_elements,
  const std::vector<double>& human_radii,
  const std::unordered_map<int, std::set<int>>& unclampable_body_part_map,
  const std::unordered_map<int, std::set<int>>& unclampable_enclosures_map,
  std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_it,
  std::vector<std::vector<RobotReach::CapsuleVelocity>>::const_iterator robot_capsule_velocities_end,
  const std::vector<double>& velocity_errors,
  std::vector<std::pair<int, int>>& unconstrained_collisions,
  std::vector<std::pair<int, int>>& constrained_collisions
);

std::vector<double> calculateRobotLinkEnergies(
  const std::vector<double>& robot_link_velocities,
  const std::vector<double>& robot_link_reflected_masses
);

/**
 * @brief Check if any link that gets into contact with the human is too fast.
 * 
 * @param human_robot_contacts Maps the human capsule index to the list of robot link indices in contact.
 *  Key is the human capsule index, value is the list of robot link indices.
 * @param robot_link_energies Robot link energy at the time of contact.
 * @param maximal_contact_energies Maximal admissible human velocities.
 * @return true All robot links are slower than the maximal contact velocity.
 * @return false Otherwise.
 */
bool checkContactEnergySafety(
  const std::map<int, std::vector<int>>& human_robot_contacts,
  const std::vector<double>& robot_link_energies,
  const std::vector<double>& maximal_contact_energies
);

/**
 * @brief Check if any link that gets into contact with the human is too fast.
 * 
 * @param[in] collisions List of collision pairs <human_idx, robot_idx>
 * @param[in] robot_energies Robot link energies.
 * @param[in] max_contact_energies The maximal contact energy for each human body part. Size = [n_robot_links][n_human_models][n_human_bodies]
 * @param[in] human_motion_model_id The human motion model id.
 * @return true: All robot links are slower than the maximal contact velocity.
 * @return false: Otherwise.
 */
bool checkContactEnergySafetyIndividualLinks(
  const std::vector<std::pair<int, int>>& collisions,
  const std::vector<double>& robot_energies,
  const std::vector<std::vector<std::vector<double>>>& max_contact_energies,
  int human_motion_model_id
);

/**
 * @brief Check if any link that gets into contact with the human is too fast.
 * 
 * @param human_robot_contacts Maps the human capsule index to the list of robot link indices in contact.
 *  Key is the human capsule index, value is the list of robot link indices.
 * @param robot_link_velocities Robot link velocity at the time of contact.
 * @param maximal_contact_velocities Maximal admissible human velocities.
 * @return true All robot links are slower than the maximal contact velocity.
 * @return false Otherwise.
 */
bool checkVelocitySafety(
  const std::map<int, std::vector<int>>& human_robot_contacts,
  const std::vector<double>& robot_link_velocities,
  const std::vector<double>& maximal_contact_velocities
);

/**
 * @brief Calculate the maximal Cartesian robot link velocities for each time interval.
 * 
 * @param robot_motions The motion of the robot at each edge of the time intervals.
 * @param velocity_error The maximal velocity error for each robot link.
 * @return std::vector<std::vector<double>> Maximal Cartesian velocity of each robot link for each time interval.
 */
std::vector<std::vector<double>> calculateMaxRobotLinkVelocitiesPerTimeInterval(
  const std::vector<Motion>& robot_motions,
  const std::vector<double>& velocity_error
);

/**
 * @brief Calculate the maximal robot link energies for each time interval.
 * 
 * @param robot_link_velocities The maximal velocity of each robot link in each time interval.
 * @param robot_link_reflected_masses The maximal reflected masses of each robot link in each time interval.
 * @return std::vector<std::vector<double>> Maximal energy of each robot link for each time interval.
 */
std::vector<std::vector<double>> calculateMaxRobotEnergiesFromReflectedMasses(
  const std::vector<std::vector<double>>& robot_link_velocities,
  const std::vector<std::vector<double>>& robot_link_reflected_masses
);

/**
 * @brief Calculate the maximal robot link energies for each time interval.
 * 
 * @param robot_inertia_matrices The inertia matrices of the robot links in each time interval.
 * @param dq The joint velocities of the robot in each time interval.
 * @return std::vector<std::vector<double>> Maximal energy of each robot link for each time interval.
 */
std::vector<std::vector<double>> calculateMaxRobotEnergiesFromInertiaMatrices(
  const std::vector<std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>>& robot_inertia_matrices,
  std::vector<std::vector<double>> dq
);

}  // namespace safety_shield

#endif // VERIFICATION_UTILS_H