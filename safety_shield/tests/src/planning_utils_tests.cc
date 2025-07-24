#include <gtest/gtest.h>

#include "safety_shield/motion.h"
#include "safety_shield/planning_utils.h"

using namespace safety_shield;

TEST(PlanningUtilsTest, GoalTrajectoryWithZeroStartAcceleration) {
  int nb_joints = 6;
  double sample_time = 0.01;
  double start_time = 0.0;

  Motion start_motion(0.0, {0.2, -0.3, 0.1, -0.4, 0.5, -0.2}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  Motion goal_motion(1.0, {0.8, 0.2, 0.6, 0.0, 0.7, 0.3}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                     {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  std::vector<double> v_max(6, 1.0);
  std::vector<double> a_max(6, 10.0);
  std::vector<double> j_max(6, 400.0);

  auto traj = computeGoalTrajectory(start_motion, goal_motion, v_max, a_max, j_max, sample_time, start_time);

  ASSERT_FALSE(traj.empty());

  for (int i = 0; i < nb_joints; ++i) {
    EXPECT_NEAR(traj.front().getAngle()[i], start_motion.getAngle()[i], 1e-6);
    EXPECT_NEAR(traj.front().getVelocity()[i], 0.0, 1e-4);
    EXPECT_NEAR(traj.front().getAcceleration()[i], 0.0, 1e-4);
  }

  const Motion& last = traj.back();
  for (int i = 0; i < nb_joints; ++i) {
    EXPECT_NEAR(last.getAngle()[i], goal_motion.getAngle()[i], 1e-3);
    EXPECT_NEAR(last.getVelocity()[i], 0.0, 1e-3);
    EXPECT_NEAR(last.getAcceleration()[i], 0.0, 1e-3);
  }
}

TEST(PlanningUtilsTest, GoalTrajectoryWithNonZeroStartAcceleration) {
  int nb_joints = 6;
  double sample_time = 0.01;
  double start_time = 0.0;

  Motion start_motion(0.0, {0.5, -0.1, 0.2, -0.3, 0.1, 0.0}, {0.2, -0.2, 0.1, -0.1, 0.05, -0.05},
                      {0.05, 0.05, -0.03, -0.04, 0.02, -0.01});

  Motion goal_motion(1.0, {1.0, 0.3, 0.7, 0.2, 0.4, 0.1}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                     {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  std::vector<double> v_max(6, 1.0);
  std::vector<double> a_max(6, 10.0);
  std::vector<double> j_max(6, 400.0);

  auto traj = computeGoalTrajectory(start_motion, goal_motion, v_max, a_max, j_max, sample_time, start_time);
  ASSERT_FALSE(traj.empty());

  for (int i = 0; i < nb_joints; ++i) {
    EXPECT_NEAR(traj.front().getAngle()[i], start_motion.getAngle()[i], 1e-6);
    EXPECT_NEAR(traj.front().getVelocity()[i], start_motion.getVelocity()[i], 1e-6);
    EXPECT_NEAR(traj.front().getAcceleration()[i], start_motion.getAcceleration()[i], 1e-6);
  }

  const Motion& last = traj.back();
  for (int i = 0; i < nb_joints; ++i) {
    EXPECT_NEAR(last.getAngle()[i], goal_motion.getAngle()[i], 1e-3);
    EXPECT_NEAR(last.getVelocity()[i], 0.0, 1e-5);
    EXPECT_NEAR(last.getAcceleration()[i], 0.0, 1e-5);
  }
}

TEST(PlanningUtilsTest, GoalTrajectoryWithNonZeroGoalVelAcc) {
  int nb_joints = 6;
  double sample_time = 0.01;
  double start_time = 0.0;

  Motion start_motion(0.0, {0.1, -0.2, 0.0, 0.3, -0.1, 0.2}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  Motion goal_motion(1.0, {0.9, 0.4, 0.7, 0.6, 0.3, 0.5}, {0.3, 0.2, 0.1, 0.1, 0.15, 0.05},
                     {0.1, 0.05, 0.02, 0.03, 0.04, 0.02});

  std::vector<double> v_max(6, 1.0);
  std::vector<double> a_max(6, 10.0);
  std::vector<double> j_max(6, 400.0);

  auto traj = computeGoalTrajectory(start_motion, goal_motion, v_max, a_max, j_max, sample_time, start_time);
  ASSERT_FALSE(traj.empty());

  const Motion& last = traj.back();
  for (int i = 0; i < nb_joints; ++i) {
    EXPECT_NEAR(last.getAngle()[i], goal_motion.getAngle()[i], 1e-2);
    EXPECT_NEAR(last.getVelocity()[i], goal_motion.getVelocity()[i], 1e-3);
    EXPECT_NEAR(last.getAcceleration()[i], goal_motion.getAcceleration()[i], 1e-3);
  }
}

TEST(PlanningUtilsTest, StoppingTrajectoryFinalZero) {
  int nb_joints = 6;
  double sample_time = 0.01;
  double start_time = 0.0;

  std::vector<double> v_max(6, 1.0);
  std::vector<double> a_max(6, 10.0);
  std::vector<double> j_max(6, 400.0);

  std::vector<Motion> test_starts = {Motion(0.0, {0.2, 0.5, -0.3, 0.4, -0.2, 0.0}, {0.5, -0.3, 0.2, -0.2, 0.1, 0.0},
                                            {0.1, -0.1, 0.05, 0.0, -0.05, 0.02}),
                                     Motion(0.0, {-0.1, 0.3, 0.1, -0.4, 0.5, 0.2}, {0.3, 0.2, -0.4, 0.0, 0.1, -0.2},
                                            {0.0, 0.1, -0.1, 0.02, -0.02, 0.03})};

  for (const auto& start_motion : test_starts) {
    auto traj = computeStoppingTrajectory(start_motion, v_max, a_max, j_max, sample_time, start_time);
    ASSERT_FALSE(traj.empty());

    const Motion& last = traj.back();
    for (int i = 0; i < nb_joints; ++i) {
      EXPECT_NEAR(last.getVelocity()[i], 0.0, 1e-4);
      EXPECT_NEAR(last.getAcceleration()[i], 0.0, 1e-4);
    }
  }
}

TEST(ConvertTrajectoryTest, MatchesStartAndEndStateAndDuration6DOF) {
  int nb_joints = 6;
  double sample_time = 0.01;
  double start_time = 0.0;

  // Setup Ruckig planner
  ruckig::InputParameter<ruckig::DynamicDOFs> input(nb_joints);
  ruckig::OutputParameter<ruckig::DynamicDOFs> output(nb_joints);
  ruckig::Ruckig<ruckig::DynamicDOFs> planner(nb_joints, sample_time);

  // Set input for 6 joints
  input.current_position = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  input.current_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  input.current_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  input.target_position = {1.0, 0.9, 0.8, 0.7, 0.6, 0.5};
  input.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  input.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  input.max_velocity = std::vector<double>(nb_joints, 1.0);
  input.max_acceleration = std::vector<double>(nb_joints, 2.0);
  input.max_jerk = std::vector<double>(nb_joints, 10.0);

  ASSERT_EQ(planner.calculate(input, output.trajectory), ruckig::Result::Working);

  // Get reference start and end states
  const double duration = output.trajectory.get_duration();
  std::vector<double> q(nb_joints), dq(nb_joints), ddq(nb_joints), dddq(nb_joints);
  size_t section = 0;

  output.trajectory.at_time(0.0, q, dq, ddq, dddq, section);
  Motion reference_start(start_time, q, dq, ddq, dddq, start_time);

  output.trajectory.at_time(duration, q, dq, ddq, dddq, section);
  Motion reference_end(duration, q, dq, ddq, dddq, duration);

  // Convert to motion sequence
  auto motions = convertTrajectory(output, nb_joints, sample_time, start_time);
  ASSERT_GE(motions.size(), 2);

  const Motion& start = motions.front();
  const Motion& end = motions.back();

  // Validate start state
  for (int i = 0; i < nb_joints; ++i) {
    EXPECT_NEAR(start.getAngle()[i], reference_start.getAngle()[i], 1e-6);
    EXPECT_NEAR(start.getVelocity()[i], reference_start.getVelocity()[i], 1e-6);
    EXPECT_NEAR(start.getAcceleration()[i], reference_start.getAcceleration()[i], 1e-6);
  }

  // Validate end state
  for (int i = 0; i < nb_joints; ++i) {
    EXPECT_NEAR(end.getAngle()[i], reference_end.getAngle()[i], 1e-6);
    EXPECT_NEAR(end.getVelocity()[i], reference_end.getVelocity()[i], 1e-6);
    EXPECT_NEAR(end.getAcceleration()[i], reference_end.getAcceleration()[i], 1e-6);
  }

  // Validate timing
  EXPECT_NEAR(start.getTime(), start_time, start_time / 100);
  EXPECT_NEAR(end.getTime(), duration + start_time, sample_time / 100);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}