#include <gtest/gtest.h>
#include <math.h>

#include <Eigen/Dense>

#include "safety_shield/exceptions.h"
#include "safety_shield/safety_shield.h"
#include "safety_shield_fixture.h"
#include "spdlog/spdlog.h"

namespace safety_shield {

void throwRobotMovementException() {
  try {
    throw RobotMovementException();
  } catch (RobotMovementException e) {
    std::cout << e.what() << std::endl;
  }
}

void throwTrajectoryException(std::string msg) {
  try {
    throw TrajectoryException(msg);
  } catch (TrajectoryException e) {
    std::cout << e.what() << std::endl;
  }
}

TEST(SafetyShieldExceptionTest, ExpectionTest) {
  EXPECT_NO_THROW(throwRobotMovementException());
  EXPECT_NO_THROW(throwTrajectoryException("Test"));
}

TEST_F(SafetyShieldTest, InitializationTest) {
  EXPECT_DOUBLE_EQ(0, 0.0);
}

TEST_F(SafetyShieldTest, CalculateMaxAccJerkTest) {
  std::vector<double> prev_speed = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> a_max_part = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
  std::vector<double> j_max_part = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
  // max_s_stop: 0.2
  // a_max_allowed: [10, 10, 10, 10, 10, 10]
  // j_max_allowed: [400, 400, 400, 400, 400, 400]
  double a_max_manoeuvre, j_max_manoeuvre;
  shield_->calculateMaxAccJerk(prev_speed, a_max_part, j_max_part, a_max_manoeuvre, j_max_manoeuvre);
  EXPECT_NEAR(a_max_manoeuvre, (10.0 - 2.0) / (1.0 + 2.0 * 0.2), 1e-5);
  EXPECT_NEAR(j_max_manoeuvre, (400.0 - 10.0 - 3 * 2.0 * a_max_manoeuvre) / (1.0 + 2.0 * 0.2), 1e-5);
}

TEST_F(SafetyShieldTest, PlanSafetyShieldTest) {
  double pos = 0;
  double vel = 0;
  double acc = 0;
  double ve = 1;
  double a_max = 10;
  double j_max = 100;
  double final_pos, final_vel, final_acc;
  bool success;
  safety_shield::Path path;
  for (int i = 0; i < 11; i++) {
    vel = double(i) / 10.0;
    for (int j = -40; j < 40; j++) {
      acc = double(j) / 5.0;
      double t_to_a_0 = ceil((abs(acc) / j_max) / shield_->getSampleTime()) * shield_->getSampleTime();
      if (vel + acc * t_to_a_0 / 2 < 0) {
        continue;
      }
      for (int k = 0; k < 101; k++) {
        ve = double(k) / 100.0;
        success = shield_->planSafetyShield(pos, vel, acc, ve, a_max, j_max, path);
        ASSERT_TRUE(success);
        path.getFinalMotion(final_pos, final_vel, final_acc);
        ASSERT_NEAR(final_vel, ve, 1e-6);
        ASSERT_NEAR(final_acc, 0.0, 1e-2);
      }
    }
  }
}

TEST_F(SafetyShieldTest, ComputeMonitoredTrajectoryTest) {
  // Prepare initial and intended motion
  Motion current_motion(0.0, {0.1, -0.1, 0.2, -0.2, 0.1, 0.0}, {0.2, -0.2, 0.1, -0.1, 0.0, 0.05},
                        {0.05, 0.05, -0.05, 0.0, -0.02, 0.03});

  Motion motion_after_intended_step(shield_->getSampleTime(), {0.102, -0.098, 0.201, -0.199, 0.101, 0.001},
                                    {0.1, -0.1, 0.05, -0.05, 0.0, 0.025}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  // Compute monitored trajectory
  auto monitored_trajectory = shield_->computeMonitoredTrajectory(current_motion, motion_after_intended_step);

  // Check the 1st step is exactly the current_motion
  const Motion& first = monitored_trajectory[0];
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(first.getAngle()[i], current_motion.getAngle()[i], 1e-5);
    EXPECT_NEAR(first.getVelocity()[i], current_motion.getVelocity()[i], 1e-5);
    EXPECT_NEAR(first.getAcceleration()[i], current_motion.getAcceleration()[i], 1e-5);
  }

  // Check 2nd step is same as motion_after_intended_step
  const Motion& second = monitored_trajectory[1];
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(second.getAngle()[i], motion_after_intended_step.getAngle()[i], 1e-5);
    EXPECT_NEAR(second.getVelocity()[i], motion_after_intended_step.getVelocity()[i], 1e-5);
    EXPECT_NEAR(second.getAcceleration()[i], motion_after_intended_step.getAcceleration()[i], 1e-5);
  }
  // Time consistency
  EXPECT_NEAR(second.getTime(), shield_->getSampleTime(), 1e-10);

  // Last motion must be zero velocity and acceleration
  const Motion& last = monitored_trajectory.back();
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(last.getVelocity()[i], 0.0, 1e-5);
    EXPECT_NEAR(last.getAcceleration()[i], 0.0, 1e-5);
  }

  // check joint limits
  EXPECT_TRUE(shield_->checkTrajectoryForJointLimits(monitored_trajectory));
}

TEST_F(SafetyShieldTest, GoalPlanningRuckigTest) {
  double sample_time = shield_->getSampleTime();
  double start_time = 0.0;

  Motion start_motion(start_time, {0.5, -0.4, 0.3, -0.2, 0.1, 0.0}, {0.1, -0.1, 0.05, -0.05, 0.0, 0.02},
                      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  Motion goal_motion(1.0, {1.0, -0.9, 0.8, -0.7, 0.6, 0.5}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                     {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  std::vector<double> v_max(6, 1.0);
  std::vector<double> a_max(6, 10.0);
  std::vector<double> j_max(6, 400.0);

  auto trajectory = shield_->goalPlanningRuckig(start_motion, goal_motion, start_time);

  ASSERT_FALSE(trajectory.empty());

  const Motion& first = trajectory.front();
  const Motion& last = trajectory.back();
  // Check the first motion is the start_motion
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(first.getAngle()[i], start_motion.getAngle()[i], 1e-5);
    EXPECT_NEAR(first.getVelocity()[i], start_motion.getVelocity()[i], 1e-5);
    EXPECT_NEAR(first.getAcceleration()[i], start_motion.getAcceleration()[i], 1e-5);

    EXPECT_NEAR(last.getAngle()[i], goal_motion.getAngle()[i], 1e-4);
    EXPECT_NEAR(last.getVelocity()[i], goal_motion.getVelocity()[i], 1e-4);
    EXPECT_NEAR(last.getAcceleration()[i], goal_motion.getAcceleration()[i], 1e-4);
  }
  // first motion time should be equal to start_time
  EXPECT_NEAR(first.getTime(), start_time, 1e-10);
  // check joint limits
  EXPECT_TRUE(shield_->checkTrajectoryForJointLimits(trajectory));
}

TEST_F(SafetyShieldTest, ComputeIntervalEdgeMotionsTest1) {
  double sample_time = shield_->getSampleTime();  // e.g. 0.1
  std::vector<Motion> monitored_trajectory;

  for (int i = 0; i <= 100; ++i) {
    double t = i * sample_time;
    std::vector<double> angle = {t};
    std::vector<double> velocity = {1.0};
    std::vector<double> acceleration = {0.0};
    monitored_trajectory.emplace_back(Motion(t, angle, velocity, acceleration));
  }

  std::vector<double> time_points = {0.0, 0.005, 0.01, 0.015, 0.02, 0.025, 0.03, 0.035, 0.04, 0.045, 0.048};

  auto interval_motions = shield_->computeIntervalEdgeMotions(monitored_trajectory, time_points);

  EXPECT_EQ(interval_motions.size(), time_points.size());

  for (size_t i = 0; i < time_points.size(); ++i) {
    EXPECT_NEAR(interval_motions[i].getTime(), time_points[i], 1e-6);
    EXPECT_NEAR(interval_motions[i].getAngle()[0], time_points[i], 1e-6);  // since angle == time
    EXPECT_NEAR(interval_motions[i].getVelocity()[0], 1.0, 1e-6);
    EXPECT_NEAR(interval_motions[i].getAcceleration()[0], 0.0, 1e-6);
  }
}

TEST_F(SafetyShieldTest, CheckMotionForJointLimits) {
  Motion motion(0.0, {0.5, -0.1, 0.2, -0.3, 0.1, 0.0});
  EXPECT_TRUE(shield_->checkMotionForJointLimits(motion));
  motion.setAngle({-2.0, -1.5, -1.0, -1.5, -2.5, -3.0});
  EXPECT_FALSE(shield_->checkMotionForJointLimits(motion));
}

TEST_F(SafetyShieldTest, CheckTrajectoryForJointLimits) {
  std::vector<Motion> trajectory = {Motion(0.0, {0.5, -0.1, 0.2, -0.3, 0.1, 0.0}),
                                    Motion(0.1, {0.6, -0.2, 0.3, -0.4, 0.2, 0.1}),
                                    Motion(0.2, {0.7, -0.3, 0.4, -0.5, 0.3, 0.2})};

  EXPECT_TRUE(shield_->checkTrajectoryForJointLimits(trajectory));

  trajectory.back().setAngle({-2.0, -1.5, -1.0, -1.5, -2.5, -3.0});
  EXPECT_FALSE(shield_->checkTrajectoryForJointLimits(trajectory));
}
}  // namespace safety_shield

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}