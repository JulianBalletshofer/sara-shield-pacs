#include <gtest/gtest.h>
#include <math.h>

#include <cmath>
#include <Eigen/Dense>
#include <tuple>

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

TEST_F(SafetyShieldTest, NextMotionTest) {
  // Dummy human measurement
  std::vector<reach_lib::Point> dummy_human_meas(23);
  dummy_human_meas[0] = reach_lib::Point(1.5177, -0.1214,  0.3653);
  dummy_human_meas[1] = reach_lib::Point(1.3805, -0.1239,  0.3661);
  dummy_human_meas[2] = reach_lib::Point(1.4457, -0.1015,  0.5656);
  dummy_human_meas[3] = reach_lib::Point(1.55199995, -0.49652919,  0.35673341);
  dummy_human_meas[4] = reach_lib::Point(1.34220006, -0.50628106,  0.35305421);
  dummy_human_meas[5] = reach_lib::Point(1.45120127, -0.10260095,  0.70079994);
  dummy_human_meas[6] = reach_lib::Point(1.53840061, -0.89403554,  0.30875003);
  dummy_human_meas[7] = reach_lib::Point(1.35799936, -0.90420248,  0.30646656);
  dummy_human_meas[8] = reach_lib::Point(1.45260135, -0.12800337,  0.75369877);
  dummy_human_meas[9] = reach_lib::Point(1.56480009, -0.95111153,  0.42744498);
  dummy_human_meas[10] = reach_lib::Point(1.33259994, -0.95372288,  0.42934275);
  dummy_human_meas[11] = reach_lib::Point(1.4498005 , -0.08519133,  0.96759635);
  dummy_human_meas[12] = reach_lib::Point(1.53150086, -0.09399649,  0.87549717);
  dummy_human_meas[13] = reach_lib::Point(1.37090087, -0.08939671,  0.87259628);
  dummy_human_meas[14] = reach_lib::Point(1.45500055, -0.13649181,  1.03259597);
  dummy_human_meas[15] = reach_lib::Point(1.62240178, -0.08509644,  0.90599443);
  dummy_human_meas[16] = reach_lib::Point(1.27479982, -0.08029668,  0.90509315);
  dummy_human_meas[17] = reach_lib::Point(1.88200111, -0.05759683,  0.89318002);
  dummy_human_meas[18] = reach_lib::Point(1.02110049, -0.05889703,  0.89177984);
  dummy_human_meas[19] = reach_lib::Point(2.13130103, -0.05649683,  0.90218228);
  dummy_human_meas[20] = reach_lib::Point(0.76580055, -0.05329701,  0.89958196);
  dummy_human_meas[21] = reach_lib::Point(2.21530148, -0.04149671,  0.89398712);
  dummy_human_meas[22] = reach_lib::Point(0.68120021, -0.04299688,  0.8933868);

  double init_x = 0.0;
  double init_y = 0.0;
  double init_z = 0.912;
  double init_roll = 0.0;
  double init_pitch = 0.0;
  double init_yaw = 0.0;
  std::vector<double> init_qpos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  reach_lib::AABB table = reach_lib::AABB({-1.0, -1.0, -0.1}, {1.0, 1.0, 0.0});
  std::vector<reach_lib::AABB> environment_elements = {table};
  ShieldType shield_type = ShieldType::SSM;

  double t = 0.0;
  safety_shield::Motion current_motion;
  safety_shield::Motion next_motion = shield_->getCurrentMotion();
  for (int ep = 0; ep < 2; ep++) {
    for (int i = 0; i < 100; i++) {  // i < 100; i<10000
      t += 0.001;
      shield_->humanMeasurement(dummy_human_meas, t);
      t += 0.003;
      current_motion = shield_->getCurrentMotion();
      EXPECT_TRUE(current_motion.hasSamePos(&next_motion));
      EXPECT_TRUE(current_motion.hasSameVel(&next_motion));
      EXPECT_TRUE(current_motion.hasSameAcc(&next_motion));
      if (i % 10 == 0) {  // % 2
        std::vector<double> qpos{0.2 * t, t, t,
                                 t,       t, std::min(t, 3.1)};  // qpos{0.2*t, 0.0, 0.0, 0.0, 0.0, std::min(t, 3.1)};
        std::vector<double> qvel{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        shield_->newLongTermTrajectory(qpos, qvel);
        // spdlog::info("new LTT");
      }
      next_motion = shield_->step(t);
      // spdlog::info("finished step");
    }
    shield_->reset(init_x, init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, t, environment_elements, shield_type);
    next_motion = shield_->getCurrentMotion();
  }
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
// ---------------------------------------------------------------------------
// Jumping-measurement tests (single-joint robot)
// ---------------------------------------------------------------------------

/**
 * @brief Run the jumping-measurement scenario and assert kinematic bound compliance.
 *
 * Checks finite-difference bounds on consecutive step() outputs:
 *   |q(t+1) - q(t)| / dt  <= v_max
 *   |v(t+1) - v(t)| / dt  <= a_max
 *   |a(t+1) - a(t)| / dt  <= j_max
 *
 * @param shield     Initialised safety shield (already has init_qpos set).
 * @param q_end      Goal joint position.
 * @param n_jump     Number of steps between measurement switches.
 * @param sample_time Sample time in seconds.
 * @param t_max      Total simulation time in seconds.
 * @param v_max      Per-joint maximum velocity [rad/s].
 * @param a_max      Per-joint maximum acceleration [rad/s²].
 * @param j_max      Per-joint maximum jerk [rad/s³].
 */
static void runJumpingTest(SafetyShieldExposed* shield, double q_end, int n_jump,
                           double sample_time, double t_max,
                           const std::vector<double>& v_max,
                           const std::vector<double>& a_max,
                           const std::vector<double>& j_max) {
  // Single joint: one measurement point is sufficient.
  const std::vector<reach_lib::Point> meas_far   = {reach_lib::Point(10.0, 10.0, 10.0)};
  const std::vector<reach_lib::Point> meas_close = {reach_lib::Point(0.0,  0.0,  0.0)};

  // Tolerance to absorb floating-point rounding in the shield.
  constexpr double kTol = 1e-8;

  shield->newLongTermTrajectory({q_end}, {0.0});

  double t = 0.0;
  const int n_steps = static_cast<int>(t_max / sample_time);
  Motion prev_motion = shield->getCurrentMotion();

  for (int i = 0; i < n_steps; i++) {
    // Mirror the timing split from jumping_measurement_debug.cc:
    // human measurement arrives at t + dt/4, step() is called at t + dt.
    t += sample_time / 4.0;
    // Measurements start in the "far" state; switch every n_jump steps.
    const bool is_close = (static_cast<int>(std::floor(static_cast<double>(i) / n_jump)) % 2 == 1);
    shield->humanMeasurement(is_close ? meas_close : meas_far, t);
    t += 3.0 / 4.0 * sample_time;

    Motion next_motion = shield->step(t);
    spdlog::info("q_pos[0] = {}", next_motion.getAngle()[0]);

    // Skip the finite-difference check on the very first output because
    // prev_motion is the initialisation state, not a step() output.
    if (i > 0) {
      const int n_joints = static_cast<int>(v_max.size());
      for (int j = 0; j < n_joints; j++) {
        const double dq = (next_motion.getAngle()[j]        - prev_motion.getAngle()[j])        / sample_time;
        const double dv = (next_motion.getVelocity()[j]     - prev_motion.getVelocity()[j])     / sample_time;
        const double da = (next_motion.getAcceleration()[j] - prev_motion.getAcceleration()[j]) / sample_time;

        EXPECT_LE(std::abs(dq), v_max[j] + kTol)
            << "Joint " << j << " position difference exceeds v_max at step " << i
            << " (q_end=" << q_end << ", n_jump=" << n_jump << ")";
        if (std::abs(prev_motion.getAngle()[j] - 0.0) > kTol && std::abs(prev_motion.getAngle()[j] - q_end) > kTol) {
          EXPECT_GE(std::abs(dq), kTol)
            << "Joint " << j << " position difference is zero at step " << i
            << " next_motion.getAngle()[j] = " << next_motion.getAngle()[j] << ", prev_motion.getAngle()[j] = " << prev_motion.getAngle()[j]
            << " (q_end=" << q_end << ", n_jump=" << n_jump << ")";
        }
        EXPECT_LE(std::abs(dv), a_max[j] + kTol)
            << "Joint " << j << " velocity difference exceeds a_max at step " << i
            << " (q_end=" << q_end << ", n_jump=" << n_jump << ")";
        EXPECT_LE(std::abs(da), j_max[j] + kTol)
            << "Joint " << j << " acceleration difference exceeds j_max at step " << i
            << " (q_end=" << q_end << ", n_jump=" << n_jump << ")";
      }
    }

    prev_motion = next_motion;
  }
}

// Single TEST_P body used by both the simple and grid instantiations.
// Parameters: (q_end, n_jump). The robot always starts at q=0.0.
TEST_P(SafetyShieldSingleJointTest, JumpingMeasurement) {
  const double q_end  = std::get<0>(GetParam());
  const int    n_jump = std::get<1>(GetParam());
  runJumpingTest(shield_, q_end, n_jump, sample_time_,
                 1.0, v_max_allowed_, a_max_allowed_, j_max_allowed_);
}

// Helper to produce valid GTest identifiers from double/int param values.
static std::string jumpingTestName(
    const ::testing::TestParamInfo<std::tuple<double, int>>& info) {
  auto fmt = [](double v) -> std::string {
    std::string s = std::to_string(v);
    for (char& c : s) {
      if (c == '-') c = 'n';
      if (c == '.') c = 'p';
    }
    auto dot = s.find('p');
    if (dot != std::string::npos) {
      auto last = s.find_last_not_of('0');
      if (last != std::string::npos && last > dot)
        s = s.substr(0, last + 1);
      else if (last == dot)
        s = s.substr(0, dot);
    }
    return s;
  };
  return "qe" + fmt(std::get<0>(info.param)) + "_nj" + std::to_string(std::get<1>(info.param));
}

// Simple case: robot at q=0.0, goal at q=-0.12, measurement switches every 10 steps.
INSTANTIATE_TEST_SUITE_P(
    Simple,
    SafetyShieldSingleJointTest,
    ::testing::Values(std::make_tuple(-0.12, 10)),
    jumpingTestName);

// Grid test over q_end and n_jump measurements
INSTANTIATE_TEST_SUITE_P(
    Grid,
    SafetyShieldSingleJointTest,
    ::testing::Combine(
        ::testing::Values(-0.2, -0.15, -0.1, -0.05, 0.0, 0.05, 0.1, 0.15, 0.2),  // q_end
        ::testing::Values(2, 4, 8, 16)),  // n_jump
    jumpingTestName);

}  // namespace safety_shield

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}