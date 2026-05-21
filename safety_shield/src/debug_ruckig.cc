#include <string>
#include <vector>
#include <chrono>
#include <ctime>
#include "safety_shield/human_reach.h"
#include "safety_shield/long_term_traj.h"
#include "safety_shield/motion.h"
#include "safety_shield/robot_reach.h"
#include "safety_shield/safety_shield.h"
#include "safety_shield/verify.h"

auto motionToString = [](const safety_shield::Motion& m) {
    auto vecToStr = [](const std::vector<double>& v) {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < v.size(); ++i) {
            oss << v[i];
            if (i + 1 < v.size()) oss << ", ";
        }
        oss << "]";
        return oss.str();
    };
    std::ostringstream oss;
    oss << "Angles: " << vecToStr(m.getAngle())
        << " | Velocities: " << vecToStr(m.getVelocity())
        << " | Accelerations: " << vecToStr(m.getAcceleration());
    return oss.str();
};

int main() {
  double sample_time = 0.001;
  std::string trajectory_config_file = std::string("../config/trajectory_parameters_schunk.yaml");
  std::string robot_config_file = std::string("../config/robot_parameters_schunk.yaml");
  std::string mocap_config_file = std::string("../config/mujoco_mocap.yaml");
  double init_x = 0.0;
  double init_y = 0.0;
  double init_z = 0.912;
  double init_roll = 0.0;
  double init_pitch = 0.0;
  double init_yaw = 0.0;
  std::vector<double> init_qpos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  reach_lib::AABB table = reach_lib::AABB({init_x - 1.0, init_y - 1.0, init_z - 0.1}, {init_x + 1.0, init_y + 1.0, init_z});
  std::vector<reach_lib::AABB> environment_elements = {table};
  safety_shield::ShieldType shield_type = safety_shield::ShieldType::SSM;
  safety_shield::SafetyShield* shield = new
      safety_shield::SafetyShield(sample_time, trajectory_config_file, robot_config_file, mocap_config_file, init_x,
                                  init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, environment_elements, shield_type);
  safety_shield::SafetyShield* shieldNonPathConsistent = new
      safety_shield::SafetyShield(sample_time, trajectory_config_file, robot_config_file, mocap_config_file, init_x,
                                  init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, environment_elements, shield_type);
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

spdlog::info("Debug started.");
auto wall_clock_start = std::chrono::system_clock::now();  // For total runtime
std::vector<double> q{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
safety_shield::Motion start_motion(0, q, q, q, q, 0);       // Actual motion start
shieldNonPathConsistent->setNonPathConsistent();
shieldNonPathConsistent->reset(init_x, init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, 0.0, environment_elements, shield_type);
double t = 0.0;
for (int ep = 0; ep < 1; ep++) {
for (int i = 0; i < 500; i++) {
    t += 0.001;
    shield->humanMeasurement(dummy_human_meas, t);
    shieldNonPathConsistent->humanMeasurement(dummy_human_meas, t);
    t += 0.003;
    if (i % 10 == 0) {
    std::vector<double> qpos{0.2 * t, t, t, t, t, std::min(t, 3.1)};
    std::vector<double> qvel{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    shield->newLongTermTrajectory(qpos, qvel);
    shieldNonPathConsistent->newLongTermTrajectory(qpos, qvel);
    }
    // path consistent shield
    safety_shield::Motion current_motion = shield->getCurrentMotion();
    auto t_start_step = std::chrono::high_resolution_clock::now();
    safety_shield::Motion next_motion = shield->step(t);
    auto t_end_step = std::chrono::high_resolution_clock::now();
    auto step_duration = std::chrono::duration<double, std::milli>(t_end_step - t_start_step).count();

    // non path conistent shield
    safety_shield::Motion current_motion_non_path = shieldNonPathConsistent->getCurrentMotion();
    auto t_start_planning = std::chrono::high_resolution_clock::now();
    safety_shield::Motion next_motion_non_path = shieldNonPathConsistent->step(t);
    auto t_end_planning = std::chrono::high_resolution_clock::now();
    auto planning_duration = std::chrono::duration<double, std::milli>(t_end_planning - t_start_planning).count();
    bool safe = shieldNonPathConsistent->getSafety();
    spdlog::info("Step duration path conistent: {:.3f} ms | step duration non path conistent: {:.3f} ms", step_duration, planning_duration);
    
}
shield->reset(init_x, init_y, init_z, init_roll, init_pitch, init_yaw, init_qpos, t, environment_elements, shield_type);
}
delete shield;
delete shieldNonPathConsistent;
auto wall_clock_end = std::chrono::system_clock::now();
std::chrono::duration<double> elapsed_seconds = wall_clock_end - wall_clock_start;
spdlog::info("Total elapsed wall time: {:.3f} seconds", elapsed_seconds.count());
return 0;
}