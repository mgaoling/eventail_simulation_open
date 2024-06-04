#include "eventail_simulation/analyzer.hpp"
#include "eventail_simulation/simulator.hpp"

using namespace eventail_simulation;

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_log_dir = (std::filesystem::path{__FILE__}.parent_path().parent_path() / "log").string();

  // initialize the simulator
  auto sim = Simulator();
  vector<int> line_numbers = {2, 3, 4, 5, 6, 7, 8, 9, 10};

  // loop over various noise level
  auto noise_level_vec = sim.getNoiseLevels();
  for (auto noise_level : noise_level_vec) {
    vector<Analyzer> analyzers(line_numbers.size(), Analyzer(sim.getSceneNum()));

    // motion configuration
    for (int scene_idx = 0; scene_idx < sim.getSceneNum(); ++scene_idx) {
      Velocity lin_vel = sim.sampleLinearVelocity();
      Velocity ang_vel = sim.sampleAngularVelocity();

      // geometry configuration and sample noisy measurements
      int line_numbers_idx = 0;
      for (int sampled_line_idx = 1; sampled_line_idx <= line_numbers.back(); ++sampled_line_idx) {
        Line3D l = sim.sampleLine(lin_vel, ang_vel);
        sim.setupGroundTruth(lin_vel, l);
        EventSet clean_events = sim.sampleTenEvents(lin_vel, ang_vel, l);
        Measurements noisy_measurements = sim.sampleNoisyMeasurements(clean_events, ang_vel, noise_level);

        // execute the linear solver
        double linear_duration, linear_error;
        if (!sim.linearSolve(noisy_measurements, linear_duration, linear_error)) {
          EVENTAIL_ERROR("Encounter unsolvable degenerate case!");
          return 1;
        }

        // execute the linear velocity averagor and record errors
        if (line_numbers_idx < line_numbers.size() && line_numbers[line_numbers_idx] == sampled_line_idx) {
          double linear_averaged_error;
          Velocity linear_averaged_estimation;
          if (line_numbers_idx == line_numbers.size() - 1)
            sim.linearAverage(linear_averaged_estimation, linear_averaged_error, true);
          else
            sim.linearAverage(linear_averaged_estimation, linear_averaged_error, false);
          analyzers[line_numbers_idx].addData(linear_averaged_error);
          ++line_numbers_idx;
        }
      }
    }

    // analyze the performance
    for (int idx = 0; idx < line_numbers.size(); ++idx)
      analyzers[idx].analysis("Performance Analysis on Linear Averagor, with the noise level of " +
                              std::to_string(noise_level));
  }

  return 0;
}
