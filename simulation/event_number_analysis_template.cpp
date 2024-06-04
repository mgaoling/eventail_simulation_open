#include "eventail_simulation/analyzer.hpp"
#include "eventail_simulation/simulator.hpp"

using namespace eventail_simulation;

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_log_dir = (std::filesystem::path{__FILE__}.parent_path().parent_path() / "log").string();

  // initialize the simulator
  auto sim = Simulator();
  vector<int> event_numbers = {5, 6, 7, 8, 9, 10, 20, 30, 40, 50, 100, 1000};

  // loop over various noise level
  auto noise_level_vec = sim.getNoiseLevels();
  for (auto noise_level : noise_level_vec) {
    vector<Analyzer> analyzers(event_numbers.size(), Analyzer(sim.getSceneNum()));

    // motion-geometry configuration
    for (int scene_idx = 0; scene_idx < sim.getSceneNum(); ++scene_idx) {
      Velocity lin_vel = sim.sampleLinearVelocity();
      Velocity ang_vel = sim.sampleAngularVelocity();
      Line3D l = sim.sampleLine(lin_vel, ang_vel);
      sim.setupGroundTruth(lin_vel, l);

      // sample random, noisy measurements
      vector<double> error_sum(event_numbers.size(), 0.0);
      for (int noise_idx = 0; noise_idx < sim.getNoiseNum(); ++noise_idx) {
        EventSet clean_events = sim.sampleEvents(event_numbers.back(), lin_vel, ang_vel, l);
        Measurements noisy_measurements = sim.sampleNoisyMeasurements(clean_events, ang_vel, noise_level);

        // execute the linear solver with different set of events and record errors
        for (int idx = 0; idx < event_numbers.size(); ++idx) {
          Measurements noisy_measurements_subset = noisy_measurements;
          noisy_measurements_subset.first.resize(event_numbers[idx]);

          double duration = 0.0, error = 0.0;
          if (!sim.linearSolve(noisy_measurements_subset, duration, error)) {
            EVENTAIL_ERROR("Encounter unsolvable degenerate case!");
            return 1;
          }
          error_sum[idx] += error;
        }
      }

      for (int idx = 0; idx < event_numbers.size(); ++idx) analyzers[idx].addData(error_sum[idx] / sim.getNoiseNum());
    }

    // analyze the performance
    for (int idx = 0; idx < event_numbers.size(); ++idx)
      analyzers[idx].analysis("Performance Analysis on Linear Solver with " + std::to_string(event_numbers[idx]) +
                              " events under the noise level of " + std::to_string(noise_level) + '.');
  }

  return 0;
}
