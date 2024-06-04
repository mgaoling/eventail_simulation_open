#include "eventail_simulation/analyzer.hpp"
#include "eventail_simulation/simulator.hpp"

using namespace eventail_simulation;

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_log_dir = (std::filesystem::path{__FILE__}.parent_path().parent_path() / "log").string();

  // initialize the simulator
  auto sim = Simulator();
  int event_number = 1000;
  vector<double> outlier_ratio = {0.000, 0.005, 0.010, 0.015, 0.020, 0.025, 0.030, 0.035, 0.040, 0.045, 0.050,
                                  0.055, 0.060, 0.065, 0.070, 0.075, 0.080, 0.085, 0.090, 0.095, 0.100};
  vector<Analyzer> analyzers(outlier_ratio.size(), Analyzer(sim.getSceneNum()));

  // motion-geometry configuration
  for (int scene_idx = 0; scene_idx < sim.getSceneNum(); ++scene_idx) {
    Velocity lin_vel = sim.sampleLinearVelocity();
    Velocity ang_vel = sim.sampleAngularVelocity();
    Line3D l = sim.sampleLine(lin_vel, ang_vel);
    sim.setupGroundTruth(lin_vel, l);

    // sample clean events and random events to construct mixed measurements
    vector<double> error_sum(outlier_ratio.size(), 0.0);
    for (int noise_idx = 0; noise_idx < sim.getNoiseNum(); ++noise_idx) {
      EventSet signal_events = sim.sampleEvents(event_number, lin_vel, ang_vel, l);
      EventSet random_events = sim.sampleRandomEvents(outlier_ratio.back() * event_number);
      Measurements measurements = sim.sampleNoisyMeasurements(signal_events, ang_vel, 0.0);

      // execute the linear solver with different inlier ratios and record errors
      for (int idx = 0; idx < outlier_ratio.size(); ++idx) {
        Measurements mixed_measurements = measurements;
        mixed_measurements.first.resize(event_number - outlier_ratio[idx] * event_number);
        mixed_measurements.first.insert(mixed_measurements.first.end(), random_events.begin(),
                                        random_events.begin() + outlier_ratio[idx] * event_number);

        double duration = 0.0, error = 0.0;
        if (!sim.linearSolve(mixed_measurements, duration, error)) {
          EVENTAIL_ERROR("Encounter unsolvable degenerate case!");
          return 1;
        }
        error_sum[idx] += error;
      }
    }
    for (int idx = 0; idx < outlier_ratio.size(); ++idx) analyzers[idx].addData(error_sum[idx] / sim.getNoiseNum());
  }

  // analyze the performance
  for (int idx = 0; idx < outlier_ratio.size(); ++idx)
    analyzers[idx].analysis("Performance Analysis on Linear Solver with " + std::to_string(1.0 - outlier_ratio[idx]) +
                            "\% inlier ratio under the noise-free setup.");

  return 0;
}
