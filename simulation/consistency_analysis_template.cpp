#include "eventail_simulation/analyzer.hpp"
#include "eventail_simulation/simulator.hpp"

using namespace eventail_simulation;

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_log_dir = (std::filesystem::path{__FILE__}.parent_path().parent_path() / "log").string();

  // initialize the simulator
  auto sim_5 = Simulator(), sim_10 = Simulator();

  // loop over various noise level
  auto noise_level_vec = sim_10.getNoiseLevels();
  for (auto noise_level : noise_level_vec) {
    vector<Analyzer> analyzers(3, Analyzer(sim_10.getSceneNum()));

    // motion configuration
    for (int scene_idx = 0; scene_idx < sim_10.getSceneNum(); ++scene_idx) {
      Velocity lin_vel = sim_10.sampleLinearVelocity();
      Velocity ang_vel = sim_10.sampleAngularVelocity();

      // geometry configuration and sample noisy measurements
      for (int line_idx = 0; line_idx < 5; ++line_idx) {
        Line3D l = sim_10.sampleLine(lin_vel, ang_vel);
        sim_5.setupGroundTruth(lin_vel, l);
        sim_10.setupGroundTruth(lin_vel, l);
        EventSet clean_events_10 = sim_10.sampleTenEvents(lin_vel, ang_vel, l);
        Measurements noisy_measurements_10 = sim_10.sampleNoisyMeasurements(clean_events_10, ang_vel, noise_level);

        // execute both polyjam and linear solver with five events
        Measurements noisy_measurements_5 = noisy_measurements_10;
        noisy_measurements_5.first.resize(5);
        double polyjam_duration_5, linear_duration_5, polyjam_error_5, linear_error_5;
        if (!sim_5.polyjamSolve(noisy_measurements_5, polyjam_duration_5, polyjam_error_5)) {
          EVENTAIL_WARN("Encounter unsolvable degenerate case!");
          line_idx--;
          continue;
        }
        if (!sim_5.linearSolve(noisy_measurements_5, linear_duration_5, linear_error_5)) {
          EVENTAIL_ERROR("Encounter unsolvable degenerate case!");
          return 1;
        }

        // execute the linear solver with ten events
        double linear_duration_10, linear_error_10;
        if (!sim_10.linearSolve(noisy_measurements_10, linear_duration_10, linear_error_10)) {
          EVENTAIL_ERROR("Encounter unsolvable degenerate case!");
          return 1;
        }
      }

      // execute both polyjam and linear velocity averagor and record errors
      double polyjam_averaged_error_5, linear_averaged_error_5, linear_averaged_error_10;
      Velocity polyjam_averaged_estimation_5, linear_averaged_estimation_5, linear_averaged_estimation_10;
      sim_5.polyjamAverage(polyjam_averaged_estimation_5, polyjam_averaged_error_5);
      sim_5.linearAverage(linear_averaged_estimation_5, linear_averaged_error_5);
      sim_10.linearAverage(linear_averaged_estimation_10, linear_averaged_error_10);
      analyzers[0].addData(polyjam_averaged_error_5);
      analyzers[1].addData(linear_averaged_error_5);
      analyzers[2].addData(linear_averaged_error_10);
    }

    // analyze the performance
    analyzers[0].analysis("Performance Analysis on Polyjam Averagor (5 events), with the noise level of " +
                          std::to_string(noise_level));
    analyzers[1].analysis("Performance Analysis on Linear Averagor (5 events), with the noise level of " +
                          std::to_string(noise_level));
    analyzers[2].analysis("Performance Analysis on Linear Averagor (10 events), with the noise level of " +
                          std::to_string(noise_level));
  }

  return 0;
}
