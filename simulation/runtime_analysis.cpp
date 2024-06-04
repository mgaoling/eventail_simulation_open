#include "eventail_simulation/analyzer.hpp"
#include "eventail_simulation/simulator.hpp"

using namespace eventail_simulation;

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_log_dir = (std::filesystem::path{__FILE__}.parent_path().parent_path() / "log").string();

  // initialize the simulator
  auto sim = Simulator();
  auto engine = Random();
  auto linear_analyzer = Analyzer(sim.getSceneNum() * sim.getNoiseNum());
  auto polyjam_analyzer = Analyzer(sim.getSceneNum() * sim.getNoiseNum());

  // motion-geometry configuration
  for (int scene_idx = 0; scene_idx < sim.getSceneNum(); ++scene_idx) {
    Velocity lin_vel = sim.sampleLinearVelocity();
    Velocity ang_vel = sim.sampleAngularVelocity();
    Line3D l = sim.sampleLine(lin_vel, ang_vel);
    sim.setupGroundTruth(lin_vel, l);

    // sample random, noisy measurements
    for (int noise_idx = 0; noise_idx < sim.getNoiseNum(); ++noise_idx) {
      EventSet clean_events = sim.sampleFiveEvents(lin_vel, ang_vel, l);
      Measurements noisy_measurements =
          sim.sampleNoisyMeasurements(clean_events, ang_vel, engine.sampleUniformScalar(0, 5));

      // execute both polyjam and linear solver and record errors
      double linear_duration, polyjam_duration, linear_error, polyjam_error;
      if (!sim.linearSolve(noisy_measurements, linear_duration, linear_error)) {
        EVENTAIL_WARN("Encounter unsolvable degenerate case!");
        --noise_idx;
        continue;
      }
      if (!sim.polyjamSolve(noisy_measurements, polyjam_duration, polyjam_error)) {
        EVENTAIL_WARN("Encounter unsolvable degenerate case!");
        --noise_idx;
        continue;
      }
      linear_analyzer.addData(linear_duration);
      polyjam_analyzer.addData(polyjam_duration);
    }
  }

  // analyze the runtime performance
  linear_analyzer.analysis("Runtime Analysis on Linear Solver");
  polyjam_analyzer.analysis("Runtime Analysis on Polyjam Solver");

  return 0;
}
