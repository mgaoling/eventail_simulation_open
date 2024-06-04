#include "eventail_simulation/analyzer.hpp"

namespace eventail_simulation {

Analyzer::Analyzer(int size) { holder_.reserve(size); }

void Analyzer::addData(double data) { holder_.emplace_back(data); }

void Analyzer::analysis(string description, double skip_percentage) {
  // Skip the top and bottom X% of the sorted data. This is to avoid degenerate cases caused by random sampling.
  if (skip_percentage < 0.0 || skip_percentage > 1.0) {
    EVENTAIL_WARN("The skip percentage shall fall in the interval from 0 to 1. Use 0 instead.");
    skip_percentage = 0.0;
  }
  LOG(INFO) << "Analyzer: " << description << " | skip_percentage = " << skip_percentage;

  // Analyze the data.
  glogVector("Data: ", holder_);
  sort(holder_.begin(), holder_.end());
  size_t size = holder_.size();
  double min = *std::min_element(holder_.begin(), holder_.end());
  double max = *std::max_element(holder_.begin(), holder_.end());
  double mean = std::accumulate(holder_.begin(), holder_.end(), 0.0) / size;
  double median = (size % 2 == 0) ? (holder_[size / 2 - 1] + holder_[size / 2]) / 2 : holder_[size / 2];
  double sum_of_squares = 0.0;
  for (auto value : holder_) sum_of_squares += pow(value - mean, 2);
  double std_dev = sqrt(sum_of_squares / size);
  LOG(INFO) << "Range: [" << min << ", " << max << "]" << endl;
  LOG(INFO) << "Size: " << size << endl;
  LOG(INFO) << "Mean: " << mean << endl;
  LOG(INFO) << "Median: " << median << endl;
  LOG(INFO) << "Standard Deviation: " << std_dev << endl;

  // Output the analysis.
  EVENTAIL_INFO(description);
  cout << "Range: [" << min << ", " << max << "]" << endl;
  cout << "Size: " << size << endl;
  cout << "Mean: " << mean << endl;
  cout << "Median: " << median << endl;
  cout << "Standard Deviation: " << std_dev << endl;
}

}  // namespace eventail_simulation
