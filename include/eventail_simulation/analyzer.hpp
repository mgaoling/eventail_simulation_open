#pragma once

#include "geometry.hpp"
#include "types.hpp"

namespace eventail_simulation {

class Analyzer {
 public:
  using Ptr = shared_ptr<Analyzer>;
  explicit Analyzer(int size);

  void addData(double data);
  void analysis(string description, double skip_percentage = 0.0);

 private:
  vector<double> holder_;
};

}  // namespace eventail_simulation
