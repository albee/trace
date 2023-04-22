#ifndef CSV_READ_H_
#define CSV_READ_H_

#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

namespace csv {
  template<typename M> M load_csv(const std::string& path);
  template<typename M> M load_dat_chaser(const std::string& path);
  template<typename M> M load_dat_target (const std::string& path);
}  // namespace csv

#endif
