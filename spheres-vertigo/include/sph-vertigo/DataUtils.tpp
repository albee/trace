/**
 * @file DataUtils.h
 * @brief Utility functions for interacting with the raw datasets.
 * @date September 11, 2019
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran (teran@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

#ifndef SPH_VERTIGO_DATAUTILS_TPP_
#define SPH_VERTIGO_DATAUTILS_TPP_

namespace sph {

/* ************************************************************************** */
template <typename MsgVecT>
double AvgMsgTimestep(const MsgVecT &msg_vec) {
  double cumulative_dt = 0;
  unsigned int num_meas = msg_vec.size();

  for (unsigned int i = 0; i < num_meas - 1; i++) {
    cumulative_dt += msg_vec[i + 1].sph_test_time - msg_vec[i].sph_test_time;
  }
  return (cumulative_dt / (num_meas - 1.0) / 1000.0);
}

}  // namespace sph

#endif  // SPH_VERTIGO_DATAUTILS_TPP_
