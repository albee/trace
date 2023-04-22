/**
 * @file conic_fitting.h
 * @brief Functions related to conic fitting to 2D data points.
 * @author Antonio Teran (antonioteran@google.com)
 * Copyright 2021 Antonio Teran
 */

#ifndef POLHODE_ALIGNMENT_CONIC_FITTING_H_
#define POLHODE_ALIGNMENT_CONIC_FITTING_H_

#include <Eigen/Core>
#include <tuple>
#include <utility>

#include "polhode_alignment/conics.h"

namespace pao {

/// Bundle with canonical conic parameters, residual, and conic type.
template <typename T>
struct ConicFit {
  Conic<T> c = Conic<T>(T(1), T(0), T(2), T(0), T(0),
                     T(-1));  ///< Full conic, dummy values
  ParametricConic<T> K;       ///< Best conic fit parameters.
  T residual_square = T(-1);  ///< Best fit cost, i.e., min(abs(lambda)).
  Eigen::Matrix<T, 3, 1> eigvec{T(-1), T(-1), T(-1)};  //< Eigenvector of fit.
};

/**
 * @brief Find best canonical ellipse and hyperbola fits to the provided data.
 * @param[in] pts  2xN matrix with (x,y) plane points.
 * @return Pair of fits, first for ellipse, second for hyperbola.
 */
template <typename T>
std::pair<ConicFit<T>, ConicFit<T>> FitConicCanonical(
    const Eigen::Matrix<T, 2, Eigen::Dynamic>& pts);

/**
 * @brief Find best canonical fit between ellipse and hyperbola to data.
 * @param[in] pts  2xN matrix with (x,y) plane points.
 * @return Best canonical conic fit (either a hyperbola or an ellipse).
 */
template <typename T>
ConicFit<T> FitBestConicCanonical(
    const Eigen::Matrix<T, 2, Eigen::Dynamic>& pts);

/**
 * @brief Get the best conic fits in each plane given an optimal rotation R_EG.
 * @param[in] R_EG  Optimal rotation from rando frame G to hyperbolic axis E.
 * @param[in] G_omegaB  Angular velocity matrix 3xN in rando frame G.
 * @return Tuple with best fits on XY, XZ, and YZ planes, respectively.
 */
template <typename T>
std::tuple<ConicFit<T>, ConicFit<T>, ConicFit<T>> GetBestConicFits(
    const Eigen::Matrix<T, 3, 3>& R_EG,
    const Eigen::Matrix<T, 3, Eigen::Dynamic>& G_omegaB);

}  // namespace pao

#include "conic_fitting.tpp"

#endif  // POLHODE_ALIGNMENT_CONIC_FITTING_H_
