/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file ConicProjectionFactor.h
 *  @author Tonio Teran
 **/

#pragma once

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/RegularHessianFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/JacobianFactorSVD.h>

#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace gtsam {

/**
 * Factor that differentiates a series of rotations to obtain angular velocity
 * measurements, which are then used to find the optimal rotation that produces
 * best fit conics with respect to the angular velocity points projected onto
 * all three planes (XY, XZ, YZ).
 *
 * @addtogroup SAM
 */
class GTSAM_EXPORT ConicProjectionFactor : public NonlinearFactor {
 private:
  typedef NonlinearFactor Base;
  typedef ConicProjectionFactor This;

 public:
  /// Type of estimation method for obtaining the angular velocity measurements.
  /// If direct, the added poses or rotations directly corresponding to the
  /// tumbling target. For indirect, the delta transform between the sought
  /// tumblinb rigid body is to be computed using the observer's body pose in
  /// different frames, and chaining the poses together.
  enum class EstimationType { DIRECT, INDIRECT };

  /// Type of base variable.
  enum class VariableType { ROT, POSE };

  /// Type of conic available for least squares fitting.
  enum class ConicType { ELLIPSE, HYPERBOLA };
  // /// Bundle with canonical conic parameters [A C F], residual, and conic
  // type. typedef std::tuple<Vector3, double, ConicType> ConicFitResult;
  /**
   * @brief Bundle with canonical conic parameters, residual, and conic type.
   */
  struct ConicFitResult {
    Vector3 K;       // Canonical conic parameters [A C F]^T (scaled eigenvec).
    Vector3 v;       // Raw, unit eigenvector before scaling.
    double cost;     // Best fit cost, i.e., min(abs(lambda)).
    ConicType type;  // Type of conic for best fit (ELLIPSE, HYPERBOLA).

    inline friend std::ostream& operator<<(std::ostream& os,
                                           const ConicFitResult& r) {
      os << "Conic fit result:\n"
         << " K: [A C F] = " << r.K.transpose() << std::endl
         << " Eigvec = " << r.v.transpose() << std::endl
         << " Cost = " << r.cost << std::endl
         << " Type = "
         << ((r.type == ConicType::ELLIPSE) ? "Ellipse" : "Hyperbola")
         << std::endl;
      return os;
    }
  };

  static const int Dim = 3;  ///< SO(3) dimension.

  /// Pair of subsequent rotations from which to extract the angular velocities.
  typedef std::pair<Key, Key> RotPair;
  /// Tuple of subsequent chain rotation for indirect omega extraction.
  typedef std::tuple<Key, Key, Key, Key> RotTuple;

  // Shorten Eigen's typedefs for convenience.
  typedef Eigen::GeneralizedEigenSolver<Eigen::MatrixXd>::VectorType VectorType;
  typedef Eigen::GeneralizedEigenSolver<Eigen::MatrixXd>::ComplexVectorType
      ComplexVectorType;
  typedef Eigen::GeneralizedEigenSolver<Eigen::MatrixXd>::EigenvectorsType
      EigenvectorsType;

 protected:
  Key R_PG_;  ///< Symbol for the sought rotation of geometric frame G wrt
              ///< principal axes frame P.

  mutable SharedNoiseModel noiseModel_;  ///< Noisemodel for set of all meas.

  VariableType type_;  ///< Connected to either Rot3 or Pose3.

  EstimationType estimation_;  ///< Direct or indirect omega extraction.

  std::vector<double> deltaTs_;   ///< Delta time between rotation pairs.
  std::vector<RotPair> pairs_;    ///< Key pairs from which to extract omegas.
  std::vector<RotTuple> tuples_;  ///< Key tuples from which to extract omegas.

  mutable Eigen::GeneralizedEigenSolver<Eigen::MatrixXd>
      ges_;    ///< For cost function.
  Matrix3 C_;  ///< Conic constraints to enforce ellipse and hyperbola fits.

 public:
  /// @name Constructors
  /// @{

  /** Default constructor only for serialization. */
  ConicProjectionFactor() {}

  /**
   * @brief Constructor.
   * @param R_PG        Key for the sought orientation of G wrt P.
   * @param noiseModel  Noise model for the factor.
   * @param type        Type of variable being used, eiter rotations or poses.
   * @param estimation  Method used to extract the angular velocity meas.
   */
  ConicProjectionFactor(
      const Key& R_PG, const SharedNoiseModel& noiseModel,
      const VariableType& type = VariableType::ROT,
      const EstimationType& estimation = EstimationType::DIRECT)
      : R_PG_(R_PG),
        noiseModel_(noiseModel),
        type_(type),
        estimation_(estimation) {
    // Insert the sought rotation key to the factor.
    keys_.push_back(R_PG_);
    // Create the conic constraints to enforce an ellipse and a hyperbola.
    // clang-format off
    C_ << 0.0, 0.5, 0.0,
          0.5, 0.0, 0.0,
          0.0, 0.0, 0.0;
    // clang-format on
  }

  /** Destructor. */
  virtual ~ConicProjectionFactor() {}

  /// @}

  /// @name Basic utilities
  /// @{

  /** Dimension of the factor (rows on linearization). */
  size_t dim() const override { return Dim; }

  /** Get current noise model. */
  const SharedNoiseModel& noiseModel() const { return noiseModel_; }

  /// @}

  /// @name Testable
  /// @{

  /** Print. */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /** Check if two factors are equal. */
  using NonlinearFactor::equals;  // To let compiler know we are not hiding.
  bool equals(const ConicProjectionFactor& f, const double tol = 1e-9) const;

  /// @}

  /// @name Main functionality
  /// @{

  /**
   * @brief Add a new pair of rotations from which to extract angular velocity.
   * @param Ri      Key to the base orientation.
   * @param Rj      Key to the target/resulting orientation.
   * @param deltaT  Time elapsed between Ri and Rj, in seconds.
   */
  void add(const Key& Ri, const Key& Rj, const double deltaT);

  /**
   * @brief Overloaded function for indirect estimation of angular velocity.
   * @param Gi      Key to the base orientation of tumbling frame.
   * @param Gj      Key to the resulting orientation of tumbling frame.
   * @param Bi      Key to the base orientation of moving observer.
   * @param Bj      Key to the resulting orientation of moving observer.
   * @param deltaT  Time elapsed between Ri and Rj, in seconds.
   */
  void add(const Key& Gi, const Key& Gj, const Key& Bi, const Key& Bj,
           const double deltaT);

  /**
   * Evaluate the measurement error and (optionally) its Jacobians.
   * @param R_PG    Value for orientation of geometric frame wrt principal axes.
   * @param omegas  Vector of points with available angular velocity measrmts.
   * @param H1      Jacobian of the error wrt R_PG.
   * @return The measurement error vector h(x,l) - z.
   */
  Vector3 evaluateError(const Rot3& R_PG, const std::vector<Point3>& omegas,
                        OptionalJacobian<3, 3> H1 = boost::none) const;

  /**
   * Linearize a nonlinear factor to get a GaussianFactor.
   * Ax-b ~ h(x+dx) - z = h(x) + A*dx - z; hence b = z - h(x) = -error_vec(x).
   * @param vals Set of values for the pose and the point landmark.
   * @return Linearized factor in the form of a GaussianFactor.
   *
   * Follows same structure as SmartFactorBase to choose linearization strategy.
   */
  boost::shared_ptr<GaussianFactor> linearize(
      const Values& vals) const override;

  /**
   * Error function without the NoiseModel, z - h(x).
   * @param vals  Set of values for the involved variables.
   * @param H     Jacobians of unwhitenedError wrt variables in values.
   * @return Error without NoiseModel applied (z - h(x)).
   */
  Vector unwhitenedError(
      const Values& vals,
      boost::optional<std::vector<Matrix>&> H = boost::none) const;

  /**
   * Calculate the error of the factor, weighted by the noise mode.
   * @param vals Set of values for the pose and the point landmark.
   * @return 1/2 sqrd Mahalanobis distance b/w the prediction and measurement.
   */
  double error(const Values& vals) const override;

  /// @}

  /// @name Advanced Utilities
  /// @{

  /**
   * @brief Given attitude values, differentiate rotation to extract omegas.
   * @param vals  Set of values for all rotations stored in this factor.
   */
  std::vector<Point3> extractAngularVelocities(const Values& vals) const;

  /**
   * @brief Extract best conic fit from generalized eigenvalue problem solution.
   * @param eigenvecs  Matrix with resulting eigenvectors.
   * @param alphas     Complex number vector with eigenvalues' numerators.
   * @param betas      Real-valued vector with eigenvalues' denominator.
   *
   * The resulting eigenvalues can then be recovered as alphas/.betas.
   */
  ConicFitResult ParseEigenPair(const EigenvectorsType& eigenvecs,
                                const ComplexVectorType& alphas,
                                const VectorType& betas) const;

  /**
   * @brief Create the Jacobian d(M^T)/d(M), where M is an mxn matrix.
   * @param m  Number of rows of the matrix.
   * @param n  Number of rows of the matrix.
   */
  Matrix TransposeSelfJacobian(const size_t m, const size_t n) const;

  /**
   * @brief Create the Jacobian of d(M^T M)/d(M), where M is [x1 x2 x3].
   * @param x1  First column vector.
   * @param x2  Second column vector.
   * @param x3  Third column vector.
   * @param N   Number of rows of matrix M.
   */
  Matrix SquareSelfJacobian(const Vector& x1, const Vector& x2,
                            const Vector& x3, const size_t N) const;

  /**
   * @brief Create a canonical vector of all zeros except at position `i`.
   * @param n  Dimension of the vector.
   * @param i  Position of the `1`, 0-based.
   */
  Vector CanonicalUnitVector(const size_t n, const size_t i) const;

  /**
   * @brief Create row corresponding to the eigenvalue perturbation Jacobian.
   * @param eigenvector  The resulting eigenvector for the best conic fit.
   */
  Matrix EigenvaluePerturbationJacobian(const Vector3& eigenvector) const;

  /// @}

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace gtsam
