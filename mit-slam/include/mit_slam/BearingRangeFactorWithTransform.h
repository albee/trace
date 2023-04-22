/**
 * @file    BearingRangeFactorWithTransform.h
 * @brief   Bearing and range factor with sensor pose offset.
 * @author  Tim Setterfield
 * @author  Antonio Teran
 */

#ifndef BEARINGRANGEFACTORWITHTRANSFORM_H_
#define BEARINGRANGEFACTORWITHTRANSFORM_H_

#include <gtsam/sam/BearingRangeFactor.h>

namespace gtsam {

/**
 * Binary factor for a bearing/range measurement, with a transform applied
 * (Tim Setterfield modification, based on RangeFactorWithTransform)
 * @addtogroup SLAM
 */
template <typename A1, typename A2,
          typename B = typename Bearing<A1, A2>::result_type,
          typename R = typename Range<A1, A2>::result_type>
class BearingRangeFactorWithTransform
    : public ExpressionFactor2<BearingRange<A1, A2>, A1, A2> {
 private:
  typedef BearingRange<A1, A2> T;
  typedef ExpressionFactor2<T, A1, A2> Base;
  typedef BearingRangeFactorWithTransform<A1, A2> This;

  A1 body_T_sensor_;  ///< The pose of the sensor in the body frame

 public:
  typedef boost::shared_ptr<This> shared_ptr;

  /// default constructor
  BearingRangeFactorWithTransform() {}

  /// primary constructor
  BearingRangeFactorWithTransform(Key key1, Key key2, const B& measuredBearing,
                                  const R& measuredRange,
                                  const SharedNoiseModel& model,
                                  const A1& body_T_sensor)
      : Base(key1, key2, model, T(measuredBearing, measuredRange)),
        body_T_sensor_(body_T_sensor) {
    this->initialize(expression(key1, key2));
  }

  virtual ~BearingRangeFactorWithTransform() {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  // Return measurement expression
  virtual Expression<T> expression(Key key1, Key key2) const {
    Expression<A1> body_T_sensor__(body_T_sensor_);
    Expression<A1> nav_T_body_(key1);
    Expression<A1> nav_T_sensor_(traits<A1>::Compose, nav_T_body_,
                                 body_T_sensor__);
    Expression<A2> a2_(key2);
    return Expression<T>(T::Measure, nav_T_sensor_, a2_);
  }

  /// print
  virtual void print(const std::string& s = "",
                     const KeyFormatter& kf = DefaultKeyFormatter) const {
    std::cout << s << "BearingRangeFactorWithTransform" << std::endl;
    Base::print(s, kf);
  }

};  // BearingRangeFactor

/// traits
//template <typename A1, typename A2, typename B, typename R>
//struct traits<BearingRangeFactorWithTransform<A1, A2, B, R> >
//    : public Testable<BearingRangeFactorWithTransform<A1, A2, B, R> > {};

}  // end namespace gtsam

#endif /* BEARINGRANGEFACTORWITHTRANSFORM_H_ */
