/**
 * @file InertiaRatios.cpp
 * @brief Manifold corresponding to physically consistent inertia ratios.
 * @date September 05, 2020
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran (teran@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

#include "sph-vertigo/InertiaRatios.h"

namespace sph {

/* ************************************************************************** */
bool InertiaRatios::IsInside(const double J1, const double J2) const {
  // i)   J1 >= J2
  if (!(J1 >= J2)) return false;

  // ii)  J2 >= J1 - 1
  if (!(J2 >= J1 - 1)) return false;

  // iii) J2 >= 1
  if (!(J2 >= 1)) return false;

  // If we make it all the way here, then all constraints are satisfied.
  return true;
}

/* ************************************************************************** */
InertiaRatios InertiaRatios::Project(const double J1, const double J2) const {
  if (IsInside(J1, J2)) {
    return InertiaRatios(J1, J2);
  } else {
    // Orthogonal projection from 5 distinct locations.
    Region region = WhichRegion(J1, J2);
    if (region == Region::I) {  // Projection to left boundary.
      // Shift to local coordinates.
      // Project onto line (proj = ( (p * line)/(line * line) ) * line).
      // Shift back to global.
      // Closed form solution for line (1,1), and offset (1,1).

      double projCoord = (J1 + J2) / 2.0;
      return InertiaRatios(projCoord, projCoord);

    } else if (region == Region::II) {  // Project to left corner.
      return InertiaRatios(1.0, 1.0);

    } else if (region == Region::III) {  // Projection to lower boundary.
      // Keep x coordinate, replace y with 1.0;
      return InertiaRatios(J1, 1.0);

    } else if (region == Region::IV) {  // Projection to right corner.
      return InertiaRatios(2.0, 1.0);

    } else {  // Projection to right boundary.
      // } else if (region == Region::V) {  // Projection to right boundary.

      // Line is (1, 1), offset is (2, 1).
      double projCoord = ((J1 - 2) + (J2 - 1)) / 2.0;
      return InertiaRatios(projCoord + 2, projCoord + 1);
    }
  }
}

/* ************************************************************************** */
InertiaRatios::Region InertiaRatios::WhichRegion(const double J1,
                                                 const double J2) const {
  if (IsInside(J1, J2)) {
    return Region::O;
  } else if ((J2 > J1) && (J2 > (-J1 + 2))) {
    // I:   Open region above (x = y) and above (-x+2 = y).
    return Region::I;

  } else if ((J2 <= (-J1 + 2)) && (J1 <= 1)) {
    // II:  Closed region below (-x+2 = y) and left of (x = 1).
    return Region::II;

  } else if ((J1 > 1) && (J1 < 2) && (J2 < 1)) {
    // III: Open region between (x = 1) and (x = 2) and below (1 = y).
    return Region::III;

  } else if ((J1 >= 2) && (J2 <= (-J1 + 3))) {
    // IV:  Closed region right of (x = 2) and below (-x+3 = y).
    return Region::IV;

  } else {
    // } else if ((J2 < (J1 - 1)) && (J2 > (-J1 + 3))) {
    // V:   Open region below (x - 1 = y) and above (-x+3 = y).
    return Region::V;
  }
}

}  // namespace sph
