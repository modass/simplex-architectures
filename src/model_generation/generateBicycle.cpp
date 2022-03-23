//
// Created by stefan on 22.03.22.
//

#include "generateBicycle.h"

#include <vector>

namespace modelGenerator {

hypro::HybridAutomaton<double> generateBicycle( std::pair<double, double> delta_ranges, std::size_t discretization ) {
  using Matrix = hypro::matrix_t<double>;
  using Vector = hypro::vector_t<double>;

  hypro::HybridAutomaton<double> res;
  constexpr Eigen::Index         x     = 0;
  constexpr Eigen::Index         y     = 1;
  constexpr Eigen::Index         theta = 2;
  constexpr Eigen::Index         tick  = 3;
  constexpr Eigen::Index         C     = 4;
  std::vector<std::string>       variableNames{ "x", "y", "theta", "tick" };
  res.setVariables( variableNames );

  constexpr double velocity  = 1.0;  // use velocity = one
  constexpr double wheelbase = 1.0;  // wheelbase = 1.0
  constexpr double tick_time = 1.0;  // duty cycle of the controller

  double delta_increment = ( delta_ranges.second - delta_ranges.first ) / double( discretization );
  double delta           = delta_ranges.first + delta_increment / 2.0;
  double theta_increment = ( 2 * M_PI ) / double( discretization );
  for ( std::size_t id = 0; id < discretization; ++id ) {
    double dtheta    = velocity / wheelbase * tan( delta );
    double theta     = 0.0;
    double theta_low = theta - theta_increment / 2.0;
    double theta_up  = theta + theta_increment / 2.0;
    for ( std::size_t it = 0; it < discretization; ++it ) {
      auto loc = res.createLocation();
      // set name
      loc->setName( "delta_" + std::to_string( id ) + "_theta_" + std::to_string( it ) );
      // compute and set flow
      double dx = velocity * cos( theta );
      double dy = velocity * sin( theta );
      theta += theta_increment;
      Matrix flowmatrix      = Matrix::Zero( variableNames.size(), C );
      flowmatrix( x, C )     = dx;
      flowmatrix( y, C )     = dy;
      flowmatrix( theta, C ) = dtheta;
      loc->setFlow( hypro::linearFlow<double>( flowmatrix ) );
      // compute and set invariants
      Matrix invariant_constraints      = Matrix::Zero( 3, variableNames.size() );
      Vector invariant_constants        = Vector::Zero( 3 );
      invariant_constraints( 0, theta ) = 1;
      invariant_constraints( 1, theta ) = -1;
      invariant_constraints( 2, tick )  = 1;
      invariant_constants << theta_up, -theta_low, tick_time;
      // update values
      theta_low += theta_increment;
      theta_up += theta_increment;
      theta += theta_increment;
    }
    // update values
    delta += delta_increment;
  }

  return res;
}

}  // namespace modelGenerator