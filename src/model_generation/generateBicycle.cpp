//
// Created by stefan on 22.03.22.
//

#include "generateBicycle.h"
#include <spdlog/spdlog.h>

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

  // map which assigns locations to buckets
  std::map<std::pair<std::size_t,std::size_t>, hypro::Location<double>*> buckets;

  constexpr double velocity  = 1.0;  // use velocity = one
  constexpr double wheelbase = 1.0;  // wheelbase = 1.0
  constexpr double tick_time = 1.0;  // duty cycle of the controller

  double delta_increment = ( delta_ranges.second - delta_ranges.first ) / double( discretization );
  double delta           = delta_ranges.first + delta_increment / 2.0;
  double theta_increment = ( 2 * M_PI ) / double( discretization );
  for ( std::size_t id = 0; id < discretization; ++id ) {
    double dtheta    = velocity / wheelbase * tan( delta );
    double theta_low = theta - theta_increment / 2.0;
    double theta_up  = theta + theta_increment / 2.0;
    for ( std::size_t it = 0; it < discretization; ++it ) {
      auto loc = res.createLocation();
      buckets.emplace(std::make_pair(id,it),loc);
      // set name
      loc->setName( "delta_" + std::to_string( id ) + "_theta_" + std::to_string( it ) );
      // compute and set flow
      double dx = velocity * cos( theta );
      double dy = velocity * sin( theta );
      Matrix flowmatrix      = Matrix::Zero( variableNames.size(), C+1 );
      flowmatrix( x, C )     = dx;
      flowmatrix( y, C )     = dy;
      flowmatrix( theta, C ) = dtheta;
      flowmatrix( tick, C ) = 1.0;
      loc->setFlow( hypro::linearFlow<double>( flowmatrix ) );
      // compute and set invariants
      Matrix invariant_constraints      = Matrix::Zero( 3, variableNames.size() );
      Vector invariant_constants        = Vector::Zero( 3 );
      invariant_constraints( 0, theta ) = 1;
      invariant_constraints( 1, theta ) = -1;
      invariant_constraints( 2, tick )  = 1;
      invariant_constants << theta_up, -theta_low, tick_time;
      loc->setInvariant({invariant_constraints,invariant_constants});
      // update values
      theta_low += theta_increment;
      theta_up += theta_increment;
      dtheta += theta_increment;
    }
    // update values
    delta += delta_increment;
  }

  // set some dummy initial states
  std::vector<carl::Interval<double>> initialSet{ { carl::Interval<double>{ 0, 0 }, carl::Interval<double>{ 0, 0 }, carl::Interval<double>{ 0, 0 }, carl::Interval<double>{ 0, 0 } } };
  hypro::HybridAutomaton<double>::locationConditionMap initialConfigurations{{res.getLocations().front(), hypro::conditionFromIntervals(initialSet)}};
  res.setInitialStates(initialConfigurations);

  /* compute/add transitions
   * Idea: Transitions where delta changes can only occur between two cycles (i.e., when tick = tick_time), all other
   * transitions are enabled by the violation of respective invariants.
   */

  for(auto& [key,source] : buckets) {
    auto& [delta,theta] = key;
    // determine neighbor indices
    auto lowerDeltaNeighbor = delta > 0 ? delta-1 : discretization-1;
    auto upperDeltaNeighbor = delta < discretization - 1 ? delta+1 : 0;
    auto lowerThetaNeighbor = theta > 0 ? theta-1 : discretization-1;
    auto upperThetaNeighbor = theta < discretization - 1 ? theta+1 : 0;
    // create theta-transitions, lower first
    auto lowerTheta = source->createTransition(buckets[std::make_pair(delta,lowerThetaNeighbor)]);
    // upper theta neighbor
    auto upperTheta = source->createTransition(buckets[std::make_pair(delta,upperThetaNeighbor)]);
    // create delta-transitions, lower first
    auto lowerDelta = source->createTransition(buckets[std::make_pair(lowerDeltaNeighbor,theta)]);
    // upper delta neighbor
    auto upperDelta = source->createTransition(buckets[std::make_pair(upperThetaNeighbor,theta)]);
    // delta-guard
    Matrix guard_constraints = Matrix::Zero(2,variableNames.size());
    guard_constraints(0,tick) = 1;
    guard_constraints(1,tick) = -1;
    Vector guard_constants = Vector::Zero(2);
    guard_constants << tick_time, -tick_time;
    lowerDelta->setGuard({guard_constraints,guard_constants});
    upperDelta->setGuard({guard_constraints,guard_constants});
    // delta-reset
    Matrix reset_matrix = Matrix::Identity(variableNames.size(), variableNames.size());
    Vector reset_vector = Vector::Zero(variableNames.size());
    reset_matrix(tick,tick) = 0;
    lowerDelta->setReset(hypro::Reset<double>(reset_matrix,reset_vector));
    upperDelta->setReset(hypro::Reset<double>(reset_matrix,reset_vector));
  }

  return res;
}

}  // namespace modelGenerator