//
// Created by stefan on 22.03.22.
//

#include "generateBicycle.h"
#include <spdlog/spdlog.h>

#include <vector>
#include "util.h"

namespace modelGenerator {

hypro::HybridAutomaton<double> generateBicycle( std::pair<double, double> delta_ranges, std::size_t delta_discretization, std::size_t theta_discretization ) {
  // delta: steering angle, relative to the current heading
  using Matrix = hypro::matrix_t<double>;
  using Vector = hypro::vector_t<double>;

  hypro::HybridAutomaton<double> res;
  constexpr Eigen::Index         x     = 0; // global position x
  constexpr Eigen::Index         y     = 1; // global position y
  constexpr Eigen::Index         theta = 2; // global heading (measured counter clock wise from the global x-axis)
  constexpr Eigen::Index         tick  = 3; // controller clock cycle
  constexpr Eigen::Index         v     = 4; // velocity
  constexpr Eigen::Index         C     = 5; // constants
  std::vector<std::string>       variableNames{ "x", "y", "theta", "tick", "v" };
  res.setVariables( variableNames );

  // map which assigns locations to buckets
  std::map<std::pair<std::size_t, std::size_t>, hypro::Location<double>*> buckets;

  constexpr double velocity  = 1.0;  // use velocity = one
  constexpr double wheelbase = 1.0;  // wheelbase = 1.0
  constexpr double tick_time = 0.1;  // duty cycle of the controller

  double delta_min_rad = degreeToRadiance(delta_ranges.first);
  double delta_max_rad = degreeToRadiance(delta_ranges.second);

  double delta_increment = (delta_max_rad - delta_min_rad ) / double( delta_discretization );
  double delta           = delta_min_rad + delta_increment / 2.0;
  double theta_increment = ( 2 * M_PI ) / double( theta_discretization );
  for ( std::size_t id = 0; id < delta_discretization; ++id ) {
    double dtheta    = 1 / wheelbase * tan( delta );
    double vtheta    = theta_increment / 2.0; //theta value
    double theta_low = 0;
    double theta_up  = theta_increment;
    for ( std::size_t it = 0; it < theta_discretization; ++it ) {
      auto loc = res.createLocation();
      buckets.emplace( std::make_pair( id, it ), loc );
      // set name
      loc->setName( "delta_" + std::to_string( id ) + "_theta_" + std::to_string( it ) );
      // compute and set flow
      double dx              = cos( vtheta );
      double dy              = sin( vtheta );
      Matrix flowmatrix      = Matrix::Zero( variableNames.size() + 1, C + 1 );
      flowmatrix( x, v )     = dx;
      flowmatrix( y, v )     = dy;
      flowmatrix( theta, v ) = dtheta;
      flowmatrix( tick, C )  = 1.0;
      flowmatrix( v, C )     = 0;
      loc->setFlow( hypro::linearFlow<double>( flowmatrix ) );
      // compute and set invariants
      // ATTENTION: The order of constraints is important, it is reused for guards later!!!
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
      vtheta += theta_increment;
    }
    // update values
    delta += delta_increment;
  }

  // set some dummy initial states
  std::vector<carl::Interval<double>> initialSet{ { carl::Interval<double>{ 0, 0 }, carl::Interval<double>{ 0, 0 }, carl::Interval<double>{ 0, 0 }, carl::Interval<double>{ 0, 0 }, carl::Interval<double>{ 1, 1 }} };
  hypro::HybridAutomaton<double>::locationConditionMap initialConfigurations{{res.getLocations().front(), hypro::conditionFromIntervals(initialSet)}};
  res.setInitialStates(initialConfigurations);

  /* compute/add transitions
   * Idea: Transitions where delta changes can only occur between two cycles (i.e., when tick = tick_time), all other
   * transitions are enabled by the violation of respective invariants. The only variable that is reset is tick.
   */

  for(auto& [key,source] : buckets) {
    auto& [deltaBucket,thetaBucket] = key;
    // determine neighbor indices
    //bool haveLowerDeltaNeighbor = deltaBucket > 0;
    //bool haveUpperDeltaNeighbor = deltaBucket < discretization - 1;
    auto lowerThetaNeighbor = thetaBucket > 0 ? thetaBucket-1 : theta_discretization-1;
    auto upperThetaNeighbor = thetaBucket < theta_discretization - 1 ? thetaBucket+1 : 0;
    // create theta-transitions, lower first
    auto lowerTheta = source->createTransition(buckets[std::make_pair(deltaBucket,lowerThetaNeighbor)]);
    // upper theta neighbor
    auto upperTheta = source->createTransition(buckets[std::make_pair(deltaBucket,upperThetaNeighbor)]);

    // stop car
    Matrix guard_constraints     = Matrix::Zero( 2, variableNames.size() );
    guard_constraints( 0, tick ) = 1;
    guard_constraints( 1, tick ) = -1;
    Vector guard_constants       = Vector::Zero( 2 );
    guard_constants << tick_time, -tick_time;

    Matrix reset_matrix        = Matrix::Identity( variableNames.size(), variableNames.size() );
    Vector reset_vector        = Vector::Zero( variableNames.size() );
    reset_matrix( tick, tick ) = 0;
    reset_matrix( v, v ) = 0;

    auto stopTransition = source->createTransition( source );
    stopTransition->setGuard({guard_constraints, guard_constants});
    stopTransition->setReset(hypro::Reset<double>( reset_matrix, reset_vector ));
    stopTransition->addLabel(hypro::Label("stop"));

    // delta can be changed to all other choices at end of each cycle
    for ( std::size_t id = 0; id < delta_discretization; ++id ) {
      // delta change
      auto changeDelta = source->createTransition( buckets[std::make_pair(id,upperThetaNeighbor)] );
      // delta-guard
      guard_constraints     = Matrix::Zero( 2, variableNames.size() );
      guard_constraints( 0, tick ) = 1;
      guard_constraints( 1, tick ) = -1;
      guard_constants       = Vector::Zero( 2 );
      guard_constants << tick_time, -tick_time;

      changeDelta->setGuard( { guard_constraints, guard_constants } );
      // delta-reset
      reset_matrix        = Matrix::Identity( variableNames.size(), variableNames.size() );
      reset_vector        = Vector::Zero( variableNames.size() );
      reset_matrix( tick, tick ) = 0;
      changeDelta->setReset( hypro::Reset<double>( reset_matrix, reset_vector ) );

      // label for controller synchronisation
      changeDelta->addLabel(hypro::Label("delta_"+std::to_string(id)));
    }

    // theta-guard upper
    guard_constraints = Matrix::Zero(2,variableNames.size());
    guard_constraints(0,theta) = 1;
    guard_constraints(1,theta) = -1;
    guard_constants = Vector::Zero(2);
    guard_constants << source->getInvariant().getVector()(0), -source->getInvariant().getVector()(0);
    upperTheta->addLabel(hypro::Label("theta_left"));
    upperTheta->setGuard({guard_constraints,guard_constants});
    if(thetaBucket == theta_discretization-1){ //wrap around from 360 degree to 0 degree
      Matrix theta_reset_matrix = Matrix::Identity(variableNames.size(), variableNames.size());
      Vector theta_reset_vector = Vector::Zero(variableNames.size());
      theta_reset_matrix(theta,theta) = 0;
      theta_reset_vector(theta) = 0.00001;
      upperTheta->setReset(hypro::Reset<double>(theta_reset_matrix, theta_reset_vector));
    }
    // theta-guard lower
    guard_constraints = Matrix::Zero(2,variableNames.size());
    guard_constraints(0,theta) = 1;
    guard_constraints(1,theta) = -1;
    guard_constants = Vector::Zero(2);
    guard_constants << -source->getInvariant().getVector()(1), source->getInvariant().getVector()(1);
    lowerTheta->setGuard({guard_constraints,guard_constants});
    lowerTheta->addLabel(hypro::Label("theta_right"));
    if(thetaBucket == 0){ //wrap around from 0 degree to 360 degree
      Matrix theta_reset_matrix = Matrix::Identity(variableNames.size(), variableNames.size());
      Vector theta_reset_vector = Vector::Zero(variableNames.size());
      theta_reset_matrix(theta,theta) = 0;
      theta_reset_vector(theta) = lowerTheta->getTarget()->getInvariant().getVector()(0)-0.00001;
      lowerTheta->setReset(hypro::Reset<double>(theta_reset_matrix, theta_reset_vector));
    }
  }

  return res;
}

}  // namespace modelGenerator