//
// Created by stefan on 22.03.22.
//

#include "generateCarModel.h"
#include <spdlog/spdlog.h>

#include <vector>
#include "util.h"

namespace modelGenerator {

hypro::HybridAutomaton<double> generateCarModel(std::size_t theta_discretization, double cycleTime, bool includeThetaTransitions) {
  // delta: steering angle, relative to the current heading
  using Matrix = hypro::matrix_t<double>;
  using Vector = hypro::vector_t<double>;

  hypro::HybridAutomaton<double> res;
  constexpr Eigen::Index         x     = 0; // global position x
  constexpr Eigen::Index         y     = 1; // global position y
  constexpr Eigen::Index         theta = 2; // global theta heading
  constexpr Eigen::Index         tick  = 3; // controller clock cycle
  constexpr Eigen::Index         v     = 4; // velocity
  constexpr Eigen::Index         C     = 5; // constants
  std::vector<std::string>       variableNames{ "x", "y", "theta", "tick", "v" };
  res.setVariables( variableNames );

  // map which assigns locations to buckets
  std::map<std::size_t, hypro::Location<double>*> buckets;

  constexpr double velocity  = 1.0;  // use velocity = one
  double tick_time = cycleTime;  // duty cycle of the controller

  double theta_increment = ( 2 * M_PI ) / double( theta_discretization );
  double vtheta    = theta_increment / 2.0; //theta value
  double theta_low = 0;
  double theta_up  = theta_increment;
  for ( std::size_t it = 0; it < theta_discretization; ++it ) {
    auto loc = res.createLocation();
    buckets.emplace( it, loc );
    // set name
    loc->setName( "theta_" + std::to_string( it ) );
    // compute and set flow
    double dx              = cos( vtheta );
    double dy              = sin( vtheta );
    Matrix flowmatrix      = Matrix::Zero( variableNames.size() + 1, C + 1 );
    flowmatrix( x, v )     = dx;
    flowmatrix( y, v )     = dy;
    flowmatrix( theta, C )  = 0.0;
    flowmatrix( tick, C )  = 1.0;
    flowmatrix( v, C )     = 0;
    loc->setFlow( hypro::linearFlow<double>( flowmatrix ) );
    // compute and set invariants
    // ATTENTION: The order of constraints is important, it is reused for guards later!!!
    Matrix invariant_constraints      = Matrix::Zero( 1, variableNames.size() );
    Vector invariant_constants        = Vector::Zero( 1 );
    invariant_constraints( 0, tick )  = 1;
    invariant_constants << tick_time;
    loc->setInvariant({invariant_constraints,invariant_constants});
    // update values
    theta_low += theta_increment;
    theta_up += theta_increment;
    vtheta += theta_increment;
  }


  // set some dummy initial states
  std::vector<carl::Interval<double>> initialSet{ { carl::Interval<double>{ 0, 0 }, carl::Interval<double>{ 0, 0 }, carl::Interval<double>{ 0, 0 }, carl::Interval<double>{ 1, 1 }} };
  hypro::HybridAutomaton<double>::locationConditionMap initialConfigurations{{res.getLocations().front(), hypro::conditionFromIntervals(initialSet)}};
  res.setInitialStates(initialConfigurations);


  for(auto& [thetaBucket,source] : buckets) {

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


    if(includeThetaTransitions) {
      // theta can be changed to all other choices at end of each cycle
      vtheta = theta_increment / 2.0;  // theta value
      for ( std::size_t it = 0; it < theta_discretization; ++it ) {
        // theta change
        auto changeTheta = source->createTransition( buckets[it] );
        // theta-guard
        guard_constraints            = Matrix::Zero( 2, variableNames.size() );
        guard_constraints( 0, tick ) = 1;
        guard_constraints( 1, tick ) = -1;
        guard_constants              = Vector::Zero( 2 );
        guard_constants << tick_time, -tick_time;

        changeTheta->setGuard( { guard_constraints, guard_constants } );
        // theta-reset
        reset_matrix                 = Matrix::Identity( variableNames.size(), variableNames.size() );
        reset_vector                 = Vector::Zero( variableNames.size() );
        reset_matrix( tick, tick )   = 0;
        reset_matrix( theta, theta ) = 0;
        reset_vector( theta )        = vtheta;
        changeTheta->setReset( hypro::Reset<double>( reset_matrix, reset_vector ) );

        // label for controller synchronisation
        changeTheta->addLabel( hypro::Label( "set_theta_" + std::to_string( it ) ) );
        vtheta += theta_increment;
      }
    }
  }

  return res;
}

}  // namespace modelGenerator