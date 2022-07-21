//
// Created by bmaderbacher on 06.07.22.
//

#include "generateBaseController.h"

#include "tool_car/ctrlConversion.h"

hypro::HybridAutomaton<double> simplexArchitectures::generateBaseController(
    double x_min, double x_max, double y_min, double y_max, BicycleBaseController &ctrl,
    std::pair<double, double> delta_ranges, std::size_t delta_discretization, std::size_t theta_discretization,
    double x_interval_size, double y_interval_size ) {

  using Matrix = hypro::matrix_t<double>;
  using Vector = hypro::vector_t<double>;

  hypro::HybridAutomaton<double> res;
  constexpr Eigen::Index         x     = 0; // global position x
  constexpr Eigen::Index         y     = 1; // global position y
  constexpr Eigen::Index         C     = 2; // constants

  std::vector<std::string>       variableNames{ "x", "y"};
  res.setVariables(variableNames);

  // sync labels: theta_left, theta_right, delta_{0..delta_discretization-1}, stop

  // bucket indices: (x,y,theta)
  std::map<std::tuple<std::size_t, std::size_t, std::size_t>, hypro::Location<double>*> buckets;
  std::map<std::tuple<std::size_t, std::size_t, std::size_t>, std::pair<double,double>> outputs;

  assert(x_max > x_min);
  auto num_x_buckets = static_cast<size_t>(ceil((x_max-x_min)/x_interval_size));

  assert(y_max > y_min);
  auto num_y_buckets = static_cast<size_t>(ceil((y_max-y_min)/y_interval_size));

  double x_low  = x_min;
  double x_high = x_min + x_interval_size;
  double x_representative = x_min + (x_interval_size/2);

  double y_low  = y_min;
  double y_high = y_min + y_interval_size;
  double y_representative = y_min + (y_interval_size/2);

  double theta_increment = ( 2 * M_PI ) / double( theta_discretization );
  double theta_representative  = theta_increment / 2.0;

  //TODO merge (in x and y) neighboring locations with the same output.
  for ( std::size_t ix = 0; ix < num_x_buckets; ++ix ) {
    y_low  = y_min;
    y_high = y_min + y_interval_size;
    y_representative = y_min + (y_interval_size/2);
    for ( std::size_t iy = 0; iy < num_y_buckets; ++iy ) {
      theta_representative  = theta_increment / 2.0;
      for ( std::size_t it = 0; it < theta_discretization; ++it ) {
        auto loc = res.createLocation();
        buckets.emplace( std::make_tuple( ix, iy, it ), loc );
        loc->setName( "x_" + std::to_string( ix ) + "_y_" + std::to_string( iy ) + "_theta_" + std::to_string( it ) );

        // compute and set invariants
        // ATTENTION: The order of constraints is important, it is reused for guards later!!!
        Matrix invariant_constraints      = Matrix::Zero( 4, variableNames.size() );
        Vector invariant_constants        = Vector::Zero( 4 );
        invariant_constraints( 0, x ) = 1;
        invariant_constraints( 1, x ) = -1;
        invariant_constraints( 2, y )  = 1;
        invariant_constraints( 3, y )  = -1;
        invariant_constants << x_high, -x_low, y_high, -y_low;
        loc->setInvariant({invariant_constraints,invariant_constants});

//        std::cout << "Queried position: ( " << x_representative << "; " << y_representative << "; " << theta_representative << ")" << std::endl;
        auto output = ctrl.generateInput(Point{ x_representative, y_representative, theta_representative, 1, 1});
        outputs.emplace(std::make_tuple( ix, iy, it ), std::make_pair(output[0], output[1]));

        theta_representative += theta_increment;
      }
      y_low += y_interval_size;
      y_high += y_interval_size;
      y_representative += y_interval_size;
    }
    x_low += x_interval_size;
    x_high += x_interval_size;
    x_representative += x_interval_size;
  }

  for(auto& [key,source] : buckets) {
    auto& [xBucket, yBucket, thetaBucket] = key;
    auto lowerXNeighbor = xBucket-1;
    auto upperXNeighbor = xBucket+1;
    auto lowerYNeighbor = yBucket-1;
    auto upperYNeighbor = yBucket+1;
    auto lowerThetaNeighbor = thetaBucket > 0 ? thetaBucket-1 : theta_discretization-1;
    auto upperThetaNeighbor = thetaBucket < theta_discretization - 1 ? thetaBucket+1 : 0;

    // create theta-transitions, lower first
    auto lowerTheta = source->createTransition(buckets[std::make_tuple(xBucket, yBucket, lowerThetaNeighbor)]);
    lowerTheta->addLabel(hypro::Label("theta_right"));
    // upper theta neighbor
    auto upperTheta = source->createTransition(buckets[std::make_tuple(xBucket, yBucket, upperThetaNeighbor)]);
    upperTheta->addLabel(hypro::Label("theta_left"));

    // create x-transitions, lower first
    if(xBucket > 0) {
      auto lowerX = source->createTransition( buckets[std::make_tuple( lowerXNeighbor, yBucket, thetaBucket )] );

      Matrix guard_constraints = Matrix::Zero(2,variableNames.size());
      guard_constraints(0,x) = 1;
      guard_constraints(1,x) = -1;
      Vector guard_constants = Vector::Zero(2);
      guard_constants << -source->getInvariant().getVector()(1), source->getInvariant().getVector()(1);
      lowerX->setGuard({guard_constraints,guard_constants});
    }
    // upper x neighbor
    if(xBucket < num_x_buckets-1) {
      auto upperX = source->createTransition( buckets[std::make_tuple( upperXNeighbor, yBucket, thetaBucket )] );

      Matrix guard_constraints = Matrix::Zero(2,variableNames.size());
      guard_constraints(0,x) = 1;
      guard_constraints(1,x) = -1;
      Vector guard_constants = Vector::Zero(2);
      guard_constants << -source->getInvariant().getVector()(1), source->getInvariant().getVector()(1);
      upperX->setGuard({guard_constraints,guard_constants});
    }

    // create y-transitions, lower first
    if(yBucket > 0) {
      auto lowerY = source->createTransition( buckets[std::make_tuple( xBucket, lowerYNeighbor, thetaBucket )] );

      Matrix guard_constraints = Matrix::Zero(2,variableNames.size());
      guard_constraints(0,y) = 1;
      guard_constraints(1,y) = -1;
      Vector guard_constants = Vector::Zero(2);
      guard_constants << -source->getInvariant().getVector()(1), source->getInvariant().getVector()(1);
      lowerY->setGuard({guard_constraints,guard_constants});
    }
    // upper x neighbor
    if(yBucket < num_y_buckets-1) {
      auto upperY = source->createTransition( buckets[std::make_tuple( xBucket, upperYNeighbor, thetaBucket )] );

      Matrix guard_constraints = Matrix::Zero(2,variableNames.size());
      guard_constraints(0,y) = 1;
      guard_constraints(1,y) = -1;
      Vector guard_constants = Vector::Zero(2);
      guard_constants << -source->getInvariant().getVector()(1), source->getInvariant().getVector()(1);
      upperY->setGuard({guard_constraints,guard_constants});
    }

    // update transitions

    auto& [delta, val] = outputs[key];

    if (val == 0.0) {
      auto stopTrans = source->createTransition( source );
      stopTrans->addLabel(hypro::Label("stop"));
    } else {
      auto deltaChange = source->createTransition( source );
      auto delta_bucket = getDeltaBucket(delta, delta_ranges, delta_discretization);
      deltaChange->addLabel(hypro::Label("delta_"+std::to_string(delta_bucket)));
    }


  }

  return res;
}
