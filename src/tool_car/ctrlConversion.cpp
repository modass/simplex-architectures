/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 23.06.22.
 */

#include "ctrlConversion.h"

#include "utility/coordinate_util.h"
#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

namespace simplexArchitectures {

LocPtr convertCtrlToLocation(const Point& in, const hypro::HybridAutomaton<Number>& automaton, LocPtr lastLocation, std::size_t delta_discretization, const std::pair<double, double>& delta_ranges) {
  LocPtr res = nullptr;
  // By convention the first component of the point is the used delta.
  // Simple approach: string-comparison
  // 1 find all locations for which the theta-component matches, should be |delta_discretization| many
  std::string theta_substring = lastLocation->getName().substr(lastLocation->getName().find("theta_"), std::string::npos);
  std::vector<LocPtr> candidates;
  for(const auto* lptr : automaton.getLocations()) {
    if(lptr->getName().find(theta_substring) != std::string::npos) {
      candidates.push_back(lptr);
    }
  }
  // 2 select the candidate with the correct delta-bucket
  std::size_t delta_bucket_index = getDeltaBucket(in[0], delta_ranges, delta_discretization);
  std::string delta_substring = "delta_" + std::to_string(delta_bucket_index);
  for(const auto* lptr : candidates) {
    if(lptr->getName().find(delta_substring) != std::string::npos) {
      return lptr;
    }
  }

  return res;
}

LocPtr convertCtrlToLocationSimple(double theta, const hypro::HybridAutomaton<Number>& automaton, std::size_t theta_discretization) {
  LocPtr res = nullptr;

  std::size_t theta_bucket_index = getThetaBucket(theta, theta_discretization);
  std::string theta_substring = "theta_" + std::to_string(theta_bucket_index);
  for(const auto* lptr : automaton.getLocations()) {
    if(lptr->getName().find(theta_substring) != std::string::npos) {
      return lptr;
    }
  }

  return res;
}

double convertDeltaToTheta(double delta, double currentTheta, std::size_t theta_discretization) {

  auto targetTheta = currentTheta + 0.5 * tan(delta);
  if(targetTheta < 0) {
    targetTheta = targetTheta + 2 * M_PI;
  } else if(targetTheta >  2 * M_PI ) {
    targetTheta = targetTheta - 2 * M_PI;
  }

  std::size_t theta_bucket_index = getThetaBucket(targetTheta, theta_discretization);
  double theta_increment = ( 2 * M_PI ) / double( theta_discretization );

  return theta_increment*0.5 + theta_bucket_index * theta_increment;

}

std::size_t getThetaBucket(Number theta, std::size_t discretization) {
  if(theta < 0 || theta > 2 * M_PI) {
    throw std::logic_error("Invalid theta-value, must be between 0 and 2*PI");
  }
  double theta_increment = ( 2 * M_PI ) / double( discretization );
  double theta_low = 0;
  std::size_t theta_bucket = 0;
  bool first = true;
  while (theta > theta_low && theta_bucket < discretization) {
    ++theta_bucket;
    theta_low += theta_increment;
    first = false;
  }
  return first ? theta_bucket : --theta_bucket;
}

std::size_t getDeltaBucket(Number delta, const std::pair<double,double>& delta_ranges, std::size_t discretization) {
  auto deltaMin = DegreesToRadians(delta_ranges.first);
  auto deltaMax = DegreesToRadians(delta_ranges.second);
//  if(delta > deltaMax || delta < deltaMin) {
//    throw std::logic_error("Provided delta is not within the provided range");
//  }
  if(delta > deltaMax) {
    spdlog::debug( "Provided delta ({}) is above the provided range, truncating to maximum value.", delta);
    delta = deltaMax;
  }
  if(delta < deltaMin) {
    spdlog::debug( "Provided delta ({}) is below the provided range, truncating to minimum value.", delta );
    delta = deltaMin;
  }
  std::size_t delta_bucket_index = 0;
  double current_lower_bound = deltaMin;
  double bucket_width = (deltaMax - deltaMin) / discretization;
  bool first = true;
  while(delta > current_lower_bound && delta_bucket_index < discretization) {
    ++delta_bucket_index;
    current_lower_bound += bucket_width;
    first = false;
  }
  return first ? delta_bucket_index : --delta_bucket_index;
}

std::size_t getXBucket( Number x, double x_min, double x_max, double x_interval_size ) {
  assert(x_max > x_min);
  auto num_x_buckets = static_cast<size_t>(ceil((x_max-x_min)/x_interval_size));
  auto x_low  = x_min;
  auto x_high = x_min + x_interval_size;

  for ( std::size_t ix = 0; ix < num_x_buckets; ++ix ) {
    if(x >= x_low && x <= x_high) {
      return ix;
    }
    x_low += x_interval_size;
    x_high += x_interval_size;
  }
  throw std::logic_error("Value out of range");
}

std::size_t getYBucket( Number y, double y_min, double y_max, double y_interval_size ) {
  assert(y_max > y_min);
  auto num_y_buckets = static_cast<size_t>(ceil((y_max-y_min)/y_interval_size));
  auto y_low  = y_min;
  auto y_high = y_min + y_interval_size;

  for ( std::size_t iy = 0; iy < num_y_buckets; ++iy ) {
    if(y >= y_low && y <= y_high) {
      return iy;
    }
    y_low += y_interval_size;
    y_high += y_interval_size;
  }
  throw std::logic_error("Value out of range");}

std::vector<LocPtr> getLocationForTheta(Number theta, std::size_t discretization, const std::vector<LocPtr>& in) {
  std::vector<LocPtr> res;
  std::size_t thetaBucket = getThetaBucket(theta, discretization);
  std::string searchstring = "theta_" + std::to_string(thetaBucket);
  for(auto* l : in) {
    if(l->getName().find(searchstring) != std::string::npos) {
      res.push_back(l);
    }
  }
  return res;
}


} // namespace

