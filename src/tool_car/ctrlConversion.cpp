/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 23.06.22.
 */

#include "ctrlConversion.h"

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

std::size_t getThetaBucket(Number theta, std::size_t discretization) {
  if(theta < 0 || theta > 360) {
    throw std::logic_error("Invalid theta-value, must be between 0 and 360");
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
  if(delta > delta_ranges.second || delta < delta_ranges.first) {
    throw std::logic_error("Provided delta is not within the provided range");
  }
  std::size_t delta_bucket_index = 0;
  double current_lower_bound = delta_ranges.first;
  double bucket_width = (delta_ranges.second - delta_ranges.first) / discretization;
  bool first = true;
  while(delta > current_lower_bound && delta_bucket_index < discretization) {
    ++delta_bucket_index;
    current_lower_bound += bucket_width;
    first = false;
  }
  return first ? delta_bucket_index : --delta_bucket_index;
}

} // namespace

