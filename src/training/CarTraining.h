//
// Created by bmaderbacher on 07.10.22.
//

#ifndef SIMPLEXARCHITECTURES_CARTRAINING_H
#define SIMPLEXARCHITECTURES_CARTRAINING_H

#include "../utility/RoadSegment.h"
#include "types.h"
#include "../tool_car/ctrlConversion.h"
#include "../simulation/SamplingUtility.h"

namespace simplexArchitectures {

using locationConditionMap = Automaton::locationConditionMap;

std::vector<locationConditionMap> generateTrainingSets( GeneralRoadSegment& segment, size_t segment_id, double widening, size_t theta_discretization, size_t trainingAngles, Automaton& atm, double bcVelocity) {


  auto startPoint = segment.getStart();
  auto spacing = widening / 2.0;
  size_t numSamples = segment.getSegmentLength() / spacing;
  std::vector<Point> points;
  points.reserve(numSamples+1);
  Point currentSample = startPoint;
  Point nextSampleVector = segment.getNormalHeading() * spacing;
  for(auto i=0; i<numSamples; i++) {
    currentSample = currentSample+nextSampleVector;
    points.push_back(currentSample);
  }
//  points.push_back(segment.getEnd());

  std::vector<double> headings;
  headings.reserve(2*trainingAngles+1);
  double segmentAngle = segment.getSegmentAngle();
  size_t bucket = getThetaBucket(segmentAngle, theta_discretization);
  if (bucket > trainingAngles) {
    bucket = bucket - trainingAngles;
  } else {
    bucket = theta_discretization + bucket - trainingAngles;
  }

  for(auto i=0; i<2*trainingAngles+1; i++) {
    auto rep = getRepresentativeForThetaBucket(bucket, theta_discretization);
    headings.push_back( normalizeAngle(rep));
    bucket++;
  }

  spdlog::info("creating points ({}) and headings ({}) completed", points.size(), headings.size());

  auto location_postfix0 = "segment_"+std::to_string(segment_id)+"_zone_0_warning_C"+std::to_string(segment_id);
  auto location_postfix1 = "segment_"+std::to_string(segment_id)+"_zone_1_warning_C"+std::to_string(segment_id);
  auto location_postfix2 = "segment_"+std::to_string(segment_id)+"_zone_2_warning_C"+std::to_string(segment_id);


  std::vector<LocPtr> locs;
  locs.reserve(theta_discretization*2);
  for(const auto* candidate : atm.getLocations()) {
    if(candidate->getName().find(location_postfix0) != std::string::npos ||
       candidate->getName().find(location_postfix1) != std::string::npos ||
       candidate->getName().find(location_postfix2) != std::string::npos) {
      locs.push_back(candidate);
    }
  }

  std::vector<locationConditionMap> res;
  res.reserve(points.size()*headings.size());
  for(auto p : points) {
    Point state = Point{p[0],p[1], 0, 0,bcVelocity,0};

    for (auto h : headings) {
      state = Point{p[0],p[1], h, 0,bcVelocity,0};
      auto locs_theta = getLocationForTheta(h, theta_discretization, locs);
      for (auto l : locs_theta) {
        locationConditionMap conditionMap;
        conditionMap[l] = hypro::Condition<Number>{widenSample(state, widening, {0,1})};
        res.push_back( conditionMap );
      }
    }
  }

  return res;
}

}

#endif  // SIMPLEXARCHITECTURES_CARTRAINING_H
