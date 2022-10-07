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

std::vector<locationConditionMap> generateTrainingSets( GeneralRoadSegment& segment, double widening, size_t theta_discretization, size_t trainingAngles, Automaton& atm, double bcVelocity) {


  auto startPoint = segment.getStart();
  auto spacing = widening;
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
  size_t bucket = getThetaBucket(segmentAngle, theta_discretization) - trainingAngles;
  for(auto i=0; i<2*trainingAngles+1; i++) {
    auto rep = getRepresentativeForThetaBucket(bucket, theta_discretization);
    headings.push_back( normalizeAngle(rep));
    bucket++;
  }

  spdlog::info("creating points ({}) and headings ({}) completed", points.size(), headings.size());

  std::vector<locationConditionMap> res;
  res.reserve(points.size());
  auto i = 0;
  for(auto p : points) {
    Point state = Point{p[0],p[1], 0, 0,bcVelocity,0};
    auto locs = getLocationsForState(state, atm);
    locationConditionMap conditionMap;
    for (auto h : headings) {
      state = Point{p[0],p[1], h, 0,bcVelocity,0};
      auto locs_theta = getLocationForTheta(h, theta_discretization, locs);
      for (auto l : locs_theta) {
        conditionMap[l] = hypro::Condition<Number>{widenSample(state, widening, {0,1})};
      }
    }
    res.push_back( conditionMap );
  }

  return res;
}

}

#endif  // SIMPLEXARCHITECTURES_CARTRAINING_H
